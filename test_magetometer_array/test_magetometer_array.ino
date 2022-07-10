#include "pico/stdlib.h"
#include <CircularBuffer.h>
#include <Wire.h>
#include <EEPROM.h>
#include <elapsedMillis.h>
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"
#include <ArduinoJson.h>
#include "LittleFS.h"

#include <MHG_MMC5603NJ_Array.h>
#include <MHG_MMC5603NJ_Arduino_Library.h>
#include <MHG_MMC5603NJ_Arduino_Library_Constants.h>
#include <MHG_MMC5603NJ_IO.h>

#define mega 1000000.f
#define micro 0.000001f

#define NUM_CMD_DATA 4
#define NUM_ACT_DATA 4
#define CHAR_BUF_SIZE 128

#define VERSION 10


MHG_MMC5603NJ_Array mmcarr = MHG_MMC5603NJ_Array(); 


bool enableDataTransmission = false;
bool enableADCTransmission = true;
bool enableMagTransmission = true;
bool enableAccTransmission = true;
bool enableCoilTransmission = true;
bool enableCrossingTransmission = true;

typedef enum {NONE, FIRE, CROSS} FlashT;
FlashT autoFlash = NONE;


typedef enum {SET_COIL, SET_LED, SET_READY} ActionT;

//typedef enum {NONE, DETVAL, MAGX, MAGY, MAGZ, ACCX, ACCY, ACCZ, COIL} TransmitTypeT;

typedef enum {SYNC, DETVAL, COILI, MAGVEC, ACCVEC, CROSSING, MAG0, MAG1, MAG2, MAG3, MAG4, MAG5, MAG6, MAG7} TransmitTypeT;

typedef enum {MAG_WORKS,PICO_ERROR,  AUTO_ON, READY_FOR_CROSSING, BAD_MSG, LED_ON, TRANSMIT_SERIAL, WATCHDOG_REBOOT} LedMessageTypeT;

typedef struct {
  uint64_t us;
  float val;
} Reading1T;

typedef struct {
  uint64_t us;
  float x;
  float y;
  float z;
} Reading3T;



typedef struct {
  char cmd;
  double us;
  float data[NUM_CMD_DATA];
} CommandT;


typedef struct {
  uint64_t us;
  float slope;
  float counter;
  float targetTime;
} crossingT;



typedef struct {
  absolute_time_t us;
  ActionT action;
  float data[NUM_ACT_DATA];
} EventT;


CircularBuffer<Reading1T, 1000> analogTransmitFifo;

CircularBuffer<Reading3T, 500> coilTransmitFifo;

CircularBuffer<multiMagMeasurementT, 500> magTransmitFifo;

CircularBuffer<Reading3T, 10> crossingTransmitFifo;

CircularBuffer<CommandT, 200> commandStack;

CircularBuffer<EventT, 1000> eventFifo;
CircularBuffer<EventT, 1000> scratchEventStack;

CircularBuffer<Reading1T, 100> crossingEstimatorFifo;


/**************** PIN CONFIGURATIONS  ***************************/
/********** REVISED FOR PICO 2022 BOARD *****************************/

const int actCoilPin = 9;
const int actLEDPin = 10;
const int gainSet0 = 6;
const int gainSet1 = 7;
const int gainSet2 = 8;

const int i2cSel0 = 1;
const int i2cSel1 = 2;
const int i2cSel2 = 3;

const int sda0Pin = 4;
const int scl0Pin = 5;


const int scl1Pin = 15;
const int sda1Pin = 14;

const int resetIntegratorPin = 0;

const int detectorPin = A2;
const int refPin = A1;
const int coilIPin = A0;

const int coil_alarm_num = 1;


const float slopeMult = 50.35f;
const float radian = 57.295779513082321f;
 
const uint32_t magPeriod = 3333; //us = 300 Hz

const uint8_t indicatorPins[8] = {16,17,18,19,20,21,22,12};

/***************************************************************/
/******************* configurations ***************************/
float pulseDuration = 0.003; //seconds
float pulsePhase = 40; // degrees
float hysteresis = 0.01; // volts, - indicates trigger on falling
float trimVoltage = 0; // volts - voltage to subtract from reading
bool autoFire = true; //whether to auto fire
float retriggerDelay = 0.25; //seconds AFTER pulse delivery

float vref = 1.25;
int verbosity = 100;
const uint16_t numADCToAvg = 200; //rate < 500kHz / 3 / numavgs --> was 200 (about 800Hz), changed to 100 9/23/21 version 7
const float vscaler = 3.3 / (4096.0 * (float) numADCToAvg); // vref / 4096 * numADCToAvg -- 4096 = 3.3 V; 256 summed readings

const char *configname = "config.txt";

/********************* state GLOBALS ********************************/


Reading1T lastHigh;
Reading1T lastLow;
Reading1T lastReading;
Reading3T coilReading;

volatile bool readyForCrossing = true;

volatile bool hasMag = false;

volatile bool coilState;

float period;


crossingT lastCrossing= {0,0,0,0};
crossingT twoCrossings = {0,0,0,0};

bool restarted = true;

byte gainSetting;

EventT nextEvent;
bool eventCompleted = true;

uint64_t coilOnTime;

/*********************** core1 analog read ***********************/

uint32_t getLow(uint64_t val) {
  return * ((uint32_t *) &val);
}

uint32_t getHigh(uint64_t val) {
  return * (((uint32_t *) (&val))+1);
}

void splitIntoTwo(uint64_t val, uint32_t *low, uint32_t *high) {
    if (low != NULL) {
      *low = * ((uint32_t *) &val);
    }
    if (high != NULL) { 
      *high = * (((uint32_t *) (&val))+1);
    }
}

uint64_t combineIntoOne (uint32_t low, uint32_t high) {
  uint64_t val;
  * ((uint32_t *) &val) = low;
  * (((uint32_t *) (&val))+1) = high;
  return val;
}

uint64_t setLowerHalf (uint64_t val, uint32_t newlow) {
  * ((uint32_t *) &val) = newlow;
  return val;
}

static bool readDetector = true;

void analogReadFunctionCore1 (void) {

  /*     analogReadInternals
   * 
   *     adc_gpio_init(pin);
   *     adc_select_input(pin - A0);
   *     return adc_read();
   */
  static uint32_t vdetaccum = 0;
  static uint32_t vrefaccum = 0;
  static uint32_t vcurraccum = 0;
  static uint32_t microtime;
  static uint16_t numreads = 0;
  static bool coilActive = false;
  static int coilCountdown = 5;
  if (numreads == numADCToAvg) {
  
    rp2040.fifo.push(microtime);
    rp2040.fifo.push((uint32_t) readDetector);
    rp2040.fifo.push(readDetector ? vdetaccum : vcurraccum);
    rp2040.fifo.push(vrefaccum);

    vdetaccum = 0;
    vrefaccum = 0;
    vcurraccum = 0;
    numreads = 0;
    //after coil turns off, read the integral of the current for 10 more cycles, approx 5 ms, to get all current integrated and suppress detector feedthrough
    if (coilActive) {
      coilCountdown = 10;
    } else {
      if (coilCountdown > 0) {
        --coilCountdown;
      }else {
        readDetector = true;
        digitalWrite(resetIntegratorPin, HIGH); //done reading current integral, so reset the integrator
      }
    }
  //  readDetector = !coilActive; //transition to reading detector after transmitting data - ensures coil voltage is not fed through
    
  }
  uint32_t coilState;
  if (rp2040.fifo.pop_nb(&coilState)) {
    coilActive = (bool) coilState;
    
  }
  if (coilActive & readDetector) {
    //if the coil energizes while integrating detector, immediately stop integration and reset
      vdetaccum = 0;
      vrefaccum = 0;
      vcurraccum = 0;
      numreads = 0;
      readDetector = false; 
    }
  
  if (numreads == numADCToAvg >> 1) {
    microtime = getLow(getTime());
  }
  if (readDetector) {
    adc_select_input(detectorPin - A0);
    vdetaccum += adc_read();
    adc_select_input(refPin - A0);
    vrefaccum += adc_read();
    ++numreads;

  } else {
    if (!coilActive) { //wait until coil turns off to start reading integrator
      adc_select_input(coilIPin - A0);
      vcurraccum += adc_read();
      ++numreads;
    }
  }

  
  
}


/********************  core0 all others ********************************/



void setup() {
  // put your setup code here, to run once:
  setupPins();
  setLedMessage(WATCHDOG_REBOOT, watchdog_caused_reboot());
 
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }

  setLED(255);


  Wire.setSDA(sda0Pin);
  Wire.setSCL(scl0Pin);
  Wire.setClock(400000);
  Wire.begin();
  
  setupMAG();



}


/********************* hardware control **************************/



void setCoil(bool activate, float duration = -1) {
  coilState = activate;
  if (activate) {
    digitalWrite(resetIntegratorPin, LOW); //start integration
  }
  rp2040.fifo.push_nb((uint32_t) activate); //tell ADC loop that coil is on
  digitalWrite(actCoilPin, activate);
  if (duration > 0) {
     hardware_alarm_set_target(coil_alarm_num, make_timeout_time_us(duration * mega));
  }
  if (activate) {
    coilOnTime = time_us_64();
  }
  if (autoFlash == FIRE) {
    setLED(coilState ? 255 : 0);
  }
//  setLedMessage(COIL_ON, activate);
}

void setLED (uint8_t level) {
  analogWrite(actLEDPin, level);
  setLedMessage(LED_ON, level); 
}


void setGain(byte g) {
  digitalWrite(gainSet0, bitRead(g, 0));
  digitalWrite(gainSet1, bitRead(g, 1));
  digitalWrite(gainSet2, bitRead(g,2));
  gainSetting = g;
}

byte readGain() {
  return gainSetting;
}

uint64_t getTime() {
  return  time_us_64 (); 

}

void setLedMessage (LedMessageTypeT msg, bool setting) {
  if (msg == BAD_MSG) {
    digitalWrite(25, setting);
  }
  digitalWrite(indicatorPins[msg], setting);
  
}

/*************** setup *************************************/

void setupPins() {

  //enable pullups for i2c - these are present on board, but this is a belt/suspenders approach
  pinMode(scl1Pin, INPUT_PULLUP);
  pinMode(sda1Pin, INPUT_PULLUP);
  pinMode(scl0Pin, INPUT_PULLUP);
  pinMode(sda0Pin, INPUT_PULLUP);
  
  
  pinMode(gainSet0, OUTPUT);
  pinMode(gainSet1, OUTPUT);
  pinMode(gainSet2, OUTPUT);
  pinMode(i2cSel0, OUTPUT);
  pinMode(i2cSel1, OUTPUT);
  pinMode(i2cSel2, OUTPUT);
  
  pinMode(actCoilPin, OUTPUT);
  pinMode(actLEDPin, OUTPUT);
  pinMode(resetIntegratorPin, OUTPUT);

  for (int j = 0; j < 8; ++j) {
    pinMode(indicatorPins[j], OUTPUT);
  }
  
}


void setupMAG() {

  if (!hasMag) {
    hasMag = mmcarr.begin(Wire,8);
    if (hasMag) {
      mmcarr.enableAutomaticSetReset();
      mmcarr.performSetOperation();
      mmcarr.performResetOperation();
    }
  }
  setLedMessage(MAG_WORKS, hasMag);
}




 

/**************** ISRs **********************************/


void toggleCoil_isr(uint alarm_num) {
  setCoil(!coilState, -1);
}

/*********** polling ********************************/

elapsedMillis loopT;
int ctr = 0;
void loop() {
  for (int j = 0; j < 8; ++j) {
    Serial.print("Sensor ");
    Serial.print(j);
    Serial.println(mmcarr.isSensorConnected(j) ? " is connected" : " is not connected");
    Serial.print("Sensor ");
    Serial.print(j);
    Serial.println(mmcarr.isSensorActive(j) ? " passed self-test" : " failed self-test or connection");
  }
  while (!(Serial.available() > 0)) {
    delay(100);
  }
  while (Serial.available() > 0) {
    Serial.read();
  }
  Serial.println("reinitializing");
  mmcarr.reinitialize();
  delay(10);

}


void setReadyForCrossing(bool r) {
  readyForCrossing = r;
  setLedMessage(READY_FOR_CROSSING, r);
}

bool retrigger = false;

//needs redo?
uint8_t newDetector() {
  if (rp2040.fifo.available() < 4) {
    return 0;
  }
  uint32_t reading;
  bool valid = true;
  valid = valid && rp2040.fifo.pop_nb(&reading);
  uint64_t us = setLowerHalf(getTime(), reading);
  static uint64_t lastus = 0;
  valid = valid && rp2040.fifo.pop_nb(&reading);
  static bool wasdetector = true;
  
  bool isdetector = (bool) reading;
  if (wasdetector && !isdetector) {
    lastus = coilOnTime;
  }
  wasdetector = isdetector;

  valid = valid && rp2040.fifo.pop_nb(&reading);
  float det;
  float coilAmps; //1.1 V = 1 amp

  if (isdetector) {
    det = vscaler*reading;
  } else {
    coilAmps = vscaler*reading/1.1; //1.1 V = 1 amp
  }
 
  valid = valid && rp2040.fifo.pop_nb(&reading);
  float ref = vscaler*reading;
  float dus = us - lastus;
  lastus = us;
  if (!valid) {
    return 0;
  }
  if (isdetector) {
    lastReading.us = us;
    coilReading.y = lastReading.val = det - ref - trimVoltage; //coilReading.y stores the last detector value before coil went on
    coilReading.z = 0;
    return 1;
  } else {
    coilReading.us = us;
    coilReading.x = coilAmps;
    coilReading.z += coilAmps*dus; //coil reading.z stores the integral of the coil current over the pulse; y*z/gain = energy in the pulse (in uJ)
    return 2;
  }
  
  
}

Reading3T crossingToReading(crossingT c) {
   Reading3T r = {c.us, c.slope, c.counter, c.targetTime};
   return r;
}

void pollADC (void) {

  uint8_t whichread = 0;
  if (! (whichread = newDetector())) { //single = intentional
    return;
  }
  if (enableDataTransmission) {
    if (whichread == 1) {
      analogTransmitFifo.unshift(lastReading);
    } else {
      coilTransmitFifo.unshift(coilReading);
    }
  }
  bool high = false;
  bool low = false;
  if (readyForCrossing && abs(lastReading.val) > abs(hysteresis)) {
    if (lastReading.val > 0) {
      high = true;
      lastHigh = lastReading;
      if ((hysteresis) < 0) {
        retrigger = true;
        crossingEstimatorFifo.clear();
      }
    }
    if (lastReading.val < 0) {
      low = true;
      lastLow = lastReading;
      if ((hysteresis) > 0) {
        retrigger = true;
        crossingEstimatorFifo.clear();
      }
    }
  }

  if (retrigger && abs(lastReading.val) < abs(hysteresis)) {
    crossingEstimatorFifo.push(lastReading);
  }

  

  setLedMessage(AUTO_ON, autoFire);

  static bool crossing_led_on;

  if ( retrigger && ((hysteresis < 0 && lastLow.us > lastHigh.us) || (hysteresis > 0 && lastHigh.us > lastLow.us))) {

//    float dt = lastLow.us - lastHigh.us;
//    float tm = 0.5 * lastLow.us + 0.5 * lastHigh.us;
//    float dv = lastLow.val - lastHigh.val;
//    float vm = 0.5 * (lastLow.val + lastHigh.val);
    crossingT crossing = calculateCrossing();
    crossing.counter = lastCrossing.counter + 1;
//    crossing.us = tm - dv / dt * vm;
//    crossing.slope = mega * dv / dt;
    period = crossing.us - twoCrossings.us;
    if (!enableDataTransmission) {
      sendMessage("crossing at " + String(crossing.us * micro) + " slope = " + String(crossing.slope) + " period = " + String(period * micro), 1);
    }
    if (autoFire && period < 8 * mega) {
      crossing.targetTime = crossing.us + period * pulsePhase / 360 - pulseDuration*mega/2;
      setFiringAction(toTimeStamp(crossing.targetTime));
    }
    crossingTransmitFifo.push(crossingToReading(crossing));
    twoCrossings = lastCrossing;
    lastCrossing = crossing;
    retrigger = false;
    if (autoFlash == CROSS) {
      crossing_led_on = !crossing_led_on;
      setLED (crossing_led_on ? 255 : 0);
    }
  }

}

crossingT calculateCrossing() {
  //find t = mv + b -- intercept is crossing location: 1/m is the dv/dt slope
  crossingT crossing = {0,0,0,0};
  int n = crossingEstimatorFifo.size();
  float xx = 0, xy = 0, x = 0, y = 0;
  float t,v,m,b,d;
  uint64_t t0 = crossingEstimatorFifo[0].us;
  while(!crossingEstimatorFifo.isEmpty()){
    Reading1T r= crossingEstimatorFifo.pop();
    t = (float) (r.us - t0);
    v = r.val;
    xx += v*v;
    y += t;
    x += v;
    xy += v*t;
  }
  d = (n*xx - x*x);
  b = (xx*y - x*xy)/d;
  crossing.us = (uint64_t) (b + t0); 
  //m = (n*xy - x*y)/d;
  crossing.slope = mega*d/(n*xy - x*y); //dv/dt in volts / second
  return crossing;
}

/*
 * static int64_t absolute_time_diff_us ( absolute_time_t   from,
absolute_time_t   to 
) 
(positive if to is after from except in case of overflow)
 */
bool timePassed (absolute_time_t targetTime) {
  return absolute_time_diff_us(get_absolute_time(), targetTime) <= 0;
}


bool timePassed (uint64_t targetTime) {
  absolute_time_t t;
  update_us_since_boot(&t, targetTime);
  return timePassed(t);
}


void pollMAG(void) {
  bool dataready;
  multiMagMeasurementT measurement;
  if (readDetector) { //don't read magnetometer when coil is energized
    mmcarr.measurementCycle(getTime(), dataready, measurement);
    if (dataready) {
      //todo: semaphore/mutex if running on core 1
        magTransmitFifo.unshift(measurement);
    }
  }
}



void pollTransmit() {
    static bool wastransmit = false;
    if (enableDataTransmission && !wastransmit) {
      sendSynchronization();
    }
    wastransmit = enableDataTransmission;

    if (!crossingTransmitFifo.isEmpty()) {
      
      setLedMessage(TRANSMIT_SERIAL, true);
      if (enableCrossingTransmission) {
        sendReading(crossingTransmitFifo.pop(), CROSSING);
      } else {
        crossingTransmitFifo.pop();
      }
    }
    
    if (!analogTransmitFifo.isEmpty()) {
      setLedMessage(TRANSMIT_SERIAL, true);
      if (enableADCTransmission) {
        sendReading(analogTransmitFifo.pop(), DETVAL);
      } else {
        analogTransmitFifo.pop();
      }
  
      return;
    }

  if (!magTransmitFifo.isEmpty()) {
    setLedMessage(TRANSMIT_SERIAL, true);
    if (enableMagTransmission) {
      sendReading(magTransmitFifo.pop(), MAGVEC);
    } else {
      magTransmitFifo.pop();
    }
    return;
  }
  
  if (!coilTransmitFifo.isEmpty()) {
    setLedMessage(TRANSMIT_SERIAL, true);
    if (enableCoilTransmission) {
      sendReading(coilTransmitFifo.pop(), COILI);
    } else {
      coilTransmitFifo.pop();
    }

    return;
  }
  setLedMessage(TRANSMIT_SERIAL, false);
}

void pollEvent() {
  if (!eventFifo.isEmpty() && eventCompleted) {
    if (scheduleEvent(eventFifo.last())) {
      
      eventFifo.pop();
    }
  }
}


void pollSerial() {
  processSerialLine();
}

/********** other ****************/

void setFiringAction(absolute_time_t us) {
  restarted = false;
  setReadyForCrossing(false);
  EventT event;
  event.us = us;
  event.action = SET_COIL;
  event.data[0] = 1;
  event.data[1] = pulseDuration;
  addEvent(event);

  event.us = delayed_by_us(us, (pulseDuration + retriggerDelay) * mega);
  event.action = SET_READY;
  event.data[0] = 1;
  event.data[1] = 0;
  addEvent(event);


}


//unshift() adds to the head
//first() returns the element at head
//push() adds to the tail
//data retrieval can be performed at tail via a pop() operation or from head via an shift()

void addEvent (EventT event) {
  if (!enableDataTransmission) {
    sendMessage("event set for " + String(to_us_since_boot(event.us) * micro) + " action = " + String(event.action) + " data[0] = " + String(event.data[0]) +  " data[1] = " + String(event.data[1], 6), 1);
  }
  if (eventFifo.isEmpty()) {
    eventFifo.unshift(event);
    return;
  }

  //make sure event is inserted in order, so that elements are sorted in order of descending us
  //(last is earliest event; first is latest event)
  //static int64_t absolute_time_diff_us  ( absolute_time_t   from, absolute_time_t   to )
  //positive if to is after from except in case of overflow   

  while (!eventFifo.isEmpty() && absolute_time_diff_us(event.us, eventFifo.first().us) > 0) {
    scratchEventStack.push(eventFifo.shift());
  }
  eventFifo.unshift(event);
  while (!scratchEventStack.isEmpty()) {
    eventFifo.unshift(scratchEventStack.pop());
  }
}

void sendSynchronization() {
  float data[3] = {0,0,0};
  sendBinaryData (0, 0, data);
}

void sendReading (Reading1T reading, TransmitTypeT t) {
  float data[3] = {0,0,0};
  data[0] = reading.val;
  sendBinaryData ((uint8_t) t, reading.us, data);
}
void sendReading (Reading3T reading, TransmitTypeT t) {
  float data[3];
  data[0] = reading.x;
  data[1] = reading.y;
  data[2] = reading.z;
  sendBinaryData ((uint8_t) t, reading.us, data);
}

void sendReading (multiMagMeasurementT reading, TransmitTypeT t) {
  float data[3];
  static const TransmitTypeT magtype[8] = {MAG0, MAG1, MAG2, MAG3, MAG4, MAG5, MAG6, MAG7};
  for (int j = 0; j < MAX_SENSORS; ++j) {
    if (reading.sensorOnline[j]){
      data[0] = reading.x[j];
      data[1] = reading.y[j];
      data[2] = reading.z[j];
      sendBinaryData (magtype[j], reading.us, data);
    }
  }
}


void sendBinaryData(uint8_t ttype, uint64_t us, float data[]) {
  //ttype is type of transmission
  //us is time in microseconds, lowest 48 bits sent - rollover in 9 years

  if (!enableDataTransmission) {
    return;
  }
  
  uint8_t tbuff[20];
  tbuff[0] = ttype;
  memcpy(tbuff + 1, ((uint8_t *) &us), 6); //raspberry pi pico stores data little-endian (lsb first)
  memcpy(tbuff + 7, ((uint8_t *) data), 12);
  tbuff[19] = 0; //checksum
  for (int j = 0; j < 19; ++j) {
    tbuff[19] = tbuff[19] ^ tbuff[j];
  }
  Serial.write(tbuff, 20);
}


int readLineSerial(char buff[], int buffersize, unsigned int timeout) {
  int i = 0;
  if (!Serial.available()) {
    return 0;
  }
  elapsedMicros t0;
  while (i < buffersize - 1 && (Serial.available() || t0 < timeout)) {
    if (!Serial.available()) {
      continue;
    }
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (i > 0) { //discard newline characters at beginning of string (in case of \n\r)
        buff[i + 1] = '\0';
        return i; //positive return value
      }
    } else {
      buff[i++] = c;
    }
  }
  return -i; //0 if nothing read, negative value if read was incomplete
}


void processSerialLine() {
  char buff[CHAR_BUF_SIZE];
  int rv;
  rv = readLineSerial(buff, CHAR_BUF_SIZE, 500); //changed from 500 to 500k for debugging
  if (rv < 0) {
    sendMessage("line reading failed", 1);
    setLedMessage(BAD_MSG, true);
    return;
  }
  if (rv == 0) {
    return;
  }
  setLedMessage(BAD_MSG, false);
  restarted = false; //received a command
  int wsoff = 0;
  for (wsoff = 0; isspace(buff[wsoff]) && wsoff < rv; ++wsoff); //get rid of leading whitespace
  CommandT c;
  c.data[0] = c.data[1] = c.data[2] = c.data[3] = 0;
  sscanf(buff + wsoff, "%c %lf %f %f %f %f", &c.cmd, &c.us, c.data, c.data + 1, c.data + 2, c.data + 3); //change if num_data_bytes changes
  c.us *= mega; //convert from seconds to microseconds
  parseCommand(c);
}

absolute_time_t toTimeStamp(uint64_t us) {
  absolute_time_t t;
  update_us_since_boot(&t, us);
  return t;
}

void parseCommand (CommandT c) {
  EventT event;
  switch (toupper(c.cmd)) {
    case 'G': //gain and trim voltage
      setGain((uint8_t) c.data[0]);
      setTrimVoltage(c.data[1]);
      return;
    case 'C':
      if (c.us <= getTime()) {
        setCoil(true, c.data[0]);
      } else {
        event.us = toTimeStamp(c.us);
        event.action = SET_COIL;
        event.data[0] = 1;
        event.data[1] = c.data[0];
        addEvent(event);
      }

      return;
    case 'D':
      if (c.us <= getTime()) {
        setCoil(false);
      } else {
        event.us = toTimeStamp(c.us);
        event.action = SET_COIL;
        event.data[0] = 0;
        event.data[1] = -1;
        addEvent(event);
      }
      return;
    case 'L':
      if (c.us <= getTime()) {
        setLED(c.data[0]);
      } else {
        event.us = toTimeStamp(c.us);
        event.action = SET_LED;
        event.data[0] = c.data[0];
        addEvent(event);
      }
      return;
    case 'A':
      pulseDuration = c.data[0];
      pulsePhase = c.data[1];
      hysteresis = c.data[2];
      if (c.data[0] > 0) {
        autoFire = true;
        sendMessage("auto enabled", 1);
      } else {
        autoFire = false;
        sendMessage("auto disabled", 1);
      }
      return;
    case 'T':
      enableDataTransmission = c.data[0] > 0;
      enableADCTransmission = ((byte) c.data[0]) & ((byte) 1);
      enableMagTransmission = ((byte) c.data[0]) & ((byte) 2);
     // enableAccTransmission = ((byte) c.data[0]) & ((byte) 4);
      enableCoilTransmission = ((byte) c.data[0]) & ((byte) 8);
      enableCrossingTransmission = ((byte) c.data[0]) & ((byte) 16);
      return;
    case 'V':
      verbosity = c.data[0];
      sendMessage("verbosity set to " + String(c.data[0]),1);
      return;
    case 'R':
      Serial.println(VERSION);
      return;
    case 'F':
      autoFlash = (FlashT) c.data[0];
      return;
    case 'X': //changed from S
      sendSynchronization();
      sendSynchronization();
      return;
    case 'Q':
      resetQueuesAndTimers();
      return;
    case 'S':
      saveConfiguration();
      return;
    case '?':
      sendStatusMessage();
      return;  
    default:
      setLedMessage(BAD_MSG, true);

  }
}

void sendStatusMessage() {
  StaticJsonDocument<144> doc;

  doc["pulseDuration"] = pulseDuration;
  doc["pulsePhase"] = pulsePhase;
  doc["hysteresis"] = hysteresis;
  doc["trimVoltage"] = trimVoltage;
  doc["autoFire"] = autoFire;
  doc["retriggerDelay"] = retriggerDelay;
  doc["gainSetting"] = gainSetting;
  doc["hasMag"] = hasMag;
  doc["autoFlash"] = (uint8_t) autoFlash;
  if (serializeJson(doc, Serial) == 0) {
    sendMessage("serialization failed", 0);
    setLedMessage(PICO_ERROR, true);
  }
}


int64_t eventCallback(alarm_id_t id, void *data) {
  EventT *event = (EventT *) data;
  switch (event->action) {
    case SET_COIL:
      setCoil(event->data[0], event->data[1]);
      break;
    case SET_LED:
      setLED(event->data[0]);
      break;
    case SET_READY:
      setReadyForCrossing(event->data[0]);
      break;
  }
  eventCompleted = true;
  return 0;
}

/*static alarm_id_t add_alarm_at  ( absolute_time_t   time,
alarm_callback_t  callback,
void *  user_data,
bool  fire_if_past 
) 
*/
bool scheduleEvent(EventT event) {
  nextEvent = event;
  int rv = (add_alarm_at(event.us, eventCallback, &nextEvent, true) >= 0);
  if (rv == 0) {
    eventCompleted = true;
  }
  if (rv > 0) {
    eventCompleted = false; 
  }
  return (rv >= 0);
}


void sendMessage(String msg, int v) {
  if (enableDataTransmission) {
    return;
  }
  if (verbosity >= v) {
    Serial.println(msg);
  }
}

void setTrimVoltage(float tv) {
  trimVoltage = tv;
}

void resetQueuesAndTimers() {
  
  analogTransmitFifo.clear();
  coilTransmitFifo.clear();
  magTransmitFifo.clear();
  crossingTransmitFifo.clear();
  commandStack.clear();
  eventFifo.clear();
  scratchEventStack.clear();
  setCoil(0);
  readyForCrossing = true;
  eventCompleted = true;
  
  
}



void saveConfiguration () {
  //adapted from arduinojson.org/v6/example/config
  //and arduino-pico.readthedocs.io/en/latest/fs.hmtl
  LittleFS.remove(configname);
  File f = LittleFS.open(configname, "w");
  if (!f) {
    sendMessage("file creation failed", 0);
    setLedMessage(PICO_ERROR, true);
    return;
  }

 /*
 {
    "pulseDuration": 0.003,
    "pulsePhase": 40,
    "hysteresis": 0.01,
    "trimVoltage": 0,
    "autoFire": true,
    "retriggerDelay": 0.25
  }
  arduinojson.org/v6/assistant says 48 bytes required
  */
  StaticJsonDocument<144> doc;

  doc["pulseDuration"] = pulseDuration;
  doc["pulsePhase"] = pulsePhase;
  doc["hysteresis"] = hysteresis;
  doc["trimVoltage"] = trimVoltage;
  doc["autoFire"] = autoFire;
  doc["retriggerDelay"] = retriggerDelay;
  doc["autoFlash"] = (uint8_t) autoFlash;
  doc["gainSetting"] = gainSetting;
  if (serializeJson(doc, f) == 0) {
    sendMessage("serialization failed", 0);
    setLedMessage(PICO_ERROR, true);
  }
  f.close();  
}


int loadConfiguration () {
  //adapted from arduinojson.org/v6/example/config
  //and arduino-pico.readthedocs.io/en/latest/fs.hmtl
  File f = LittleFS.open(configname, "r");
  if (!f) {
    sendMessage("file load failed", 0);
    return 1;
  }

 /*
 {
    "pulseDuration": 0.003,
    "pulsePhase": 40,
    "hysteresis": 0.01,
    "trimVoltage": 0,
    "autoFire": true,
    "retriggerDelay": 0.25
  }
  arduinojson.org/v6/assistant says 48 bytes required
  */
  StaticJsonDocument<192> doc;

  DeserializationError error = deserializeJson(doc, f);
  if (error) {
     sendMessage("deserialization failed", 0);
     setLedMessage(PICO_ERROR, true);
     return 2;
  }
  pulseDuration = doc["pulseDuration"] | pulseDuration;
  pulsePhase = doc["pulsePhase"] | pulsePhase;
  hysteresis = doc["hysteresis"] | hysteresis;
  trimVoltage = doc["trimVoltage"] | trimVoltage;
  autoFire = doc["autoFire"] | autoFire;
  retriggerDelay = doc["retriggerDelay"] | retriggerDelay;
  autoFlash = (FlashT) (doc["autoFlash"] | autoFlash);
  gainSetting = doc["gainSetting"] | gainSetting;
  sendMessage("loaded configuration", 1);
  if (verbosity > 1) {
    serializeJson(doc, Serial);
  }

  f.close();  
  return 0;
}
