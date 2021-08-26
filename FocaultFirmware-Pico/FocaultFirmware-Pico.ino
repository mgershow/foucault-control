#include "pico/stdlib.h"
#include <CircularBuffer.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM303_Accel.h>
#include <Wire.h>
#include <EEPROM.h>
#include <elapsedMillis.h>
#include "pico/multicore.h"
#include "hardware/adc.h"

#define mega 1000000.f
#define micro 0.000001f

#define NUM_CMD_DATA 4
#define NUM_ACT_DATA 4
#define CHAR_BUF_SIZE 128

#define VERSION 6


Adafruit_LIS3MDL lis3mdl = Adafruit_LIS3MDL();
//Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);


bool enableDataTransmission = false;
bool enableADCTransmission = true;
bool enableMagTransmission = true;
bool enableAccTransmission = true;
bool enableCoilTransmission = true;

bool autoFlash = true;

typedef enum {SET_COIL, SET_LED, SET_READY} ActionT;

//typedef enum {NONE, DETVAL, MAGX, MAGY, MAGZ, ACCX, ACCY, ACCZ, COIL} TransmitTypeT;

typedef enum {SYNC, DETVAL, COILI, MAGVEC, ACCVEC} TransmitTypeT;

typedef enum {COIL_ON, BAD_MSG, READY_FOR_CROSSING, AUTO_ON, LED_ON, TRANSMIT_SERIAL } LedMessageTypeT;

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
  // float xmag;
  // float ymag;
} crossingT;


typedef struct {
  uint64_t us;
  ActionT action;
  float data[NUM_ACT_DATA];
} EventT;


CircularBuffer<Reading1T, 1000> analogTransmitFifo;

CircularBuffer<Reading3T, 500> coilTransmitFifo;

CircularBuffer<Reading3T, 500> magTransmitFifo;

CircularBuffer<Reading3T, 100> accelTransmitFifo;

CircularBuffer<CommandT, 200> commandStack;

CircularBuffer<EventT, 1000> eventFifo;
CircularBuffer<EventT, 1000> scratchEventStack;



/**************** PIN CONFIGURATIONS  ***************************/
/********** REVISED FOR PICO 2021 BOARD *****************************/

const int actCoilPin = 9;
const int actLEDPin = 8;
const int gainSet0 = 6;
const int gainSet1 = 7;

const int scl1Pin = 15;
const int sda1Pin = 14;

const int detectorPin = A2;
const int refPin = A1;
const int coilIPin = A0;

const int coil_alarm_num = 1;


const float slopeMult = 50.35f;
const float radian = 57.295779513082321f;
 
const uint32_t agrPeriod = 3333; //us = 300 Hz

/***************************************************************/
/******************* configurations ***************************/
float pulseDuration = 0.008; //seconds
float pulsePhase = 18; // degrees
float hysteresis = -0.1; // volts, - indicates trigger on falling
bool autoFire = true; //whether to auto fire
float retriggerDelay = 0.25; //seconds AFTER pulse delivery

float vref = 1.25;

int verbosity = 100;

const uint16_t numADCToAvg = 200; //rate = 500kHz / 3 / numavgs --> 32 = 5 kHz; 64 = 2.5 kHz  -- in practice rate = 100 kHz / numavgs due to overhead [1 khz = too fast for serial transmission]
const float vscaler = 3.3 / (4096.0 * (float) numADCToAvg); // vref / 4096 * numADCToAvg -- 4096 = 3.3 V; 256 summed readings

const byte EEPROM_ACC_CODE = 123; //if byte at eeprom_acc_address is 123, then magnetometer data has been written
const int EEPROM_ACC_ADDRESS = 900; //arbitrary choice
struct {
  float x = 0;
  float y = 0;
} accZero;
uint8_t numEepromWrites = 0; //prevent writing eeprom more than 255 times per power cycle - avoid accidental fatigue

/********************* state GLOBALS ********************************/



Reading1T lastHigh;
Reading1T lastLow;
Reading1T lastReading;
Reading3T coilReading;

volatile bool readyForCrossing = true;

volatile bool hasMag = false;
volatile bool hasAcc = false;

volatile bool coilState;

float halfPeriod;


crossingT lastCrossing;

bool restarted = true;

byte currentGain;


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
  static bool readDetector = true;
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
    //keep reading the coil for 5 more cycles, approx 10 ms, to get all current integrated and suppress detector feedthrough
    if (coilActive) {
      coilCountdown = 5;
    } else {
      if (coilCountdown > 0) {
        --coilCountdown;
      }else {
        readDetector = true;
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
  
  ++numreads;
  if (numreads == numADCToAvg >> 1) {
    microtime = getLow(getTime());
  }
  if (readDetector) {
    adc_select_input(detectorPin - A0);
    vdetaccum += adc_read();
    adc_select_input(refPin - A0);
    vrefaccum += adc_read();
  } else {
    adc_select_input(coilIPin - A0);
    vcurraccum += adc_read();
  }

  
  
}
//
void setup1 (void) {

  delay(5000);
  adc_init();
  adc_gpio_init(detectorPin);
  adc_gpio_init(refPin);
  adc_gpio_init(coilIPin);
  
}

void loop1(void) {
  analogReadFunctionCore1();
}

/********************  core0 all others ********************************/



void setup() {
  // put your setup code here, to run once:
  setupPins();
  Serial.begin(9600);
  elapsedMillis serialWait;
  int j = 0;

  setLED(255);



  Wire1.setSDA(sda1Pin);
  Wire1.setSCL(scl1Pin);  
  Wire1.begin();
  setGain(0);


  
  setupAGR();



  if (hardware_alarm_is_claimed(coil_alarm_num)) {
    sendMessage("hardware coil alarm is claimed!!!!", 0);
    assert(false);
  }else {
    hardware_alarm_claim(coil_alarm_num);
    hardware_alarm_set_callback(coil_alarm_num, toggleCoil_isr);
  }

  sendMessage("setup complete", 1);
  byte g = readGain();
  sendMessage("gain = " + String(g), 1);

  if (enableDataTransmission) {
    verbosity = -1;
  }
  delay(5000);

}


/********************* hardware control **************************/



void setCoil(bool activate, float duration = -1) {
  coilState = activate;
  rp2040.fifo.push_nb((uint32_t) activate); //tell ADC loop that coil is on
  digitalWrite(actCoilPin, activate);
  if (duration > 0) {
     hardware_alarm_set_target(coil_alarm_num, make_timeout_time_us(duration * mega));
  }
  if (autoFlash) {
    setLED(coilState ? 255 : 0);
  }
}

void setLED (uint8_t level) {
  analogWrite(actLEDPin, level);
}


void setGain(byte g) {
  digitalWrite(gainSet0, bitRead(g, 0));
  digitalWrite(gainSet1, bitRead(g, 1));
  currentGain = g;
}

byte readGain() {
  return currentGain;
}

uint64_t getTime() {
  return  time_us_64 (); 

}

void setLedMessage (LedMessageTypeT msg, bool setting) {
  if (msg == BAD_MSG) {
    digitalWrite(25, setting);
  }
  return; //no leds on current board
  //digitalWrite(indicatorPins[msg], setting);
}

/*************** setup *************************************/

void setupPins() {

  //enable pullups for i2c - these should be present on board, but are missing in prototype design
  pinMode(scl1Pin, INPUT_PULLUP);
  pinMode(sda1Pin, INPUT_PULLUP);
  
  pinMode(gainSet0, OUTPUT);
  pinMode(gainSet1, OUTPUT);
  pinMode(actCoilPin, OUTPUT);
  pinMode(actLEDPin, OUTPUT);

  
}


void setupAGR() {
  //accelerometer initializes to 100 Hz reading rate
  //from adafruit_lsm303_accell.cpp
  // Adafruit_BusIO_Register ctrl1 =
  //    Adafruit_BusIO_Register(i2c_dev, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 1);
  // Enable the accelerometer (100Hz)
  // ctrl1.write(0x57);

//  if (!hasAcc) {
//    hasAcc = accel.begin(0x19, &Wire1);
//    accel.setRange(LSM303_RANGE_2G);
//    accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
//  }

  if (!hasMag) {
    hasMag = lis3mdl.begin_I2C(LIS3MDL_I2CADDR_DEFAULT, &Wire1);
    if (hasMag) {
      lis3mdl.setRange(lis3mdl_range_t::LIS3MDL_RANGE_16_GAUSS);
      lis3mdl.setDataRate(lis3mdl_dataRate_t::LIS3MDL_DATARATE_300_HZ);
      lis3mdl.setOperationMode(lis3mdl_operationmode_t::LIS3MDL_CONTINUOUSMODE);
    }
  }
}




 


void readAccZeroEeprom() {
  byte code;
  EEPROM.get(EEPROM_ACC_ADDRESS, code);
  if (code == EEPROM_ACC_CODE) {
    EEPROM.get(EEPROM_ACC_ADDRESS + sizeof(byte), accZero);
  } else {
    accZero.x = 0;
    accZero.y = 0;
  }
  sendMessage("Accelerometer zero set to " + String(accZero.x) + ", " +  String(accZero.y), 1);

}

void writeAccZeroEeprom() {
  return; // just don't
  if (numEepromWrites >= 255) {
    return;
  }
  numEepromWrites++;
  byte code = EEPROM_ACC_CODE;
  EEPROM.put(EEPROM_ACC_ADDRESS, code);
  EEPROM.put(EEPROM_ACC_ADDRESS + sizeof(byte), accZero);
}
void zeroAccelerometer(float accXZero, float accYZero) {
  accZero.x = accXZero;
  accZero.y = accYZero;
  //   sendMessage("Accelerometer zero set to " + String(accZero.x) + ", " +  String(accZero.y), 1);

  writeAccZeroEeprom();
  sendMessage("Accelerometer zero set to " + String(accZero.x) + ", " +  String(accZero.y), 1);
  //    sendMessage("Accelerometer zero set to " + String(accXZero) + ", " +  String(accYZero), 1);

}

/**************** ISRs **********************************/


void toggleCoil_isr(uint alarm_num) {
  setCoil(!coilState, -1);
}

/*********** polling ********************************/

elapsedMillis loopT;
int ctr = 0;
void loop() {

  pollADC();
  pollAGR();
  pollTransmit();
  pollEvent();
  pollSerial();

}


void setReadyForCrossing(bool r) {
  readyForCrossing = r;
}

bool retrigger = false;

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
  bool isdetector = (bool) reading;

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
    coilReading.y = lastReading.val = det - ref; //coilReading.y stores the last detector value before coil went on
    coilReading.z = 0;
    return 1;
  } else {
    coilReading.us = us;
    coilReading.x = coilAmps;
    coilReading.z += coilAmps*dus; //coil reading.z stores the integral of the coil current over the pulse; y*z/gain = energy in the pulse (in uJ)
    return 2;
  }
  
  
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
      }
    }
    if (lastReading.val < 0) {
      low = true;
      lastLow = lastReading;
      if ((hysteresis) > 0) {
        retrigger = true;
      }
    }
  }

  setLedMessage(AUTO_ON, autoFire);

  if ( retrigger && ((hysteresis < 0 && lastLow.us > lastHigh.us) || (hysteresis > 0 && lastHigh.us > lastLow.us))) {

    float dt = lastLow.us - lastHigh.us;
    float tm = 0.5 * lastLow.us + 0.5 * lastHigh.us;
    float dv = lastLow.val - lastHigh.val;
    float vm = 0.5 * (lastLow.val + lastHigh.val);
    crossingT crossing;
    crossing.us = tm - dv / dt * vm;
    crossing.slope = mega * dv / dt;
    halfPeriod = crossing.us - lastCrossing.us;
    if (!enableDataTransmission) {
      sendMessage("crossing at " + String(crossing.us * micro) + " slope = " + String(crossing.slope) + " halfPeriod = " + String(halfPeriod * micro), 1);
    }
    if (autoFire && halfPeriod < 4 * mega) {
      setFiringAction(crossing.us + halfPeriod * pulsePhase / 180);
    }
    lastCrossing = crossing;
    retrigger = false;
  }

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


bool newAGR(void) {
  static absolute_time_t nextReading = make_timeout_time_us(agrPeriod);
  if (!timePassed(nextReading)) {
    return false;
  }
  if (hasMag) {
    if (!lis3mdl.magneticFieldAvailable()) {
      return false;
    }
  }
  while (timePassed(nextReading)) {
    nextReading = delayed_by_us(nextReading, agrPeriod);
  }
  return true;
  
}

void pollAGR(void) {
  if (!newAGR()) {
    return;
  }
  Reading3T reading;
  reading.us = getTime();
  sensors_event_t event;

//  if (hasAcc) {
//    accel.getEvent(&event);
//    reading.x = event.acceleration.x - accZero.x;
//    reading.y = event.acceleration.y - accZero.y;
//    reading.z = event.acceleration.z;
//    if (enableDataTransmission) {
//      accelTransmitFifo.unshift(reading);
//    }
//  }

  if (hasMag) {
    lis3mdl.getEvent(&event);
    reading.x = event.magnetic.x;
    reading.y = -event.magnetic.y; //lis3mdl is reversed from lis2mdl (or at least board is), so left handed when facing down
    reading.z = event.magnetic.z;
    if (enableDataTransmission) {
      magTransmitFifo.unshift(reading);
    }
  }

}


void pollTransmit() {
    static bool wastransmit = false;
    if (enableDataTransmission && !wastransmit) {
      sendSynchronization();
    }
    wastransmit = enableDataTransmission;
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
  if (!accelTransmitFifo.isEmpty()) {
    setLedMessage(TRANSMIT_SERIAL, true);
    if (enableAccTransmission) {
      sendReading(accelTransmitFifo.pop(), ACCVEC);
    } else {
      accelTransmitFifo.pop();
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
  if (!eventFifo.isEmpty() && doEvent(eventFifo.last())) {
    eventFifo.pop();
  }
}


void pollSerial() {
  processSerialLine();
}

/********** other ****************/

void setFiringAction(double us) {
  restarted = false;
  setReadyForCrossing(false);
  EventT event;
  event.us = us;
  event.action = SET_COIL;
  event.data[0] = 1;
  event.data[1] = pulseDuration;
  addEvent(event);

  event.us = event.us + (pulseDuration + retriggerDelay) * mega;
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
    sendMessage("event set for " + String(event.us * micro) + " action = " + String(event.action) + " data[0] = " + String(event.data[0]) +  " data[1] = " + String(event.data[1], 6), 1);
  }
  if (eventFifo.isEmpty()) {
    eventFifo.unshift(event);
    return;
  }

  //make sure event is inserted in order, so that elements are sorted in order of descending us
  //(last is earliest event; first is latest event)
  while (!eventFifo.isEmpty() && eventFifo.first().us > event.us) {
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
  tbuff[19] = 0;
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
  rv = readLineSerial(buff, CHAR_BUF_SIZE, 500);
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

void parseCommand (CommandT c) {
  EventT event;
  switch (toupper(c.cmd)) {
    case 'G':
      setGain((uint8_t) c.data[0]);
      return;
    case 'C':
      if (c.us <= getTime()) {
        setCoil(true, c.data[0]);
      } else {
        event.us = c.us;
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
        event.us = c.us;
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
        event.us = c.us;
        event.action = SET_LED;
        event.data[0] = c.data[0];
        addEvent(event);
      }
      return;
    case 'A':
      if (c.data[0] > 0) {
        autoFire = true;
        pulseDuration = c.data[0];
        pulsePhase = c.data[1];
        hysteresis = c.data[2];
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
      enableAccTransmission = ((byte) c.data[0]) & ((byte) 4);
      enableCoilTransmission = ((byte) c.data[0]) & ((byte) 8);
      return;
    case 'V':
      verbosity = c.data[0];
      sendMessage("verbosity set to " + String(c.data[0]),1);
      return;
    case 'R':
      Serial.println(VERSION);
      return;
    case 'Z':
      zeroAccelerometer(c.data[0], c.data[1]);
      return;
    case 'F':
      autoFlash = c.data[0];
      return;
    case 'S':
      sendSynchronization();
      sendSynchronization();
    default:
      setLedMessage(BAD_MSG, true);

  }
}

bool doEvent (EventT event) {
  //returns true if event is executed
  if (getTime() < event.us) {
    return false;
  }
  switch (event.action) {
    case SET_COIL:
      setCoil(event.data[0], event.data[1]);
      break;
    case SET_LED:
      setLED(event.data[0]);
      break;
    case SET_READY:
      setReadyForCrossing(event.data[0]);
      break;
  }
  return true;
}

void sendMessage(String msg, int v) {
  if (enableDataTransmission) {
    return;
  }
  if (verbosity >= v) {
    Serial.println(msg);
  }
}
