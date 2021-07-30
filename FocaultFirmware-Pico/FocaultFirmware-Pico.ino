 
#include <stdio.h>
#include "pico/stdlib.h"
#include <CircularBuffer.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Wire.h>
#include <EEPROM.h>
#include <elapsedMillis.h>
#include "pico/multicore.h"

#define mega 1000000.f
#define micro 0.000001f

#define NUM_CMD_DATA 4
#define NUM_ACT_DATA 4
#define CHAR_BUF_SIZE 128

#define VERSION 5


Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(1);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);


bool enableDataTransmission = true;
bool enableADCTransmission = true;
bool enableMagTransmission = true;
bool enableAccTransmission = true;
bool enableCoilTransmission = true;

bool autoFlash = true;

typedef enum {SET_COIL, SET_LED, SET_READY} ActionT;

typedef enum {NONE, DETVAL, MAGX, MAGY, MAGZ, ACCX, ACCY, ACCZ, COIL} TransmitTypeT;

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

CircularBuffer<Reading1T, 1000> coilTransmitFifo;

CircularBuffer<Reading3T, 100> magTransmitFifo;

CircularBuffer<Reading3T, 100> accelTransmitFifo;

CircularBuffer<CommandT, 200> commandStack;

CircularBuffer<EventT, 1000> eventFifo;
CircularBuffer<EventT, 1000> scratchEventStack;



/**************** PIN CONFIGURATIONS  ***************************/
/********** REVISED FOR PICO 2021 BOARD *****************************/

const int actCoilPin = 12;
const int actLEDPin = 11;
const int gainSet0 = 6;
const int gainSet1 = 7;

const int scl1Pin = 15;
const int sda1Pin = 19;

const int detectorPin = 28;
const int refPin = 27;
const int coilIPin = 26;

const int coil_alarm_num = 1;


const float slopeMult = 50.35f;
const float radian = 57.295779513082321f;
 
const uint32_t agrPeriod = 10000; //us = 100 Hz

/***************************************************************/
/******************* configurations ***************************/
float pulseDuration = 0.008; //seconds
float pulsePhase = 18; // degrees
float hysteresis = -0.1; // volts, - indicates trigger on falling
bool autoFire = true; //whether to auto fire
float retriggerDelay = 0.25; //seconds AFTER pulse delivery

float vref = 1.25;

int verbosity = 0;
const uint8_t numADCToAvg = 64; //rate = 500kHz / 3 / numavgs --> 32 = 5 kHz; 64 = 2.5 kHz 
const float vscaler = 3.3 / (4096 * numADCToAvg); // vref / 4096 * numADCToAvg -- 4096 = 3.3 V; 256 summed readings

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
Reading1T coilReading;

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
  static uint32_t vdetaccum = 0;
  static uint32_t vrefaccum = 0;
  static uint32_t vcurraccum = 0;
  static uint32_t microtime;
  static uint8_t numreads = 0;
  ++numreads;
  if (numreads == numADCToAvg >> 1) {
    microtime = getLow(getTime());
  }
  vdetaccum += analogRead(detectorPin);
  vrefaccum += analogRead(refPin);
  vcurraccum += analogRead(coilIPin);
  if (numreads == numADCToAvg) {
    rp2040.fifo.push(microtime);
    rp2040.fifo.push(vdetaccum);
    rp2040.fifo.push(vrefaccum);
    rp2040.fifo.push(vcurraccum);
    numreads = 0;
  }
}

void setup1 (void) {
  pinMode(detectorPin, INPUT);
  pinMode(refPin, INPUT);
  pinMode(coilIPin, INPUT);
}

void loop1(void) {
  uint64_t t = getTime();
  if (bitRead(t, 0)) { // only read on odd microseconds -- maximum 500 kHz
    analogReadFunctionCore1();
    while (getTime() == t) {
        ; // blank - wait for at least one microsecond to pass
    }
  }
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

}


/********************* hardware control **************************/



void setCoil(bool activate, float duration = -1) {
  coilState = activate;
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
  return; //no leds on current board
  //digitalWrite(indicatorPins[msg], setting);
}

/*************** setup *************************************/

void setupPins() {

  //enable pullups for i2c - these should be present on board, but are missing in prototype design
  pinMode(scl1Pin, INPUT_PULLUP);
  pinMode(sda1Pin, INPUT_PULLUP);

  pinMode(detectorPin, INPUT);
  pinMode(refPin, INPUT);

  pinMode(gainSet0, OUTPUT);
  pinMode(gainSet1, OUTPUT);
}


void setupAGR() {
  //accelerometer initializes to 100 Hz reading rate
  //from adafruit_lsm303_accell.cpp
  // Adafruit_BusIO_Register ctrl1 =
  //    Adafruit_BusIO_Register(i2c_dev, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 1);
  // Enable the accelerometer (100Hz)
  // ctrl1.write(0x57);

  if (!hasAcc) {
    hasAcc = accel.begin(0x19, &Wire1);
    accel.setRange(LSM303_RANGE_2G);
    accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
  }

  if (!hasMag) {
    lis2mdl.enableAutoRange(true);
    hasMag = lis2mdl.begin(0x1E, &Wire1);
    if (hasMag) {
      lis2mdl.setDataRate(lis2mdl_rate_t::LIS2MDL_RATE_100_HZ);
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

//now handled in ADC loop
//void pollCoil (void) {
//  bool trans = !coilReading.val != !coilState;
//  if (trans) {
//    coilTransmitFifo.unshift(coilReading);
//  }
//  coilReading.us = getTime();
//  coilReading.val = coilState;
//  if (trans || coilTransmitT > 100) {
//    coilTransmitFifo.unshift(coilReading);
//    if (coilTransmitT > 100) {
//      coilTransmitT = coilTransmitT - 100;
//    }
//  }
//
//}

void setReadyForCrossing(bool r) {
  readyForCrossing = r;
}

bool retrigger = false;

bool newDetector() {
  if (rp2040.fifo.available() < 4) {
    return false;
  }
  uint64_t us = setLowerHalf(getTime(), rp2040.fifo.pop());
  float det = vscaler*rp2040.fifo.pop();
  float ref = vscaler*rp2040.fifo.pop();
  float coilAmps = vscaler*rp2040.fifo.pop()/1.1; //1.1 V = 1 amp
  lastReading.us = us;
  lastReading.val = det - ref;
  coilReading.us = us;
  coilReading.val = coilAmps;
  return true;
}

void pollADC (void) {

  if (!newDetector()) {
    return;
  }
  analogTransmitFifo.unshift(lastReading);
  coilTransmitFifo.unshift(coilReading);
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
    sendMessage("crossing at " + String(crossing.us * micro) + " slope = " + String(crossing.slope) + " halfPeriod = " + String(halfPeriod * micro), 1);

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

  if (hasAcc) {
    accel.getEvent(&event);
    reading.x = event.acceleration.x - accZero.x;
    reading.y = event.acceleration.y - accZero.y;
    reading.z = event.acceleration.z;
    accelTransmitFifo.unshift(reading);
  }

  if (hasMag) {
    lis2mdl.getEvent(&event);
    reading.x = event.magnetic.x;
    reading.y = event.magnetic.y;
    reading.z = event.magnetic.z;
    magTransmitFifo.unshift(reading);
  }

}

void pollTransmit() {
    if (!analogTransmitFifo.isEmpty()) {
      setLedMessage(TRANSMIT_SERIAL, true);
      if (enableADCTransmission) {
        sendReading1(coilTransmitFifo.pop(), COIL);
      } else {
        coilTransmitFifo.pop();
      }
  
      return;
    }
//  if (analogTransmitFifo.size() >= numADCToAvg) {
//    setLedMessage(TRANSMIT_SERIAL, true);
//    double us = 0;
//    double val = 0;
//    Reading1T r;
//    Reading1T avgreading;
//    for (unsigned int j = 0; j < numADCToAvg; ++j) {
//      r = analogTransmitFifo.pop();
//      us += r.us;
//      val += r.val;
//    }
//    avgreading.us = us / numADCToAvg;
//    avgreading.val = val / numADCToAvg;
//    if (enableADCTransmission) {
//      sendReading1(avgreading, DETVAL);
//    }
//    return;
//  }
  if (!magTransmitFifo.isEmpty()) {
    setLedMessage(TRANSMIT_SERIAL, true);
    if (enableMagTransmission) {
      sendReading3(magTransmitFifo.pop(), MAGX);
    } else {
      magTransmitFifo.pop();
    }
    return;
  }
  if (!accelTransmitFifo.isEmpty()) {
    setLedMessage(TRANSMIT_SERIAL, true);
    if (enableAccTransmission) {
      sendReading3(accelTransmitFifo.pop(), ACCX);
    } else {
      accelTransmitFifo.pop();
    }
    return;
  }
  if (!coilTransmitFifo.isEmpty()) {
    setLedMessage(TRANSMIT_SERIAL, true);
    if (enableCoilTransmission) {
      sendReading1(coilTransmitFifo.pop(), COIL);
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
  sendMessage("event set for " + String(event.us * micro) + " action = " + String(event.action) + " data[0] = " + String(event.data[0]) +  " data[1] = " + String(event.data[1], 6), 1);

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

//byte double float = 13 byes; expanding to double double double would be 24 bytes. 1 MB / 24 bytes = 43,690 readings
//at 1 khz = 43 seconds
//100 MB -> 1.5 hours...
//can go to 2+ hrs with 13 byte alignment
//only storing magnetometer data: could pack as double float float float = 20 bytes @ 100 Hz
//524 seconds / MB --> 100 MB = 14.5 Hrs
//storing magnetometer data as ms (uint) u16 u16 u16 -> 10 bytes --> 29 hours
void sendReading1 (Reading1T reading, TransmitTypeT t) {
  sendDataAsText ((byte) t, reading.us, reading.val);
}

void sendReading3 (Reading3T reading, TransmitTypeT xtype) {
  //xtype is MAGX or ACCELX depending on whether mag or accel is sent
  sendDataAsText((byte) xtype, reading.us, reading.x);
  sendDataAsText((byte) xtype + byte(1), reading.us, reading.y);
  sendDataAsText((byte) xtype + byte(2), reading.us, reading.z);
}

void sendDataAsText(byte ttype, double us, float data) {
  if (!enableDataTransmission) {
    return;
  }
  Serial.print(ttype);
  Serial.print(" ");
  Serial.print(us * micro, 6);
  Serial.print(" ");
  Serial.println(data, 12);
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
  if (verbosity >= v) {
    Serial.println(msg);
  }
}
