#include <TeensyTimerTool.h>
using namespace TeensyTimerTool;
#include <CircularBuffer.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Wire.h>
#include <ADC.h>
#include <ADC_util.h>

#define mega 1000000.f
#define micro 0.000001f

#define NUM_CMD_DATA 4
#define NUM_ACT_DATA 4
#define CHAR_BUF_SIZE 128

//#define LEGACY 

Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(1);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
ADC *adc = new ADC(); // adc object

IntervalTimer agrTimer;


bool enableDataTransmission = true;
bool enableADCTransmission = true;
bool enableMagTransmission = true;
bool enableAccTransmission = true;
bool enableCoilTransmission = true;

typedef enum {SET_COIL, SET_LED, SET_READY} ActionT;

typedef enum {NONE, DETVAL, MAGX, MAGY, MAGZ, ACCX, ACCY, ACCZ, COIL} TransmitTypeT;

const int indicatorPins[] = {0,1,2,3,4,5,6,7};

typedef enum {COIL_ON, BAD_MSG, READY_FOR_CROSSING, AUTO_ON, LED_ON, TRANSMIT_SERIAL } LedMessageTypeT;

typedef struct {
  double us;
  float val;
} Reading1T;

typedef struct {
  double us;
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
  double us;
  float slope;
 // float xmag;
 // float ymag;
} crossingT;


typedef struct {
  double us;
  ActionT action;
  float data[NUM_ACT_DATA];
} EventT;

//on teensy LC with 32 averages and everything at VERY_LOW_SPEED, sampling rate is 2 kHz; 200 samples = 100 ms; 20 samples = 10 ms

CircularBuffer<Reading1T, 1000> analogTransmitFifo; 
//using index_t = decltype(analogBuffer)::index_t;

CircularBuffer<Reading1T, 1000> coilTransmitFifo; 

//100 Hz, 10 = 100 ms
CircularBuffer<Reading3T, 100> magTransmitFifo; 

CircularBuffer<Reading3T, 100> accelTransmitFifo; 

CircularBuffer<CommandT, 200> commandStack;

CircularBuffer<EventT, 1000> eventFifo;
CircularBuffer<EventT, 1000> scratchEventStack;



/**************** PIN CONFIGURATIONS  ***************************/

#ifdef LEGACY
  const int coilOffPin = 23;
  const int actCoilPin = 22; //coil control pin, sets current level
  const int actLEDPin = 3;
#else
  const int actCoilPin = 23;
  const int actLEDPin = 22;
#endif

const int cat5171Addr = 44;

const int detectorPin = A1;
const int refPin = A0;


const float slopeMult = 50.35f; 
const float radian = 57.295779513082321f;

/***************************************************************/
/******************* configurations ***************************/
float pulseDuration = 0.008; //seconds
float pulsePhase = 18; // degrees
float hysteresis = -0.1; // volts, - indicates trigger on falling
bool autoFire = true; //whether to auto fire
float retriggerDelay = 0.25; //seconds AFTER pulse delivery

float vref = 1.25;

int verbosity = 2;

unsigned int numADCToAvg = 16;

/********************* state GLOBALS ********************************/

volatile bool newDetector;
volatile bool newAGR;


Reading1T lastHigh;
Reading1T lastLow;
Reading1T lastReading;

volatile bool readyForCrossing = true;

volatile bool hasMag = false;
volatile bool hasAcc = false;

volatile bool coilState;

float halfPeriod;


crossingT lastCrossing;

bool restarted = true;


OneShotTimer coilTimer(GPT1);

volatile double baseTime = 0;
volatile unsigned long lastmicros = 0;
const double microrollover = 4294967295;
/********************* hardware control **************************/

void setLedMessage (LedMessageTypeT msg, bool setting) {
  digitalWrite(indicatorPins[msg], setting);
}

void setCoil(bool activate, float duration = -1) {
  coilState = activate;
  #ifdef LEGACY
    digitalWrite(coilOffPin, !activate);
  #endif
    digitalWrite(actCoilPin, activate);
    setLedMessage(COIL_ON, activate);
    if (duration > 0) {
      coilTimer.trigger(duration*mega);
    }
}

void setLED (uint8_t level) {

  analogWrite(actLEDPin, level);
  analogWrite(indicatorPins[LED_ON], level);
}


void setGain(byte g) {
  Wire.beginTransmission(cat5171Addr);
  Wire.write(byte(0));
  Wire.write(g);
  Wire.endTransmission();
}
byte readGain() {
  Wire.requestFrom(cat5171Addr,1,true);
  return Wire.read();
}

double getTime() {

  noInterrupts();
  bool rollover = micros() < lastmicros;
  if (rollover) {
    baseTime += microrollover;
  }
  lastmicros = micros();
  interrupts();
  if (rollover) {
    sendMessage("micros rollover", 1);
  }

  return baseTime + lastmicros;
}

/*************** setup *************************************/

void setupPins() {
  #ifdef LEGACY
   pinMode(coilOffPin, OUTPUT);
   pinMode (actCoilPin, OUTPUT);
   pinMode (actLEDPin, OUTPUT);
  #endif
  pinMode(detectorPin, INPUT);
  pinMode(refPin, INPUT);
  for (int j = 0; j < 8; ++j) {
    pinMode(indicatorPins[j], OUTPUT);
  }
}

void setupADC() {
  ADC_CONVERSION_SPEED cs = ADC_CONVERSION_SPEED::VERY_LOW_SPEED;
  ADC_SAMPLING_SPEED ss = ADC_SAMPLING_SPEED::VERY_LOW_SPEED;
  uint8_t numavgs = 32;
  uint8_t res = 16;
  
    adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
    adc->adc0->setAveraging(numavgs); // set number of averages
    adc->adc0->setResolution(res); // set bits of resolution
    adc->adc0->setConversionSpeed(cs); // change the conversion speed
    adc->adc0->setSamplingSpeed(ss); // change the sampling speed
    #ifdef ADC_DUAL_ADCS
      adc->adc1->setReference(ADC_REFERENCE::REF_3V3);
      adc->adc1->setAveraging(numavgs); // set number of averages
      adc->adc1->setResolution(res); // set bits of resolution
      adc->adc1->setConversionSpeed(cs); // change the conversion speed
      adc->adc1->setSamplingSpeed(ss); // change the sampling speed
      adc->startSynchronizedContinuous(detectorPin, refPin);
      adc->adc0->enableInterrupts(sync_isr);
    #else
       vref = 3.3/adc->adc0->getMaxValue()*((uint16_t) adc->adc0->analogRead(refPin)); 
       adc->adc0->enableInterrupts(adc0_isr);
       adc->adc0->startContinuous(detectorPin);
    #endif
}

void setupAGR() {
  //accelerometer initializes to 100 Hz reading rate 
  //from adafruit_lsm303_accell.cpp
  // Adafruit_BusIO_Register ctrl1 =
  //    Adafruit_BusIO_Register(i2c_dev, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 1);
  // Enable the accelerometer (100Hz)
  // ctrl1.write(0x57);

  //Serial.println("a");
  if (!hasAcc) {
    hasAcc = accel.begin();
    accel.setRange(LSM303_RANGE_2G);
    accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
  }
 // Serial.println("b");
  if (!hasMag) {
    lis2mdl.enableAutoRange(true);
    hasMag = lis2mdl.begin();  
    if (hasMag) {
      lis2mdl.setDataRate(lis2mdl_rate_t::LIS2MDL_RATE_100_HZ);
    }
  }
 // Serial.println("c");
}

void startAGRTimer() {
    agrTimer.priority(255);
    agrTimer.begin(agr_isr, 10000); //magnetometer updates at 100 Hz, T = 10^4 us
}



void setup() {
  // put your setup code here, to run once:
  setupPins();
  Serial.begin(9600);
  elapsedMillis serialWait;
  int j = 0;
  while(!Serial && serialWait < 10000) {
    setLEDIndicators(j);
    setLED(j);
    j = (j+1) % 256; 
    delay(20);
  }
  setLEDIndicators(0);
  setLED(255);
  
  Wire.begin();
  setGain(0);
  setupADC();
  setupAGR();
  startAGRTimer();

  coilTimer.begin(toggleCoil_isr);
  
  sendMessage("setup complete", 1);
  byte g = readGain();
  sendMessage("gain = " + String(g), 1);

  if (enableDataTransmission) {
    verbosity = -1;
  }
  
}

/**************** ISRs **********************************/


void adc0_isr(void) {
  lastReading.us = getTime();
  lastReading.val = 3.3/adc->adc0->getMaxValue()*((uint16_t) adc->adc0->analogReadContinuous()) - vref;
  newDetector = true;
}

void sync_isr(void) {
  #ifdef ADC_DUAL_ADCS
   lastReading.us = getTime();
   ADC::Sync_result result = adc->readSynchronizedContinuous();
   lastReading.val = 3.3/adc->adc0->getMaxValue()*(result.result_adc0-result.result_adc1); 
   newDetector = true;
  #else
   adc0_isr();
  #endif
   
}

void agr_isr(void) {
  newAGR = true;
}

void toggleCoil_isr(void) {
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
  pollCoil();
 
}

elapsedMillis coilTransmitT;
Reading1T coilReading;
void pollCoil (void) {
   bool trans = !coilReading.val != !coilState;
   if (trans) {
    coilTransmitFifo.unshift(coilReading);    
   }
   coilReading.us = getTime();
   coilReading.val = coilState;
   if (trans || coilTransmitT > 100) {
    coilTransmitFifo.unshift(coilReading);   
    if (coilTransmitT > 100) {
      coilTransmitT = coilTransmitT - 100;
    }
  }
  
}

void setReadyForCrossing(bool r) {
  readyForCrossing = r;
  setLedMessage(READY_FOR_CROSSING, r);
}

bool retrigger = false;
void pollADC (void) {
  if (!newDetector) {
    return;
  }
  newDetector = false;
  analogTransmitFifo.unshift(lastReading);
  //digitalWrite(1, lastReading.val > abs(hysteresis));

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
  
 // digitalWrite(1,high);
//  digitalWrite(2,low);
  if ( retrigger && ((hysteresis < 0 && lastLow.us > lastHigh.us) || (hysteresis > 0 && lastHigh.us > lastLow.us))) {
    
    float dt = lastLow.us - lastHigh.us;
    float tm = 0.5*lastLow.us + 0.5*lastHigh.us;
    float dv = lastLow.val - lastHigh.val;
    float vm = 0.5*(lastLow.val + lastHigh.val);
    crossingT crossing;
    crossing.us = tm - dv/dt*vm;
    crossing.slope = mega*dv/dt;
    halfPeriod = crossing.us - lastCrossing.us;
    sendMessage("crossing at " + String(crossing.us * micro) + " slope = " + String(crossing.slope) + " halfPeriod = " + String(halfPeriod * micro), 1); 
   
    if (autoFire && halfPeriod < 4*mega) {
       setFiringAction(crossing.us + halfPeriod*pulsePhase / 180);
    }
    lastCrossing = crossing;
    retrigger = false;
  }
 
}

elapsedMillis agrT;
void pollAGR(void) {
  if (!newAGR) {
    return;
  }
  newAGR = false;
  Reading3T reading;
  reading.us = getTime();
  sensors_event_t event;

  if (hasAcc) {
    accel.getEvent(&event);
    reading.x = event.acceleration.x;
    reading.y = event.acceleration.y;
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

//  if (!hasMag || !hasAcc && agrT > 1000) {
//    agrT = agrT - 1000;
//    setupAGR(); 
//  }
  
}

void pollTransmit() {
  if (analogTransmitFifo.size() >= numADCToAvg) {
    setLedMessage(TRANSMIT_SERIAL, true);
    double us = 0; 
    double val = 0;
    Reading1T r;
    Reading1T avgreading;
    for (unsigned int j = 0; j < numADCToAvg; ++j) {
      r = analogTransmitFifo.pop();
      us += r.us;
      val += r.val;
    }
    avgreading.us = us / numADCToAvg;
    avgreading.val = val / numADCToAvg;
    if (enableADCTransmission) {
      sendReading1(avgreading, DETVAL);
    }
    return; 
  }
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
  if(!eventFifo.isEmpty() && doEvent(eventFifo.last())) {
    eventFifo.pop();
  }
}

elapsedMillis ledT;

void setLEDIndicators(byte v) {
  for (int j = 0; j < 8; ++j) {
    digitalWrite(indicatorPins[j],bitRead(v,j));
  }
}

void pollLEDIndicators() {
  float phaseFrac = (getTime()-lastCrossing.us)*(180.0/pulsePhase)/halfPeriod;
  if (readyForCrossing) {
    phaseFrac = 0;
  }
  
  if (restarted) {  
    phaseFrac = (((int) ledT)/8000.0);
    if (ledT > 8000) {
      ledT = ledT - 8000;
    }
  }
  for (int j = 0; j < 8; ++j) {
    digitalWrite(indicatorPins[j],j < ceil(phaseFrac*8));
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

  event.us = event.us + (pulseDuration + retriggerDelay)*mega;
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
   sendMessage("event set for " + String(event.us * micro) + " action = " + String(event.action) + " data[0] = " + String(event.data[0]) +  " data[1] = " + String(event.data[1],6), 1); 
   
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


void sendReading1 (Reading1T reading, TransmitTypeT t) {
  sendDataAsText ((byte) t, reading.us, reading.val);
}

void sendReading3 (Reading3T reading, TransmitTypeT xtype) {
  //xtype is MAGX or ACCELX depending on whether mag or accel is sent
  sendDataAsText((byte) xtype, reading.us, reading.x);
  sendDataAsText((byte) xtype+ byte(1), reading.us, reading.y);
  sendDataAsText((byte) xtype+ byte(2), reading.us, reading.z);
}

void sendDataAsText(byte ttype, double us, float data) {
  if (!enableDataTransmission) {
    return;
  }
  Serial.print(ttype);
  Serial.print(" ");
  Serial.print(us*micro, 6);
  Serial.print(" ");
  Serial.println(data, 12); 
}


int readLineSerial(char buff[], int buffersize, unsigned int timeout) {
  int i = 0;
  if (!Serial.available()) {
    return 0;
  }
  elapsedMicros t0;
  while (i < buffersize-1 && (Serial.available() || t0 < timeout)) {
    if (!Serial.available()) {
      continue;
    }
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (i > 0) { //discard newline characters at beginning of string (in case of \n\r)
        buff[i+1] = '\0';
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
    sendMessage("line reading failed",1);
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
  sscanf(buff + wsoff, "%c %lf %f %f %f %f", &c.cmd, &c.us, c.data, c.data + 1, c.data + 2, c.data +3); //change if num_data_bytes changes
  c.us *= mega; //convert from seconds to microseconds
  parseCommand(c);  
}

void parseCommand (CommandT c) {
  EventT event;
  switch(toupper(c.cmd)) {
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
    case 'D': {
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
    }     
    default:
      setLedMessage(BAD_MSG, true);
     
  }
}

bool doEvent (EventT event) {
  //returns true if event is executed
  if (getTime() < event.us) {
    return false;
  }
  switch(event.action) {
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
