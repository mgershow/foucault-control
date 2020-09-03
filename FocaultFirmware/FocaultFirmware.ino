
#include <CircularBuffer.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Wire.h>
#include <ADC.h>
#include <ADC_util.h>

#define mega 1000000.f
#define micro 0.000001f

#define NUM_CMD_DATA_BYTES 4
#define CHAR_BUF_SIZE 128

#define LEGACY 

Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(1);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
ADC *adc = new ADC(); // adc object

IntervalTimer agrTimer;

typedef enum {SET_COIL, SET_LED, SET_READY} ActionT;

typedef enum {NONE, DETVAL, MAGX, MAGY, MAGZ, ACCX, ACCY, ACCZ} TransmitTypeT;

typedef struct {
  unsigned long us;
  float val;
} Reading1T;

typedef struct {
  unsigned long us;
  float x;
  float y;
  float z;
} Reading3T;

typedef struct {
  char cmd;
  unsigned long us;
  int data[NUM_CMD_DATA];
} CommandT;


typedef struct {
  float us;
  float slope;
 // float xmag;
 // float ymag;
} crossingT;


typedef struct {
  unsigned long us;
  ActionT action;
  int data;
} EventT;

//on teensy LC with 32 averages and everything at VERY_LOW_SPEED, sampling rate is 2 kHz; 200 samples = 100 ms; 20 samples = 10 ms

CircularBuffer<Reading1T, 100> analogTransmitFifo; 
//using index_t = decltype(analogBuffer)::index_t;

//100 Hz, 10 = 100 ms
CircularBuffer<Reading3T, 10> magTransmitFifo; 

CircularBuffer<Reading3T, 10> accelTransmitFifo; 

CircularBuffer<CommandT, 20> commandStack;

CircularBuffer<EventT, 20> eventFifo;

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

/***************************************************************/
/******************* configurations ***************************/
float pulseDuration = 0.01; //seconds
float pulsePhase = 15; // degrees
float hysteresis = -0.025; // volts, - indicates trigger on falling
bool autoFire = true; //whether to auto fire
float retriggerDelay = 0.05; //seconds AFTER pulse delivery

float halfPeriod;


/********************* GLOBALS ********************************/

volatile bool newDetector;
volatile bool newAGR;

float vref = 1.25;

volatile Reading1T lastHigh;
volatile Reading1T lastLow;
volatile Reading1T lastReading;

volatile bool readyForCrossing = true;

volatile bool hasMag = false;
volatile bool hasAcc = false;

const float slopeMult = 50.35f; 
const float radian = 57.295779513082321f;

crossingT lastCrossing;
/********************* hardware control **************************/

void activateCoil(bool activate) {
  #ifdef LEGACY
    digitalWrite(coilOffPin, !activate);
  #endif
    digitalWrite(actCoilPin, activate);
}

void setLED (uint8_t level) {
//  if (level == 0) {
//    digitalWrite(actLEDPin, LOW);
//    return;
//  }
//  if (level == 255) {
//    digitalWrite(actLEDPin, HIGH);
//    return;
//  }
  analogWrite(actLEDPin, level);
}

void setupPins() {
  #ifdef LEGACY
   pinMode(coilOffPin, OUTPUT);
   pinMode (actCoilPin, OUTPUT);
   pinMode (actLEDPin, OUTPUT);
  #endif
  pinMode(detectorPin, INPUT);
  pinMode(refPin, INPUT);
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

  if (!hasAcc) {
    hasAcc = accel.begin();
    accel.setRange(LSM303_RANGE_2G);
    accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
  }
  if (!hasMag) {
    lis2mdl.enableAutoRange(true);
    hasMag = lis2mdl.begin();  
    lis2mdl.setDataRate(lis2mdl_rate_t::LIS2MDL_RATE_100_HZ);
  }
}

void startAGRTimer() {
    agrTimer.priority(255);
    agrTimer.begin(agr_isr, 10000); //magnetometer updates at 100 Hz, T = 10^4 us
}

void setGain(byte g) {
  Wire.beginTransmission(cat5171Addr);
  Wire.write(byte(0));
  Wire.write(g);
  Wire.endTransmission();
}

/**************** ISRs **********************************/


void acd0_isr(void) {
  lastADCReading.us = micros();
  lastADCReading.val = 3.3/adc->adc0->getMaxValue()*((uint16_t) adc->adc0->analogReadContinuous()) - vref;
  newDetector = true;
}

void sync_isr(void) {
   lastADCReading.us = micros();
   ADC::Sync_result result = adc->readSynchronizedContinuous();
   lastADCReading.val = 3.3/adc->adc0->getMaxValue()*(result.result_adc0-result.result_adc1); 
   newDetector = true;
}

void agr_isr(void) {
  newAGR = true;
}

/*********** polling ********************************/

void pollADC (void) {
  if !newDetector {
    return;
  }
  newDetector = false;
  analogTransmitBuffer.unshift(lastReading);
  if (lastReading > abs(hysteresis)) {
    lastHigh = lastReading;
  }
  if (lastReading < -abs(hysteresis)) {
    lastLow = lastReading;
  }
  if (readyForCrossing && hysteresis < 0 && lastLow.us > lastHigh.us || hysteresis > 0 && lastHigh.us > lastLow.us) {
    float dt = lastLow.us - lastHigh.us;
    float tm = 0.5*lastLow.us + 0.5*lastHigh.us;
    float dv = lastLow.val - lastHigh.val;
    float vm = 0.5*(lastLow.val + lastHigh.val);
    crossingT crossing;
    crossing.us = tm - dv/dt*vm;
    crossing.slope = dv/dt;
    halfPeriod = crossing.us - lastCrossing.us;
    if (autoFire && halfPeriod < 4*mega) {
       setFiringAction(crossing.us + halfPeriod*pulsePhase / 180);
    }
    lastCrossing = crossing;
  }
 
}

void pollAGR(void) {
  if !newAGR {
    return;
  }
  newAGR = false;
  Reading3T reading;
  reading.us = us;

  if (hasAcc) {
    sensors_event_t event;
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

  if (!hasMag || !hasAcc) {
    setupAGR(); //could just call setupAGR because of the checks inside
  }
  
}

void pollTrasmit() {
  if (!analogTransmitFifo.isEmpty()) {
    sendReading1(analogTransmitFifo.pop(), DETVAL); 
  }
  if (!magTransmitFifo.isEmpty()) {
    sendReading3(magTransmitFifo.pop(), MAGX); 
  }
  if (!accelTransmitFifo.isEmpty()) {
    sendReading3(accelTransmitFifo.pop(), ACCX); 
  }
}

void pollEvent() {
  if(!eventFifo.isEmpty() && doEvent(eventFifo.last())) {
    eventFifo.pop();
  }
}


/********** other ****************/

void setFiringAction(unsigned long us) {
  readyForCrossing = false;
  EventT event;
  event.us = us;
  event.action = SET_COIL;
  event.data = 1;
  addEvent(event);
  event.us = event.us + pulseDuration*mega;
  event.data = 0;
  addEvent(event);
  event.us = event.us + retriggerDelay*mega;
  event.action = SET_READY;
  event.data = 1;
  addEvent(event);
}

void addEvent (EventT event) {
  eventFifo.unshift(event); //todo insert sorted by time
}


void sendReading1 (Reading1T reading, TransmitTypeT t) {
  sendDataAsText (t, reading.us, reading.val);
}

void sendReading3 (Reading3T reading, TransmitTypeT xtype) {
  //xtype is MAGX or ACCELX depending on whether mag or accel is sent
  sendDataAsText(xtype, reading.us, reading.x);
  sendDataAsText(ytype, reading.us, reading.y);
  sendDataAsText(ztype, reading.us, reading.z);
}

void sendDataAsText(TransmitTypeT t, unsigned long us, float data) {
  Serial.print((byte) t);
  Serial.print(" ");
  Serial.print(us, DEC);
  Serial.print(" ");
  Serial.println(data, 8); 
}

void processSerialLine() {
  char buff[CHAR_BUF_SIZE];
  int rv = readLineSerial(buff, CHAR_BUF_SIZE, 500);
  if (rv < 0) {
    Serial.println("line reading failed");
    return; 
  }
  if (rv == 0) {
    return;
  }
  int wsoff = 0;
  for (wsoff = 0; isspace(buff[wsoff]) && wsoff < rv; ++wsoff); //get rid of leading whitespace
  CommandT c;
  sscanf(buff + wsoff, "%c %ul %i %i %i %i", c.cmd, c.us, c.data, c.data + 1, c.data + 2, c.data +3); //change if num_data_bytes changes
  parseCommand(c);  
}

void parseCommand (CommandT c) {
  EventT event;
  unsigned long us;
  switch(toupper(c.cmd)) {
    case 'G':
      setGain((uint8_t) c.data[0]);
      return;
    case 'C':
      if (c.us <= micros()) {
        activateCoil(true);
        us = micros();
      } else {
        event.us = c.us;
        us = c.us;
        event.action = SET_COIL;
        event.data = 1;
        addEvent(event);
      }
      if (c.data[0] > 0) {
        event.us = us + c.data[0];
        event.action = SET_COIL;
        event.data = 0;
      }
      return;
    case 'D': {
      if (c.us <= micros()) {
        activateCoil(false);
      } else {
        event.us = c.us;
        event.action = SET_COIL;
        event.data = 0;
        addEvent(event);
      }
      return;
    case 'L':
      if (c.us <= micros()) {
        setLED(c.data[0]);
        us = micros();
      } else {
        event.us = c.us;
        event.action = SET_LED;
        event.data = c.data[0];
        addEvent(event);
      }
      return;
    }      
  }
}

bool doEvent (EventT event) {
  //returns true if event is executed
  if (micros() < event.us) {
    return false;
  }
  switch(event.action) {
    case SET_COIL:
      setCoil(event.data);
      break;
    case SET_LED:
      setLED(event.data);
      break;
    case SET_READY:
      readyForCrossing = event.data;
      break;   
  }
  return true;
}


int readLineSerial(char buff[], int buffersize, int timeout = 2000) {
  int i = 0;
  if (!Serial.available()) {
    return 0;
  }
  elapsedMicros t0;
  while (i < buffersize-1 && (Serial.available() || t0 < timeout)) {
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


void setup() {
  // put your setup code here, to run once:

  pinMode(actCoilPin, OUTPUT);
  pinMode(bypassSensePin, OUTPUT);
  digitalWrite(bypassSensePin, HIGH);
  pinMode(detPin, INPUT);
  Wire.begin();


  

  Serial.begin(115200);
  
  //setup analog input, adapted from teensy example

   adc->adc0->setAveraging(32); // max appears to be 32
   adc->adc0->setResolution(16); // set bits of resolution

   adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
   adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED); // change the sampling speed
   adc->adc0->enableInterrupts(adc0_isr);
   adc->adc0->startContinuous(detPin);

  //setup mag sensor, adapted from adafruit example
  if (!lis2mdl.begin()) {  // I2C mode
    while (!Serial)
      delay(10); // adafruit setup code
    Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
  }


  while(!analogBuffer.isFull()) {
  }
  triggerThresh = meanAnalogValue(100);
  Serial.print("trigger threshold set to ");
  Serial.println(triggerThresh);
  
}

void loop() {
  detectorLoop();
  pulseLoop();
  magLoop();
  displayLoop();
}



void mag_isr(void) {
  newMagReading = true;
}

void instrumentLoop() {
  if (newADCReading) {
    newADCReading = false;
    analogBuffer.push(newADCReading);
  }
  if (newMagnetometerReading) {
    unsigned long us = micros();
    newMagnetometerReading = false;
    sensors_event_t event;
    lis2mdl.getEvent(&event); //needed to populate lis2mdl.raw
    magReadingT mr;
    mr.us = us;
    mr.x = lis2mdl.raw.x;
    mr.y = lis2mdl.raw.y;
    mr.z = lis2mdl.raw.z;
    
    
  }
}

void fitAnalogLine (index_t numel, float thresh, float &slope, float &xintercept) {
  //adapted from example in numerical recipes in C, 2nd edition (1992). 
  //fits a line of value vs. time; finds the time at which value = thresh, and finds the slope dval/dtime
  if (numel == 0) {
    numel = analogBuffer.size();    
  }
  index_t sz = analogBuffer.size();
  numel = min(numel, sz);
  unsigned long t0 = analogBuffer[analogBuffer.size() - numel].us;
  float sx = 0, sy = 0, sxx = 0, sxy = 0;

  //compute: time = a + b*(val - thresh). xintercept = a, slope = 1/b;
  for (index_t j = sz - numel; j < sz; ++j) {
    float y = (float) (analogBuffer[j].us - t0); //use unsigned math to take care of rollovers
    float x = analogBuffer[j].val - thresh;
    sx += x;
    sy += y;
    sxx += x*x;
    sxy += x*y;
  }
  float d = numel*sxx - sx*sx;
  xintercept = t0 + (sxx*sy - sx*sxy)/d;
  slope = d / (numel*sxy - sx*sy);
  
}

float meanAnalogValue (index_t numel) {
   if (numel == 0) {
    numel = analogBuffer.size();    
  }
  index_t sz = analogBuffer.size();
  numel = min(numel, sz);

  float s = 0;

  //compute: time = a + b*(val - thresh). xintercept = a, slope = 1/b;
  for (index_t j = sz - numel; j < sz; ++j) {
    s += analogBuffer[j].val;
  }
  return s/numel;
}

void detectorLoop() {
  index_t numToCrossing;
  if (!crossingBuffer.isEmpty() && micros() - crossingBuffer.last().us < mega*(retriggerDelay + pulseDelayFraction * halfPeriod)) { //too soon after last one
    return; 
  }
  if (!detectCrossing(numToCrossing)) {
    return;
  }
  if (numToCrossing < halfMinLineSamples) {
    return;
  }
  crossingT crossing;
  index_t sz = analogBuffer.size();
  numToCrossing = min(sz/2, numToCrossing);
  fitAnalogLine(2*numToCrossing, triggerThresh, crossing.slope, crossing.us);
  if (!crossingBuffer.isEmpty()) {
    halfPeriod = (crossing.us - crossingBuffer.last().us)*micro;
  }
  noInterrupts();
  crossingBuffer.push(crossing);
  interrupts();
  pulseArmed = true;
  magArmed = true;
}

void pulseLoop() {
  if (!pulseArmed || crossingBuffer.isEmpty()) {
    return;
  }
  if (micros() - crossingBuffer.last().us < pulseDelayFraction * halfPeriod * mega) {
    return;
  }
  pulseArmed = false;
  digitalWrite(actCoilPin, HIGH);
  //todo: rewrite using interrupts or timer pins
  if (pulseDuration < .016) {
    delayMicroseconds(pulseDuration*mega);
  } else {
    delay(pulseDuration*1000);
  }
  digitalWrite(actCoilPin, LOW);
}

void magLoop() {
  if (!magArmed || crossingBuffer.isEmpty()) {
    return;
  }
  if (micros() - crossingBuffer.last().us < 0.5 * halfPeriod * mega) { //read at 1/4 period and 3/4 period
    return;
  }
  magArmed = false;
  crossingT crossing;
  //todo, more sophisticated fit/averaging?
  magReadingT mag = magBuffer.last();
  noInterrupts();
  crossing = crossingBuffer.pop();
  crossing.xmag = mag.x;
  crossing.ymag = mag.y;
  crossingBuffer.push(crossing);
  interrupts();
  triggerThresh = meanAnalogValue(100); //this is when detector should be quietest, magnet is far away and not moving
}

void displayLoop() {
  if (crossingBuffer.size() < 2 || crossingBuffer.last().us == lastDisplayedCrossing) {
    return;
  }
  if (magArmed) {
    return;
  }
  
  crossingT c1 = crossingBuffer.last();
  crossingT c2 = crossingBuffer[crossingBuffer.size()-2];
  crossingT temp;
  if (c1.ymag < c2.ymag) { //keep angle between 0 and 180
    temp = c1;
    c1 = c2;
    c2 = temp;
  }
  float theta = atan2f(c1.ymag - c2.ymag, c1.xmag - c2.xmag);
  Serial.print("crossing at ");
  Serial.print(micro*crossingBuffer.last().us, 2);
  Serial.print(" s: period = ");
  Serial.print(halfPeriod*2);
  Serial.print(" s, slope = ");
  Serial.print(slopeMult * crossingBuffer.last().slope, 2);
  Serial.print(" V/s, angle = ");
  Serial.print(theta*radian, 2);
  Serial.println (" deg.");
}

bool detectCrossing(index_t &numToCrossing) {
   uint16_t val = analogBuffer.last().val;
   if (!((triggerFalling && val < triggerThresh - triggerHyst) || (triggerRising && val > triggerThresh - triggerHyst))) {
      return false;
   }
   bool falling = val < triggerThresh - triggerHyst; //protect against bidiretional triggering
   for (index_t j = analogBuffer.size() - 1; j >= 0; --j) {
      if ((falling && analogBuffer[j].val > triggerThresh) || (!falling && analogBuffer[j].val < triggerThresh)) {
        numToCrossing = analogBuffer.size()-j;
      }  
      if ((falling && analogBuffer[j].val > triggerThresh + triggerHyst) || (!falling && analogBuffer[j].val < triggerThresh - triggerHyst)) {
        return true;
      }    
   }
   return false;
}
