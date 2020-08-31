#include <CircularBuffer.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ADC.h>
#include <ADC_util.h>

#define mega 1000000.f
#define micro 0.000001f


Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
#define LIS2MDL_CLK 13
#define LIS2MDL_MISO 12
#define LIS2MDL_MOSI 11
#define LIS2MDL_CS 10
ADC *adc = new ADC(); // adc object

IntervalTimer magTimer;
IntervalTimer statusTimer;

const int detPin = A1;

const int actCoilPin = 5;
const int bypassSensePin = 4;
const int cat5171Addr = 44;

const int halfMinLineSamples = 10;


const float slopeMult = 50.35f; 
const float radian = 57.295779513082321f;

typedef struct {
  unsigned long us;
  uint16_t val;
} analogReadingT;

typedef struct {
  unsigned long us;
  uint16_t x;
  uint16_t y;
  uint16_t z;
} magReadingT;

typedef struct {
  float us;
  float slope;
  uint16_t xmag;
  uint16_t ymag;
} crossingT;

float halfPeriod = 1;
float triggerThresh = 0;
float triggerHyst = 200;
float retriggerDelay = 0.05; //AFTER pulse delivery

bool triggerFalling = true;
bool triggerRising = false;

bool pulseArmed = false;
bool magArmed = false;

float pulseDelayFraction = 0.25;
float pulseDuration = 0.005;

float lastDisplayedCrossing = 0;

bool getstatus = false;

//on teensy LC with 32 averages and everything at VERY_LOW_SPEED, sampling rate is 2 kHz; 200 samples = 100 ms; 20 samples = 10 ms
CircularBuffer<analogReadingT, 200> analogBuffer; //1200 bytes + overhead
using index_t = decltype(analogBuffer)::index_t;

//100 Hz, 10 = 100 ms
CircularBuffer<magReadingT, 10> magBuffer; //100 bytes + overhead

//~< 1Hz
CircularBuffer<crossingT, 10> crossingBuffer; //80 bytes + overhead

void setup() {
  // put your setup code here, to run once:

  pinMode(actCoilPin, OUTPUT);
  pinMode(bypassSensePin, OUTPUT);
  digitalWrite(bypassSensePin, HIGH);
  pinMode(detPin, INPUT);

   Wire.begin();

//  Wire.requestFrom(cat5171Addr, 1);
//  byte val = Wire.read();
//  Serial.print("CAT5171 value is ");
//  Serial.println(
//  
  Wire.beginTransmission(cat5171Addr);
  Wire.write(0);
  Wire.write(14);
  Wire.endTransmission();

  

  Serial.begin(9600);
  while(!Serial) {
    delay(10);
  }
  Serial.println("hello there");
  //setup analog input, adapted from teensy example

Serial.println("setting up mag sensor");
  lis2mdl.enableAutoRange(true);

  //setup mag sensor, adapted from adafruit example
  if (!lis2mdl.begin()) {  // I2C mode
    Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
        while (1) delay(10);

  }
    lis2mdl.printSensorDetails();
  
  Serial.println("magnetometer setup");

   adc->adc0->setAveraging(32); // max appears to be 32
   adc->adc0->setResolution(16); // set bits of resolution

   adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
   adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED); // change the sampling speed
   adc->adc0->enableInterrupts(adc0_isr);
   adc->adc0->startContinuous(detPin);

  
  
//  if (!magTimer.begin(mag_isr, 10000)) {
//    //magnetometer updates at 100 Hz, T = 10^4 us
//    Serial.println("mag timer failed to start");
//  }
//  if (!statusTimer.begin(status_isr, 1000000)) {
//    Serial.println("status timer failed to start");
//  }
  Serial.println("filling buffer");
    Serial.print("buffer has ");
    Serial.print(analogBuffer.available());
    Serial.println(" available");

      
//  while(!analogBuffer.isFull()) {
//
//    Serial.print("buffer has ");
//    Serial.print(analogBuffer.available());
//    Serial.println(" available");
//  }
  triggerThresh = meanAnalogValue(100);
  Serial.print("trigger threshold set to ");
  Serial.println(triggerThresh);
  
}

void loop() {
  Serial.println("loop");
 // detectorLoop();
 // pulseLoop();
  // magLoop();
  displayLoop();
}

void acd0_isr(void) {
  analogReadingT reading;
  reading.us = micros();
  reading.val = adc->adc0->analogReadContinuous();
  Serial.println("hi");
  noInterrupts();
  analogBuffer.push(reading);
  interrupts();
  Serial.println("bye");
}

void status_isr (void) {
  getstatus = true;
  Serial.println("foo bar");
}

void mag_isr(void) {
  magReadingT reading;
  sensors_event_t event;
  reading.us = micros();
  lis2mdl.getEvent(&event); //note that event is not used, but required to populate raw
  reading.x = lis2mdl.raw.x;
  reading.y = lis2mdl.raw.y;
  reading.z = lis2mdl.raw.z;
  //Serial.println("mag_isr");
  noInterrupts();
  magBuffer.push(reading);
  interrupts();
 //Serial.println("mag_isr_done");
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
  if (getstatus) {
    getstatus = false;
    Serial.print("analog buffer has ");
    Serial.print(analogBuffer.available());
    Serial.println(" available");
    Serial.print("mag buffer has ");
    Serial.print(magBuffer.available());
    Serial.println(" available");

  }
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
