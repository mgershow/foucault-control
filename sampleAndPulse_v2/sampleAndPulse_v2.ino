/* adapted from analogContinuousRead.ino 
 *  https://github.com/pedvide/ADC/tree/master/examples/analogContinuousRead
 *  
 *  Example for analogContinuousRead
*  It measures continuously the voltage on pin A9,
*  Write v and press enter on the serial console to get the value
*  Write c and press enter on the serial console to check that the conversion is taking place,
*  Write t to check if the voltage agrees with the comparison in the setup()
*  Write s to stop the conversion, you can restart it writing r.
*/
 #include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <ADC.h>
#include <ADC_util.h>
#include <Wire.h>

Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
#define LIS2MDL_CLK 13
#define LIS2MDL_MISO 12
#define LIS2MDL_MOSI 11
#define LIS2MDL_CS 10

const int readPin = A1; // DET_OUT
const int threshPin = A0; // 1.25V = VRef

const int coilOff = 23;
const int gainCtrlAddr = 44;
const int currCtrlAddr = 45;
//const int readPin = A9; // ADC0
//const int readPin2 = A3; // ADC1
//const int readPin3 = A2; // ADC0 or ADC1

const int gainSetting = 2; //levels
const float currentSetting = 1; //Amp

float thresh = 12800;
uint16_t hysteresis = (uint16_t) (.025/3.3 * 65536);
uint16_t lastValue = 0;
bool state;
bool transition;
bool firing;
int timeToActivate = 350;
int durToActivate = 40;
int retriggerDelay = 50;
bool dirToActivate = false;

volatile bool newadc = false;

unsigned long lastLow;
unsigned long lastHigh;
unsigned long lastRead;
unsigned long deltaRead;
uint16_t lowVal;
uint16_t highVal;

unsigned long maxTransitionInterval = 10000; //microseconds

float lastCrossing = 0;
sensors_event_t lastevent;
ADC *adc = new ADC(); // adc object

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(readPin, INPUT);
    pinMode(threshPin, INPUT);

    
    pinMode(coilOff, OUTPUT);
    digitalWrite(coilOff, HIGH);
    

    Serial.begin(9600);

    ///// ADC0 ////
    // reference can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 (not for Teensy LC) or ADC_REFERENCE::REF_EXT.
    //adc->adc0->setReference(ADC_REFERENCE::REF_1V2); // change all 3.3 to 1.2 if you change the reference to 1V2

    adc->adc0->setAveraging(32); // set number of averages
    adc->adc0->setResolution(16); // set bits of resolution

    // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
    // see the documentation for more information
    // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
    // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED); // change the conversion speed
    // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED); // change the sampling speed

    // always call the compare functions after changing the resolution!
    //adc->adc0->enableCompare(1.0/3.3*adc->getMaxValue(), 0); // measurement will be ready if value < 1.0V
    //adc->adc0->enableCompareRange(1.0*adc->adc0->getMaxValue()/3.3, 2.0*adc->adc0->getMaxValue()/3.3, 0, 1); // ready if value lies out of [1.0,2.0] V

    // If you enable interrupts, notice that the isr will read the result, so that isComplete() will return false (most of the time)
    adc->adc0->enableInterrupts(adc0_isr);

    adc->adc0->startContinuous(readPin);
    //adc->startContinuousDifferential(A10, A11, ADC_0);

    while(!Serial) {
      delay(10);
    }
    thresh =  adc->adc0->analogRead(threshPin);
    
    Serial.print("thresh = ");
    Serial.println(thresh);

//    lis2mdl.enableAutoRange(true);

  /* Initialise the sensor */
 // if (!lis2mdl.begin()) {  // I2C mode
  //if (! lis2mdl.begin_SPI(LIS2MDL_CS)) {  // hardware SPI mode
  //if (! lis2mdl.begin_SPI(LIS2MDL_CS, LIS2MDL_CLK, LIS2MDL_MISO, LIS2MDL_MOSI)) { // soft SPI
    /* There was a problem detecting the LIS2MDL ... check your connections */
  //  Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
  //  while (1) delay(10);
 // }

  /* Display some basic information on this sensor */
 // lis2mdl.printSensorDetails();

    Wire.begin();
    Wire.beginTransmission(gainCtrlAddr);
    Wire.write(byte(0x00));
    Wire.write(gainSetting);
    Wire.endTransmission();
    Serial.print("gain set to "); Serial.println(catDtoGain(gainSetting), DEC);

    uint8_t cbyte = (uint8_t) (currentSetting / .0083 + 0.5);
    Wire.beginTransmission(currCtrlAddr);
    Wire.write(byte(0x00));
    Wire.write(cbyte);
    Wire.endTransmission();

    Serial.print("current set to "); Serial.print(cbyte * 0.0083); Serial.println (" A.");
    
    delay(500);
}

int value = 0;
int value2 = 0;
char c=0;

void loop() {

    if (Serial.available()) {
        c = Serial.read();
        if(c=='c') { // conversion active?
            Serial.print("Converting? ADC0: ");
            Serial.println(adc->adc0->isConverting());
            #ifdef ADC_DUAL_ADCS
            Serial.print("Converting? ADC1: ");
            Serial.println(adc->adc1->isConverting());
            #endif
        } else if(c=='s') { // stop conversion
            adc->adc0->stopContinuous();
            Serial.println("Stopped");
        } else if(c=='t') { // conversion successful?
            Serial.print("Conversion successful? ADC0: ");
            Serial.println(adc->adc0->isComplete());
            #ifdef ADC_DUAL_ADCS
            Serial.print("Conversion successful? ADC1: ");
            Serial.println(adc->adc1->isComplete());
            #endif
            Serial.print("Delta read = ");
            Serial.println(deltaRead, DEC);
        } else if(c=='r') { // restart conversion
            Serial.println("Restarting conversions ");
            adc->adc0->startContinuous(readPin);
            //adc->startContinuousDifferential(A10, A11, ADC_0);
        } else if(c=='v') { // value
            Serial.print("Value ADC0: ");
            value = (uint16_t)adc->adc0->analogReadContinuous(); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
            Serial.println(lastValue, DEC);//*3.3/adc->adc0->getMaxValue(), DEC);
            Serial.print ("Thresh = ");
            Serial.println(thresh + (state ? -1*hysteresis : hysteresis));

            #ifdef ADC_DUAL_ADCS
                        Serial.print("Value ADC1: ");
            value2 = (uint16_t)adc->adc1->analogReadContinuous(); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
            Serial.println(value2*3.3/adc->adc1->getMaxValue(), DEC);
            #endif
        } else if(c=='n') { // new single conversion on readPin3
            // this shows how even when both ADCs are busy with continuous measurements
            // you can still call analogRead, it will pause the conversion, get the value and resume the continuous conversion automatically.
            Serial.print("Single read on readPin3: ");
            Serial.println(adc->adc0->analogRead(readPin)*3.3/adc->adc0->getMaxValue(), DEC);
        }
    }

    // Print errors, if any.
    if(adc->adc0->fail_flag != ADC_ERROR::CLEAR) {
      Serial.print("ADC0: "); Serial.println(getStringADCError(adc->adc0->fail_flag));
    }
    #ifdef ADC_DUAL_ADCS
    if(adc->adc1->fail_flag != ADC_ERROR::CLEAR) {
      Serial.print("ADC1: "); Serial.println(getStringADCError(adc->adc1->fail_flag));
    }
    #endif
    bool falling = true;
    bool rising = false;
    if (((micros() - lastHigh < maxTransitionInterval) && falling && (lastLow - lastHigh) < maxTransitionInterval) || (micros() - lastLow < maxTransitionInterval) && rising && (lastHigh - lastLow) < maxTransitionInterval) {
        transition = true;
      }
   

    if (transition) {
      double deltaT = lastLow > lastHigh ? lastLow-lastHigh : lastHigh - lastLow;
      float crosstime = (lastLow + lastHigh)/2;
      float period = (crosstime - lastCrossing)*0.000001;
      Serial.print ("period = ");
      Serial.println(period,2);
      lastCrossing = crosstime;
      double deltaV = highVal - lowVal;
      Serial.print("slope = ");
      Serial.println(deltaV/deltaT, DEC);
      firing = true;
      digitalWriteFast(LED_BUILTIN,true);
      delay(timeToActivate);
      digitalWrite(coilOff, LOW);
      delay(durToActivate);
      digitalWrite(coilOff, HIGH);
      digitalWriteFast(LED_BUILTIN,false);
      transition = false;
      delay(retriggerDelay);
      firing = false;
      if (period > 0 && period < 3) {
       // Serial.print(period*500 - retriggerDelay - timeToActivate - durToActivate, DEC);
        if (period*500 > retriggerDelay + timeToActivate + durToActivate) {
          delay(period*500 - retriggerDelay - timeToActivate - durToActivate);
        }
        sensors_event_t event;
     // lis2mdl.getEvent(&event);
//      double deltaX = event.magnetic.x - lastevent.magnetic.x;
  //    double deltaY = event.magnetic.y - lastevent.magnetic.y;
//      if (deltaY < 0) {
//        deltaY = -deltaY;
//        deltaX = -deltaX;
//      }
 //     lastevent = event;
 //     Serial.print("theta = ");
 //     Serial.println(atan2(deltaY, deltaX)*180/3.14159,2);
      }
      
    } 
    //delay(100);
}

float catDtoGain(float D) {
   return (1 + 49400.0/2700.0 + 49400.0/(D/256*50000 + 50));
}

void adc0_isr(void) {
//    if (transition)
//      return;
    unsigned long temp = micros();
    deltaRead = temp - lastRead;
    lastRead = temp;
    lastValue = (uint16_t) adc->adc0->analogReadContinuous();
    newadc = true;
    if (!firing) {
      if (lastValue < thresh - hysteresis) {
        lastLow = micros();
        lowVal = lastValue;
      } 
      if (lastValue > thresh + hysteresis) {
        lastHigh = micros();
        highVal = lastValue;
      }
    } 
    // Toggle the led
}
