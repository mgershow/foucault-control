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

#include <ADC.h>
#include <ADC_util.h>
#include <Wire.h>

const int readPin = A1; // ADC0

const int actCoil = 5;
const int bypassSense = 4;
const int cat5171Addr = 44;
//const int readPin = A9; // ADC0
//const int readPin2 = A3; // ADC1
//const int readPin3 = A2; // ADC0 or ADC1

uint16_t thresh = 35800;
uint16_t hysteresis = 200;
uint16_t lastValue;
bool state;
bool transition;
bool firing;
int timeToActivate = 50;
int durToActivate = 100;
int retriggerDelay = 500;
bool dirToActivate = false;

unsigned long lastLow;
unsigned long lastHigh;

unsigned long maxTransitionInterval = 10000; //microseconds

ADC *adc = new ADC(); // adc object

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(readPin, INPUT);
    pinMode(actCoil, OUTPUT);
    digitalWrite(actCoil, LOW);
    pinMode(bypassSense, OUTPUT);
    digitalWrite(bypassSense, HIGH);

    Serial.begin(9600);

    ///// ADC0 ////
    // reference can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 (not for Teensy LC) or ADC_REFERENCE::REF_EXT.
    //adc->adc0->setReference(ADC_REFERENCE::REF_1V2); // change all 3.3 to 1.2 if you change the reference to 1V2

    adc->adc0->setAveraging(16); // set number of averages
    adc->adc0->setResolution(16); // set bits of resolution

    // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
    // see the documentation for more information
    // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
    // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED); // change the conversion speed
    // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed

    // always call the compare functions after changing the resolution!
    //adc->adc0->enableCompare(1.0/3.3*adc->getMaxValue(), 0); // measurement will be ready if value < 1.0V
    //adc->adc0->enableCompareRange(1.0*adc->adc0->getMaxValue()/3.3, 2.0*adc->adc0->getMaxValue()/3.3, 0, 1); // ready if value lies out of [1.0,2.0] V

    // If you enable interrupts, notice that the isr will read the result, so that isComplete() will return false (most of the time)
    adc->adc0->enableInterrupts(adc0_isr);

    adc->adc0->startContinuous(readPin);
    //adc->startContinuousDifferential(A10, A11, ADC_0);

   

    pinMode(actCoil, OUTPUT);
    digitalWrite(actCoil, LOW);
    pinMode(bypassSense, OUTPUT);
    digitalWrite(bypassSense, HIGH);

    Wire.begin();
    Wire.beginTransmission(cat5171Addr);
    Wire.write(byte(0x00));
    Wire.write(14);
    Wire.endTransmission();

    delay(500);
}

int value = 0;
int value2 = 0;
char c=0;
bool on = false;
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
        } else if (c == 'o') {
          on = !on;
          digitalWriteFast(LED_BUILTIN,on);           
          digitalWrite(actCoil, on);
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

    if (((micros() - lastHigh < maxTransitionInterval) && (lastLow - lastHigh) < maxTransitionInterval) || (micros() - lastLow < maxTransitionInterval) && (lastHigh - lastLow) < maxTransitionInterval) {
        transition = true;
      }
   

    if (false && transition) {
      unsigned long delta = lastLow > lastHigh ? lastLow-lastHigh : lastHigh - lastLow;
      Serial.print("Delta = ");
      Serial.println(delta, DEC);
      firing = true;
      digitalWriteFast(LED_BUILTIN,true);
      delay(timeToActivate);
      digitalWrite(actCoil, true);
      delay(durToActivate);
      digitalWrite(actCoil, false);
      digitalWriteFast(LED_BUILTIN,false);
      transition = false;
      delay(retriggerDelay);
      firing = false;
    } 
    //delay(100);
}

void adc0_isr(void) {
//    if (transition)
//      return;
    lastValue = (uint16_t) adc->adc0->analogReadContinuous();
    if (!firing) {
      if (lastValue < thresh - hysteresis) {
        lastLow = micros();
      } 
      if (lastValue > thresh + hysteresis) {
        lastHigh = micros();
      }
    } 
    // Toggle the led
}
