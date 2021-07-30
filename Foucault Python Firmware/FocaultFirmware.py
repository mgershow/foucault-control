from enum import Enum
from dataclasses import dataclass

NUM_CMD_DATA = 4
NUM_ACT_DATA = 4
CHAR_BUF_SIZE = 128

class ActionT(Enum):
    SET_COIL = 0
    SET_LED = 1
    SET_READY = 2

class TransmitTypeT(Enum):
    NONE = 0
    DETVAL = 1
    MAGX = 2
    MAGY = 3
    MAGZ = 4
    ACCX = 5
    ACCY = 6
    ACCZ = 7
    COIL = 8

@dataclass
class Reading1T:
    us : float
    val : float

@dataclass
class Reading3T:
    us : float
    x : float
    y : float
    z : float

@dataclass
class CommandT:
    cmd : str
    us : float
    data : List[float] = [0.0] * NUM_CMD_DATA

@dataclass
class crossingT:
    us : float
    slope : float

@dataclass
class EventT:
    us : float
    data : List[float] = [0.0] * NUM_ACT_DATA

class FoucaultControlBoard:

   


Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(1);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
ADC *adc = new ADC(); // adc object

IntervalTimer agrTimer;


bool enableDataTransmission = true;
bool enableADCTransmission = true;
bool enableMagTransmission = true;
bool enableAccTransmission = true;
bool enableCoilTransmission = true;

bool autoFlash = true;


const int indicatorPins[] = {0,0,0,0,0,0,0,0}; //pin 0 is not conected - there are no indicators on this board

typedef enum {COIL_ON, BAD_MSG, READY_FOR_CROSSING, AUTO_ON, LED_ON, TRANSMIT_SERIAL } LedMessageTypeT;




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
/********** REVISED FOR 2021 BOARD *****************************/

const int actCoilPin = 7;
const int actLEDPin = 8;
const int gainSet0 = 2;
const int gainSet1 = 3;


const int detectorPin = A0;
const int refPin = A1;


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

int verbosity = 0;

unsigned int numADCToAvg = 16;

const byte EEPROM_ACC_CODE = 123; //if byte at eeprom_acc_address is 123, then magnetometer data has been written
const int EEPROM_ACC_ADDRESS = 900; //arbitrary choice
struct {
  float x = 0;
  float y = 0;
} accZero;
uint8_t numEepromWrites = 0; //prevent writing eeprom more than 255 times per power cycle - avoid accidental fatigue

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
volatile double lasttime = 0;
const double microrollover = 4294967295;

byte currentGain;
/********************* hardware control **************************/

void setLedMessage (LedMessageTypeT msg, bool setting) {
  digitalWrite(indicatorPins[msg], setting);
}

void setCoil(bool activate, float duration = -1) {
  coilState = activate;
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
  digitalWrite(gainSet0, bitRead(g, 0));
  digitalWrite(gainSet1, bitRead(g, 1));
  currentGain = g;
}
byte readGain() {
  return currentGain;
}

double getTime() {
  double thistime;
  bool rollover;
  noInterrupts();
  thistime = micros() + baseTime;
  if (rollover = (thistime < lasttime)) { //intentional use of assignment = 
    baseTime += microrollover;
    thistime += microrollover;
  }
  lasttime = thistime;
  interrupts();
  if (rollover) {
    sendMessage("micros rollover", 1);
  }
  return thistime;
}

/*************** setup *************************************/

void setupPins() {

  //enable pullups for i2c - these should be present on board, but are missing in prototype design
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP); 

  pinMode(detectorPin, INPUT);
  pinMode(refPin, INPUT);
  for (int j = 0; j < 8; ++j) {
    pinMode(indicatorPins[j], OUTPUT);
  }
  pinMode(gainSet0, OUTPUT);
  pinMode(gainSet1, OUTPUT);
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
    hasAcc = accel.begin(0x19, &Wire1);
    accel.setRange(LSM303_RANGE_2G);
    accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
  }
 // Serial.println("b");
  if (!hasMag) {
    lis2mdl.enableAutoRange(true);
    hasMag = lis2mdl.begin(0x1E,&Wire1);  
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
  
  Wire1.begin();
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
  if (numEepromWrites >= 255) {
    return;
  }
  numEepromWrites++;
  byte code = EEPROM_ACC_CODE;
  EEPROM.put(EEPROM_ACC_ADDRESS,code);
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

  if (autoFlash) {
    event.us = us + 1; // 1 microsecond later
    event.data[0] = 255;
    event.action = SET_LED;
    addEvent(event);
    event.us = us + pulseDuration*mega + 1;
    event.data[0] = 0;
    addEvent(event);
  }

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