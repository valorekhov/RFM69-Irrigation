// **********************************************************************************************************
// Moteino 8mhz + Toro 53746 Zero-pressue drip irrigation valve 

#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/rfm69
#include <SPI.h>           //included in Arduino IDE (www.arduino.cc)
#include <Wire.h>          //included in Arduino IDE (www.arduino.cc)
#include "LowPower.h"      //get it here: https://github.com/lowpowerlab/lowpower
                           //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <Time.h>

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************

#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
//#define IS_RFM69HW_HCW  1//uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#include "Network_Config.h"

//*********************************************************************************************
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -60
//*********************************************************************************************
#define SEND_LOOPS   450 //send data this many sleep loops (15 loops of 8sec cycles = 120sec ~ 2 minutes)
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_LONGEST; //period_t is an enum type defined in the LowPower library (LowPower.h)
//*********************************************************************************************

#define BATT_LOW      240  //(volts)
#define BATT_READ_LOOPS  SEND_LOOPS*450  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
//*****************************************************************************************************************************

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#define END_SWITCH_PIN  3
#define END_SWITCH_SINK 2
#define MOTOR_PIN       4
#define LED2_PIN        5
#define SW_COM_PIN      7
#define SW_COM2_PIN     A0
#define SW1_PIN         6
//#define SW2_PIN         A1
#define SW3_PIN         A1
#define SW4_PIN         A2

#define BLINK_EN                 //uncomment to blink LED on every send
//#define SERIAL_EN                //comment out if you don't want any serial output

#ifdef SERIAL_EN
  #define SERIAL_BAUD   115200
  #define DEBUG(input)   {Serial.print(input);}
  #define DEBUGln(input) {Serial.println(input);}
  #define SERIALFLUSH() {Serial.flush();}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define SERIALFLUSH();
#endif
//*****************************************************************************************************************************

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

char buffer[100];
int8_t valveState = -1;
uint16_t periodicOn = 0;
uint16_t periodicOff = 0;
uint16_t zoneTimer = 0;
uint8_t switchState = 0;

uint16_t programs[3][2] = {{1,1440}, {3,1440}, {3,60}};

void setup(void)
{
#ifdef SERIAL_EN
  while (!Serial) {Blink(LED, 500); Blink(LED, 1500); } // wait until serial console is open, remove if not tethered to computer. 
  Serial.begin(SERIAL_BAUD);
#endif
  pinMode(LED, OUTPUT);
  //pinMode(END_SWITCH_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

#ifdef SERIAL_EN
  sprintf(buffer, "Valve - transmitting at: %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buffer);
#endif

  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!
  
  for (uint8_t i=0; i<=A5; i++)
  {
    if (i == RF69_SPI_CS) continue;
    if (i == LED2_PIN) continue;
        
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  
  SERIALFLUSH();
  readBattery();
  valveOff(0);
}

byte sendLoops=0;
byte battReadLoops=0;
unsigned int batteryVolts = 500;
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars

void loop()
{
  bool skipReport = false;
  bool skipSleep = false;
  //DEBUGln(second());
  if (second() <= 8){ // trigger every minute

    if (zoneTimer > 0) zoneTimer--;

    DEBUG("Zone timer: ");
    DEBUGln(zoneTimer);
    DEBUG("State: ");
    DEBUGln(valveState);
    
    if (zoneTimer == 0){
      if (valveState == 0 && periodicOn > 0){
          DEBUG("Entering ON zone (min) "); DEBUGln(periodicOn);
          zoneTimer = periodicOn;  
          valveOn((uint8_t)0x20);
          skipReport = true;
          blinkLED2(3000, 1);
      } else if (valveState == 1 && periodicOff > 0) {
          DEBUG("Entering OFF zone (min) "); DEBUGln(periodicOff);
          zoneTimer = periodicOff;
          valveOff((uint8_t)0x20);
          skipReport = true;
          blinkLED2(1500, 2);
      }
    }
  }

  uint8_t newSw = readSwitches();
  //DEBUG("Switch state "); DEBUGln(newSw);
  if (newSw & 0b111 == 0){
    newSw = 0;  // Ignore Program Selection Switches in Manual OFF mode;
  } /*else if (newSw & 1 == 1){
    newSw = 1;  // Ignore Program Selection Switches in Manual ON mode;
  } else if (newSw & 2 == 2){
    newSw = 2;  // Ignore Program Selection Switches in Remote Control mode;
  }*/
  
  if (newSw != switchState){
    DEBUG("New switch state "); DEBUGln(newSw);
    if (newSw == 0) {
        blinkLED2(1000, 1);
        valveOff((uint8_t)2);
        periodicOn = periodicOff = 0;
        zoneTimer = 0;
        skipSleep = false;
    } else if (newSw == 1){ // DURATION=ON and FREQ=OFF
        pinMode(LED2_PIN, OUTPUT);
        digitalWrite(LED2_PIN,LOW);        
        valveOn((uint8_t)2);
        periodicOn = periodicOff = 0;
        zoneTimer = 0;
        skipSleep = false;
    } else if (newSw & 1 == 1 && newSw & 0b111000 > 0){ // DURATION=ON and FREQ=WEEK, 72 or 48
        uint8_t progIdx = newSw >> 3 & 0b111;
        blinkLED2(1000, 3);
        if (valveState == 1){
            valveOff((uint8_t)2);
        }

        switch(progIdx){
          case 2:
            progIdx = 1;
            break;
          case 4:
            progIdx = 2;
            break;      
          default:
            progIdx = 0;
            break;    
        }
        
        DEBUG("Stored program mode: "); DEBUGln(progIdx);
        periodicOn = programs[progIdx][0];
        periodicOff = programs[progIdx][1];
        DEBUG("Setting PerON : "); DEBUGln(periodicOn);  
        DEBUG("Setting PerOFF: "); DEBUGln(periodicOff);     
        zoneTimer = 0;
        skipSleep = false;
        blinkLED2(2000, progIdx+1);
    } else if (newSw == 2){ //DURATION=120
        if (valveState == 1){
          valveOff((uint8_t)2);
        }
        blinkLED2(1000, 3);
        skipSleep = true;
    }
    switchState = newSw;
  }
  
  if (battReadLoops--<=0) //only read battery every BATT_READ_LOOPS cycles
  {
    readBattery();
    battReadLoops = BATT_READ_LOOPS-1;
  }
  
  if (sendLoops--<=0)   //send readings every SEND_LOOPS
  {
    sendLoops = SEND_LOOPS-1;

    if (!skipReport && switchState > 0){
      reportState(0);
    }

    byte sendbuf[6];

#ifdef SERIAL_EN    
    sprintf(buffer, "BAT=%s S=%d", BATstr, valveState);
    DEBUGln(buffer); 
#endif

    #ifdef BLINK_EN
      Blink(LED, 1500);
    #endif
  }

  if (radio.receiveDone())
  {
    if (switchState == 2 && radio.DATALEN >= 2 && (uint8_t)radio.DATA[0] == 0xFD){
      switch((uint8_t)radio.DATA[1]){
        case 0:
          valveOff((uint8_t)1);
          periodicOn = periodicOff = 0;
          zoneTimer = 0;
          break;
        case 1:
          valveOn((uint8_t)1);
          break;
        case 12:
          scheduleOn( ((uint16_t)radio.DATA[2]) << 8 | radio.DATA[3], ((uint16_t)radio.DATA[4]) << 8 | radio.DATA[5] );
          break;             
      }
    }
        
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
    }

    if (switchState !=2 ) {
        DEBUGln("Valve not in Remote Control mode. Sending back and Error");
        uint8_t buf[4] = {0xEE, 00, 00, 00};
        buf[2] = switchState;
        if (valveState >= 0){
          buf[3] = (uint8_t)valveState;
        }
        radio.sendWithRetry(GATEWAYID, buf, 4, 3);
    }
    
    Blink(LED, 100);Blink(LED, 100);Blink(LED, 100);
  }
 
  SERIALFLUSH();

  if (switchState != 2)
    radio.sleep(); //you can comment out this line if you want this node to listen for wireless programming requests

  if (skipSleep){
    delay(1000);
    DEBUGln("AWAKE");
  } else {
    //TODO: Re-enable
    //delay(8000);
    LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF);
    adjustTime(8);
    DEBUGln("WAKEUP");
  }
}

bool debounceRead(uint8_t pin){
  uint8_t cHigh = 0;
  uint8_t cLow = 0;
  for(uint8_t i = 0; i<7; i++){
    if (digitalRead(pin) == HIGH){
     cHigh++;
    } else {
      cLow++;
    }
    delay(20);
  }
  return cHigh > cLow;
}

void monitorForEdgeTransition(bool expectedState){
    pinMode(END_SWITCH_PIN, INPUT);
    pinMode(END_SWITCH_SINK, OUTPUT);
    digitalWrite(END_SWITCH_SINK, LOW);
    
    bool curReading = debounceRead(END_SWITCH_PIN);
    
    DEBUG("Valve open start state:"); DEBUGln(curReading);
    
    if (curReading == expectedState){  
      // We are interested in catching a transition when the end switch goes from being the oposite of expected state
      // to the expected state and stopping righ there.
      // We need to skip to the current and expected states being different;
        DEBUGln("Start motor state1");
        digitalWrite(MOTOR_PIN, HIGH);
        bool state1Reading;
        do{
          state1Reading = debounceRead(END_SWITCH_PIN);
          //DEBUG("Valve open state1:"); DEBUGln(state1Reading);
        } while(curReading == state1Reading);
    }
    
    DEBUGln("Start motor state2");
    digitalWrite(MOTOR_PIN, HIGH);
    do{
      curReading = debounceRead(END_SWITCH_PIN);
      //DEBUG("Valve open state2:"); DEBUGln(curReading);
    } while(curReading != expectedState);

    
    //digitalWrite(MOTOR_PIN, HIGH);
    do{
      curReading = debounceRead(END_SWITCH_PIN);
      //DEBUG("Valve open state3:"); DEBUGln(curReading);
    } while(expectedState == curReading);
      
    digitalWrite(MOTOR_PIN, LOW);
    DEBUGln("Stop motor");
  
    
    pinMode(END_SWITCH_PIN, OUTPUT);
    digitalWrite(END_SWITCH_PIN, LOW);
    pinMode(END_SWITCH_SINK, INPUT);
}

uint8_t readSwitches(){

  uint8_t ret = 0;
  pinMode(SW_COM_PIN, OUTPUT);
  digitalWrite(SW_COM_PIN, LOW);
  
  pinMode(SW1_PIN, INPUT);
  pinMode(SW3_PIN, INPUT);
  pinMode(SW4_PIN, INPUT);

  //DEBUG("SW1 is "); DEBUGln(!debounceRead(SW1_PIN));
  if (!debounceRead(SW1_PIN)) 
    ret = ret | 0b1;
  //DEBUG("SW3 is "); DEBUGln(!debounceRead(SW3_PIN));
  if (!debounceRead(SW3_PIN)) 
    ret = ret | 0b10;
  //DEBUG("SW4 is "); DEBUGln(!debounceRead(SW4_PIN));
  if (!debounceRead(SW4_PIN)) 
    ret = ret | 0b100;  
  pinMode(SW_COM_PIN, INPUT);


  pinMode(SW_COM2_PIN, OUTPUT);
  digitalWrite(SW_COM2_PIN, LOW);
  //DEBUG("=SW1 is "); DEBUGln(!debounceRead(SW1_PIN));
  if (!debounceRead(SW1_PIN)) 
    ret = ret | 0b1000;  
  //DEBUG("=SW3 is "); DEBUGln(!debounceRead(SW3_PIN));
  if (!debounceRead(SW3_PIN)) 
    ret = ret | 0b10000;    
  //DEBUG("=SW4 is "); DEBUGln(!debounceRead(SW4_PIN));
  if (!debounceRead(SW4_PIN)) 
    ret = ret | 0b100000;    
  pinMode(SW_COM2_PIN, INPUT);

  return ret;
}

void valveOff(uint8_t eventId){
  DEBUGln("Setting valve OFF");
  monitorForEdgeTransition(false);
  valveState = 0;
  reportState(eventId);
}

void valveOn(uint8_t eventId){
  DEBUGln("Setting valve ON");
  monitorForEdgeTransition(true);
  valveState = 1;
  reportState(eventId);
}

void scheduleOn(uint16_t onPeriod, uint16_t offPeriod){
  DEBUGln("Setting autoschedule");
  DEBUG("ON (min):"); DEBUGln(onPeriod);
  DEBUG("OFF (min):"); DEBUGln(offPeriod);
  periodicOn = onPeriod;
  periodicOff = offPeriod;
  zoneTimer = 0;
}

void reportState(uint8_t eventId){
  uint8_t buf[5] = {0xFE, 00, 0xFF, 00, 00};
  buf[1] = eventId;
  if (valveState >= 0){
    buf[2] = (uint8_t)valveState;
  }
  uint16_t v = batteryVolts;
  buf[3] = v >> 8 & 0xFF;
  buf[4] = v & 0xFF;
  radio.sendWithRetry(GATEWAYID, buf, 5, 3);
  DEBUG("Valve state:"); DEBUG(valveState); DEBUGln();
  DEBUG("Switch state:"); DEBUG(switchState); DEBUGln();
  DEBUG("Battery volts:"); DEBUG(v); DEBUGln();
}

int getBandgap(void) // Returns actual value of Vcc (x 100)
 {
     
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // For mega boards
  const long InternalReferenceVoltage = 1115L;  // Adjust this value to your boards specific internal BG voltage x1000
     // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc reference
     // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)         -Selects channel 30, bandgap voltage, to measure
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR)| (0<<MUX5) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);

#else
  // For 168/328 boards
  const long InternalReferenceVoltage = 1056L;  // Adjust this value to your boards specific internal BG voltage x1000
     // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
     // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
    
#endif
  delay(50);  // Let mux settle a little to get a more stable A/D conversion
     // Start a conversion  
  ADCSRA |= _BV( ADSC );
     // Wait for it to complete
  while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
     // Scale the value
  int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // calculates for straight line value 
  return results;

 }

void readBattery()
{
  batteryVolts = 0;
  for(uint8_t i=0; i<5; i++){
    batteryVolts += getBandgap();
  }
  batteryVolts = batteryVolts / 5;
  
  sprintf(BATstr, "%d", batteryVolts);
  if (batteryVolts <= BATT_LOW) BATstr = "LOW";
}

void blinkLED2(unsigned int delayMs, uint8_t times)
{
  for(uint8_t i=0; i<times; i++){
    pinMode(LED2_PIN, OUTPUT);
    digitalWrite(LED2_PIN,LOW);
    delay(delayMs/2);
    digitalWrite(LED2_PIN,HIGH);
    delay(delayMs/2);  
  }
}

void Blink(byte pin, unsigned int delayMs)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin,HIGH);
  delay(delayMs/2);
  digitalWrite(pin,LOW);
  delay(delayMs/2);  
}
