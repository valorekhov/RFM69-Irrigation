// **********************************************************************************************************
// Moteino 8mhz + Toro 53746 Zero-pressue drip irrigation valve 

#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/rfm69
#include <SPI.h>           //included in Arduino IDE (www.arduino.cc)
#include <Wire.h>          //included in Arduino IDE (www.arduino.cc)
#include "LowPower.h"      //get it here: https://github.com/lowpowerlab/lowpower
                           //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************

#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
//#define IS_RFM69HW_HCW  1//uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#include "Network_Config.h"

//*********************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -60
//*********************************************************************************************
#define SEND_LOOPS   75 //send data this many sleep loops (15 loops of 8sec cycles = 120sec ~ 2 minutes)
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_LONGEST; //period_t is an enum type defined in the LowPower library (LowPower.h)
//*********************************************************************************************

#define BATT_LOW      3.4  //(volts)
#define BATT_READ_LOOPS  SEND_LOOPS*450  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
//*****************************************************************************************************************************

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#define END_SWITCH_PIN  10
#define MOTOR_PIN       11
#define VBATPIN         13

#define BLINK_EN                 //uncomment to blink LED on every send
#define SERIAL_EN                //comment out if you don't want any serial output

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

void setup(void)
{
#ifdef SERIAL_EN
  while (!Serial) {Blink(LED, 500); Blink(LED, 1500); } // wait until serial console is open, remove if not tethered to computer. 
  Serial.begin(SERIAL_BAUD);
#endif
  pinMode(LED, OUTPUT);
  //pinMode(END_SWITCH_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  

  for(uint8_t i = 0; i < 5; i++){ 
    Blink(LED, 2000 + i*100);
  }
  DEBUGln("Started");

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
    
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  
  SERIALFLUSH();
  readBattery();
}

char input=0;
byte sendLoops=0;
byte battReadLoops=0;
float batteryVolts = 5;
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars

void loop()
{
  if (battReadLoops--<=0) //only read battery every BATT_READ_LOOPS cycles
  {
    readBattery();
    battReadLoops = BATT_READ_LOOPS-1;
  }
  
  if (sendLoops--<=0)   //send readings every SEND_LOOPS
  {
    sendLoops = SEND_LOOPS-1;

    //TODO: Read end switch


    byte sendbuf[6];

#ifdef SERIAL_EN    
    sprintf(buffer, "BAT=%s T=%d H=%d", BATstr, 0, 0);
    DEBUGln(buffer); 
#endif
  
    radio.sendWithRetry(GATEWAYID, sendbuf, 6, 1); //retry one time

    #ifdef BLINK_EN
      Blink(LED, 1500);
    #endif
  }

  if (radio.receiveDone())
  {
    if (radio.DATALEN >= 2 && (uint8_t)radio.DATA[0] == 0xFD){
      switch((uint8_t)radio.DATA[1]){
        case 0:
          valveOff((uint8_t)1);
          periodicOn = periodicOff = 0;
          break;
        case 1:
          valveOn((uint8_t)1);
          break;
        case 12:
          scheduleOn( radio.DATA[2] << 8 & radio.DATA[3], radio.DATA[4] << 8 & radio.DATA[5] );
          break;             
      }
    }
        
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
    }
    Blink(LED, 100);Blink(LED, 100);Blink(LED, 100);
  }
 
  SERIALFLUSH();
  radio.sleep(); //you can comment out this line if you want this node to listen for wireless programming requests

  LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF);
  DEBUGln("WAKEUP");
}

bool debounceRead(byte pin){
  byte cHigh, cLow;
  for(byte i = 0; i< 5; i++){
    if (digitalRead(pin) == HIGH){
     cHigh++;
    } else {
      cLow++;
    }
    delay(50);
  }
  return cHigh > cLow;
}

void monitorForEdgeTransition(bool rising){
  pinMode(END_SWITCH_PIN, INPUT);

  bool curReading = debounceRead(END_SWITCH_PIN);
  if (curReading == rising){  
    // We are interested in catching a transition when the end switch goes from being the oposite of expected state
    // to the expected state and stopping righ there.
    // We need to skip to the current and expected states being different;
      digitalWrite(END_SWITCH_PIN, HIGH);
      do{
        curReading = debounceRead(END_SWITCH_PIN);
        delay(50);
      } while(curReading != rising);
  }

  digitalWrite(END_SWITCH_PIN, HIGH);
  do{
    curReading = debounceRead(END_SWITCH_PIN);
    delay(50);
  } while(curReading == rising);
  
  digitalWrite(END_SWITCH_PIN, LOW);

  pinMode(END_SWITCH_PIN, OUTPUT);
  digitalWrite(END_SWITCH_PIN, LOW);
}

void valveOff(uint8_t eventId){
  monitorForEdgeTransition(false);
  valveState = 0;
  reportState(eventId);
}

void valveOn(uint8_t eventId){
  monitorForEdgeTransition(true);
  valveState = 1;
  reportState(eventId);
}

void scheduleOn(uint16_t onPeriod, uint16_t offPeriod){
  periodicOn = onPeriod;
  periodicOff = offPeriod;
}

void reportState(uint8_t eventId){
  uint8_t buf[5] = {0xFE, 00, 0xFF, 00, 00};
  buf[1] = eventId;
  if (valveState >= 0){
    buf[2] = (uint8_t)valveState;
  }
  uint16_t v = (uint16_t)(batteryVolts*100);
  buf[3] = v >> 8 & 0xFF;
  buf[4] = v & 0xFF;
  radio.sendWithRetry(GATEWAYID, buf, 5, 3);
  DEBUG("Valve state:"); DEBUG(valveState); DEBUGln();
  DEBUG("Battery volts:"); DEBUG(v); DEBUGln();
}

void readBattery()
{
  float readings=0;

  pinMode(VBATPIN, INPUT);
  for (byte i=0; i<5; i++) {
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    readings += measuredvbat;
  }
   
  batteryVolts = readings / 5.0;
  sprintf(BATstr, "%d", (uint16_t)(batteryVolts*100));
  if (batteryVolts <= BATT_LOW) BATstr = "LOW";

  pinMode(VBATPIN, OUTPUT);
  digitalWrite(VBATPIN,LOW);
}

void Blink(byte pin, unsigned int delayMs)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin,HIGH);
  delay(delayMs/2);
  digitalWrite(pin,LOW);
  delay(delayMs/2);  
}
