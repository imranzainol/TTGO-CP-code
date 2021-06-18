/*
  Tutorial: Log Sensor's Data to Blynk Server Using GPRS on TTGO ESP32 SIM800L
  Board:
  - TTGO T-Call ESP32 with SIM800L GPRS Module
    https://my.cytron.io/p-ttgo-t-call-esp32-with-sim800l-gprs-module

  Sensor:
  - MPL3115A2 I2C Barometric/Altitude/Temperature Sensor
    https://my.cytron.io/p-mpl3115a2-i2c-barometric-altitude-temperature-sensor
    Connection    ESP32 | MPL3115A2
                     5V - Vin
                    GND - GND
                     22 - SCL
                     21 - SDA

  External libraries:
  - Adafruit MPL3115A2 Library by Adafruit Version 1.2.2
  - Blynk by Volodymyr Shymanskyy Version 0.6.1
  - TinyGSM by Volodymyr Shymanskyy Version 0.7.9

  Created by:
  20 Jan 2020   Idris Zainal Abidin, Cytron Technologies
*/

/* Comment this out to disable prints and save space */
#define RXD2 3
#define TXD2 1

byte ByteArray[250];
int ByteData[20];


#define BLYNK_PRINT Serial


#define TINY_GSM_MODEM_SIM800

// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:.
//#define BLYNK_HEARTBEAT 30

#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>

#define SIM800L_RX     27
#define SIM800L_TX     26
#define SIM800L_PWRKEY 4
#define SIM800L_RST    5
#define SIM800L_POWER  23

#define BUTTON1 15
#define BUTTON2 0
#define LED_BLUE  13
#define LED_GREEN 14
#define PIEZO   25

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "JVNsgtg-FbCQEFuhYbtWX1lULmgzOzca";

// Your GPRS credentials
// Leave empty, if missing user or pass
char apn[]  = "XOX";
char user[] = "";
char pass[] = "";

// Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1
TinyGsm modem(SerialAT);

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

float pascals = 0.0;
float altitude = 0.0;
float celsius = 0.0;
long prevMillisSensor = 0;
int intervalSensor = 2000;
long prevMillisBlynk = 0;
int intervalBlynk = 20000;
byte counter = 0;
boolean ledState = LOW;
int constant = 10;
unsigned long prevTime;
//const int potPin = 34;
int i = 0;
// variable for storing the potentiometer value
int potValue = 0;
int maxPoint = 0;
int maxPointFinal = 0;


#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */


RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup()
{
  int count=0;
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(PIEZO, OUTPUT);
  pinMode(SIM800L_PWRKEY, OUTPUT);
  pinMode(SIM800L_RST, OUTPUT);
  pinMode(SIM800L_POWER, OUTPUT);

  digitalWrite(SIM800L_PWRKEY, LOW);
  digitalWrite(SIM800L_RST, HIGH);
  digitalWrite(SIM800L_POWER, HIGH);

  digitalWrite(LED_BLUE, LOW);

  // Debug console
  Serial.begin(115200);
  
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  /// Speed default PZEM-016 Modbus

  delay(10);

  if (!baro.begin()) {
    Serial.println("Couldn't find sensor");
  }

  // Set GSM module baud rate
  SerialAT.begin(115200, SERIAL_8N1, SIM800L_TX, SIM800L_RX);
  delay(3000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  modem.restart();

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");

  Blynk.begin(auth, modem, apn, user, pass);

  digitalWrite(LED_BLUE, HIGH);

  prevMillisSensor = millis();
  prevMillisBlynk = millis();

//Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  /*
  Next we decide what all peripherals to shut down/keep on
  By default, ESP32 will automatically power down the peripherals
  not needed by the wakeup source, but if you want to be a poweruser
  this is for you. Read in detail at the API docs
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  Left the line commented as an example of how to configure peripherals.
  The line below turns off all RTC peripherals in deep sleep.
  */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /*
  Now that we have setup a wake cause and if needed setup the
  peripherals state in deep sleep, we can now start going to
  deep sleep.
  In the case that no wake up sources were provided but deep
  sleep was started, it will sleep forever unless hardware
  reset occurs.
  */
 
 // Serial.println("This will never be printed");

}

int count = 0;

 

void loop()
 {
  Blynk.run();

  if (millis() - prevMillisSensor > intervalSensor) {
    if (!baro.begin()) {
      Serial.println("Couldn't find sensor");
    }
    else {
      pascals = baro.getPressure() / 1000;
      Serial.print(pascals);
      Serial.print(" kPa\t");

      altitude = baro.getAltitude();
      Serial.print(altitude);
      Serial.print(" meters\t");

      celsius = baro.getTemperature(); 
      Serial.print(celsius);
      Serial.println("*C");
    }

    prevMillisSensor = millis();
  }
///////////////////////////////////////////------------------------------------------------------

delay(20);


  //// -  1 step
 ///// Master frame request 10 registers (Input Registers) ///////////////////////////////////////////
 ///// Trama de Maestro solicitud 10 registros (Input Registers) ////////////////////////////////////
//       
 byte msg[] = {0xff,0x03,0x02,0x58,0x00,0x02,0x51,0xBE};
// byte msg[] = {0x01,0x03,0x02,0x58,0x00,0x02,0x44,0x60};

 int i;
 int len=8; 

////// Sending Frame Modbus for Serial Port 2 
/////// Envio de Trama Modbus por Puerto Serial #2

Serial.println("ENVIO DATOS  -  SEND DATA");
for(i = 0 ; i < len ; i++){
      Serial2.write(msg[i]); 
      Serial.print("[");
      Serial.print(i);
      Serial.print("]");
      Serial.print("=");   
      Serial.print(String(msg[i], HEX));      
 }
 len = 0;
 Serial.println();
 Serial.println();
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////





//// -  2 step
////////// Reception  Frame 10 Registers (Input Registers) Modbus RTU - RS485 ////////////////////////
/////////// Recepción de Trama 10 Registros (Input Registers) Modbus RTU - RS485 //////////////////////

int a = 0;
 while(Serial2.available()) 
 {
 ByteArray[a] = Serial2.read();
 a++;
 }

 int b = 0;
 String registros;
//    Serial.println("DATA RECEPTION  -  RECEPCIÓN DATOS");
//    for(b = 0 ; b < a ; b++){      
//      Serial.print("[");
//      Serial.print(b);
//      Serial.print("]");
//      Serial.print("=");    
    
 //     registros =String(ByteArray[b], HEX);   
//      Serial.print(registros);
//      Serial.print(" ");         
//   }
//      Serial.println();
 //      Serial.println();
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////



//// -  3 step
//////// Procesamiento de Registros HEX  //////////////////////////////////////////////////////////
//////// HEX Registers Processing ////////////////////////////////////// ////////////////////
//////// Conversion de 2 Byte a 1 int - Conversion from 2 Byte to 1 int

//Serial.println("REGISTERS HEX");

ByteData[0] = ByteArray[3] * 256 + ByteArray[4];
//Serial.println(ByteData[0],DEC);
ByteData[1] = ByteArray[5] * 256 + ByteArray[6];
//Serial.println(ByteData[1],DEC);
ByteData[2] = ByteArray[7] * 256 + ByteArray[8];
//Serial.println(ByteData[2],DEC);
//ByteData[3] = ByteArray[9] * 256 + ByteArray[10];
//Serial.println(ByteData[3],DEC);
//ByteData[4] = ByteArray[11] * 256 + ByteArray[12];
//Serial.println(ByteData[4],DEC);
//ByteData[5] = ByteArray[13] * 256 + ByteArray[14];
//Serial.println(ByteData[5],DEC);
//ByteData[6] = ByteArray[15] * 256 + ByteArray[16];
//Serial.println(ByteData[6],DEC);
//ByteData[7] = ByteArray[17] * 256 + ByteArray[18];
//Serial.println(ByteData[7],DEC);
//ByteData[8] = ByteArray[19] * 256 + ByteArray[20];
//Serial.println(ByteData[8],DEC);
//ByteData[9] = ByteArray[21] * 256 + ByteArray[22];
//Serial.println(ByteData[9],DEC);
//ByteData[10] = ByteArray[23] * 256 + ByteArray[24];
//Serial.println(ByteData[10],DEC);

//Serial.println();
//Serial.println();
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////



//// -  4 step
///////// Securities Normalization ////////////////////////////////////// /////////////////////////
/////////Normalizacion de Valores//////////////////////////////////////////////////////////////////

float Port1 ,Port2,Power,Energy,Frequency,PowerFactor;
Port1     = ByteData[0] * 1;     // Tensión (80-260VAC)
Port2     = ByteData[1] * 1;   // Corriente (0-100A)
//Power       = ByteData[3] * 0.1;     // Potencia Activa (0-23000W)
//Energy      = ByteData[5] ;          // Potencia Acumulada (0-9999kWh)
//Frequency   = ByteData[7] * 0.1;     // Frecuencia (45-65Hz)
//PowerFactor = ByteData[8] * 0.01;    // Factor de Potencia (0.00 – 1.00)  

Serial.println("MEDICIONES - MEASUREMENTS");
Serial.print("Port1 ");
Serial.println(Port1);
Serial.print("Port2 ");
Serial.println(Port2);
//Serial.print("Power ");
//Serial.println(Power);
//Serial.print("Energy ");
//Serial.println(Energy);
//Serial.print("Frequency ");
//Serial.println(Frequency);
//Serial.print("PowerFactor ");
//Serial.println(PowerFactor);
//Serial.println();
delay(50);                      /// delay para permitir ver valores - delay to allow viewing values -
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
  
//////////////////////////////////////////  ----------------------------------------------------------
potValue = analogRead(potPin);

while (i < 5) {
  maxPoint = Port1;
  if(maxPoint > maxPointFinal){
    maxPoint = maxPointFinal;
  }
  i++;
}

Serial.println(maxPointFinal);

if(maxPointFinal > 2000){
  
  setpoint = 4/10*maxPointFinal;
  
}
else{
  
  setpoint = 2000;
  
}


Serial.println("Raw Data:");
Blynk.virtualWrite(62, constant);

Serial.print("Raw Data:");
Blynk.virtualWrite(61, Port1);

if (Port1 > setpoint){
   Serial.print("Potential On:");
 Serial.println(Port1);
  Blynk.virtualWrite(62, Port1);
  prevTime = millis()+1000; 
  
}

if (Port1 < setpoint){
  
  unsigned long currentTime = millis();

  
  if(currentTime - prevTime >= 100){
    Serial.print("Potential Off:");
    Serial.println(Port1);
    Blynk.virtualWrite(63, Port1);
  }dw
  
  prevTime = currentTime;
  
}
 
  count++;
      
  if (millis() - prevMillisBlynk > intervalBlynk) {

    if (counter == 1) {
      // Up data to Blynk
      Serial.println("Publish pressure data to Blynk V0");
      Blynk.virtualWrite(0, pascals);
    }
    else if (counter == 2) {
      // Up data to Blynk
      Serial.println("Publish altitude data to Blynk V1");
      Blynk.virtualWrite(1, altitude);
    }
    else if (counter == 3) {
      // Up data to Blynk
      
      counter = 0;
    }
    counter++;

    digitalWrite(LED_BLUE, ledState);
    ledState = !ledState;

    prevMillisBlynk = millis();
  }
  to_sleep();
  delay(2000);
 
}


void to_sleep(){
  if (count == 999){
Serial.println("Going to sleep now");
  delay(4000);
//  Serial.flush(); 
  esp_deep_sleep_start();
}
else {
}


}
