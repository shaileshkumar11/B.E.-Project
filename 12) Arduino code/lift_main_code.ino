const int trigPin = 25;
const int echoPin = 24;
long duration;
int distance;
int alert;
int in0, in1, in2;
///////////////////////////////////////////////////////////////////

const int channelM0 = 5;
const int channelM1 = 6;
const int channelM2 = 7;

/////////////////////////////////////
const int irSensorPin3 = 47;
const int irSensorPin2 = 45;
const int irSensorPin1 = 26;
//////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
///////////////////////////////////////////////////////////////////



int irSensorValue3;
int irSensorValue2;
int irSensorValue1;
////////////////////////////////////////////////////////////////////////////////////
byte pinsOut[] = { 30, 31, 32, 33, 34, 35, 36, 37 };
int m = 0;  // variable for storing the pin of APR33A3

/////////////////////////////////////////////////////////////////////////////////////////

/**
  ******************************************************************************
  * @file    vr_sample_control_led.ino
  * @author  JiapengLi
  * @brief   This file provides a demostration on 
              how to control led by using VoiceRecognitionModule
  ******************************************************************************
  * @note:
        voice control led
  ******************************************************************************
  * @section  HISTORY
    
    2013/06/13    Initial version.
  */
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"
/**        
  Connection
  Arduino    VoiceRecognitionModule
   2   ------->     TX
   3   ------->     RX
*/
VR myVR(10, 11);  // 2:RX 3:TX, you can choose your favourite pins.

uint8_t records[7];  // save record
uint8_t buf[64];

int led = 13;

#define ground (0)
#define first (1)
#define second (2)

/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf     --> command length
           len     --> number of parameters
*/
void printSignature(uint8_t *buf, int len)
{
  int i;
  for(i=0; i<len; i++){
    if(buf[i]>0x19 && buf[i]<0x7F){
      Serial.write(buf[i]);
    }
    else{
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}

/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf  -->  VR module return value when voice is recognized.
             buf[0]  -->  Group mode(FF: None Group, 0x8n: User, 0x0n:System
             buf[1]  -->  number of record which is recognized. 
             buf[2]  -->  Recognizer index(position) value of the recognized record.
             buf[3]  -->  Signature length
             buf[4]~buf[n] --> Signature
*/
void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if(buf[0] == 0xFF){
    Serial.print("NONE");
  }
  else if(buf[0]&0x80){
    Serial.print("UG ");
    Serial.print(buf[0]&(~0x80), DEC);
  }
  else{
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if(buf[3]>0){
    printSignature(buf+4, buf[3]);
  }
  else{
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}
////////////////////////////////////////////////////////////////////////
const int ANALOG_PIN = A8;
int sensorValue;
/////////////////////////////////////////////////////////////////////////////
const int motor1A = 22;  // IN1
const int motor1B = 23;  // IN2
////////////////////////////////////////////////////////////////////
const int buttonPin1 = 50;
const int buttonPin2 = 53;
const int buttonPin3 = 52;
int flagup;
int flagdown;
int flagmedium;
int status;
////////////////////////////////////////////////////////////////////////////////////
#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
const int HX711_dout = 49;  //mcu > HX711 dout pin
const int HX711_sck = 48;   //mcu > HX711 sck pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_eepromAdress = 0;
unsigned long t = 0;
float i;
///////////////////////////////////////////////////////////////////
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 51
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float temperatureC;
//////////////////////////////////////////////////////////////////
void setup() {
  pinMode(channelM0, OUTPUT);
  pinMode(channelM1, OUTPUT);
  pinMode(channelM2, OUTPUT);
  digitalWrite(channelM0, HIGH);
  digitalWrite(channelM1, HIGH);
  digitalWrite(channelM2, HIGH);

  for (unsigned n = 0; n < 8; n++) {
    pinMode(pinsOut[n], OUTPUT);
    digitalWrite(pinsOut[n], HIGH);
  }
  Serial.begin(9600);
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  /////////////////////////////////////////////////////////////////////////
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
  ///////////////////////////////////////////////////////////////////////////
  sensors.begin();
  pinMode(irSensorPin1, INPUT);
  pinMode(irSensorPin2, INPUT);
  pinMode(irSensorPin3, INPUT);
  //////////////////////////////////////////////////////////////////////
  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  float calibrationValue;    // calibration value (see example file "Calibration.ino")
  calibrationValue = 696.0;  // uncomment this if you want to set the calibration value in the sketch
  #if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
  #endif
  EEPROM.get(calVal_eepromAdress, calibrationValue);  // uncomment this if you want to fetch the calibration value from eeprom

  unsigned long stabilizingtime = 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1)
      ;
  } else {
    LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
    Serial.println("Startup is complete");
  }
  //////////////////////////////////////////////////////////
  myVR.begin(9600);
  
  Serial.println("Elechouse Voice Recognition V3 Module\r\nControl LED sample");



  if (myVR.clear() == 0) {
    Serial.println("Recognizer cleared.");
  } else {
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while (1);
  }

  if (myVR.load((uint8_t)ground) >= 0) {
    Serial.println("ground loaded");
  }

  if (myVR.load((uint8_t)first) >= 0) {
    Serial.println("first loaded");
  }
  if (myVR.load((uint8_t)second) >= 0) {
    Serial.println(" second loaded");
  }
  /*

  digitalWrite(pinsOut[7], LOW);
  delay(1000);
  digitalWrite(pinsOut[7], HIGH);
  delay(1000)  ;
  digitalWrite(pinsOut[8], LOW);
  delay(1000);
  digitalWrite(pinsOut[8], HIGH);
  delay(1000)  ;
  digitalWrite(pinsOut[0], LOW);
  delay(1000);
  digitalWrite(pinsOut[0], HIGH);
  delay(1000)  ;
 */
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Hello, world!");
  lcd.setCursor(2, 1);
  lcd.print("Ywrobot Arduino!");
  lcd.clear();
}

void loop() {
  lcd.setCursor(5, 0);
  lcd.print("Welcome");
  lcd.setCursor(0, 1);
  lcd.print("Temp= ");
  lcd.setCursor(6, 1);
  lcd.print(temperatureC);
  flame_lift();
  temp_lift();
  if (temperatureC >= 50 || sensorValue <= 200) {
    lift_stop();
    digitalWrite(pinsOut[1], LOW);
    delay(1000);
    digitalWrite(pinsOut[1], HIGH);
    delay(1000);
  } 
  else {
    int ret;
    ret = myVR.recognize(buf, 50);
    if (ret > 0) {
      switch (buf[1]) {
        case ground:
          in0 = 1;
          break;
        case first:
          in1 = 1;
          break;
        case second:
          in2 = 1;
          break;
        default:
          Serial.println("Record function undefined");
          break;
      }
      /** voice recognized */
      printVR(buf);
    }
    int last_floor = digitalRead(buttonPin1);
    int second_Floor = digitalRead(buttonPin2);
    int ground_floor = digitalRead(buttonPin3);
    irSensorValue1 = digitalRead(irSensorPin1);
    irSensorValue2 = digitalRead(irSensorPin2);
    irSensorValue3 = digitalRead(irSensorPin3);
    if (last_floor == HIGH || irSensorValue3 == LOW) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("plz wait ");
      lcd.setCursor(0, 1);
      lcd.print("lift load check");
      delay(1000);
      lift_load_check();
      flame_lift();
      temp_lift();
      status = 2;
      flagup = 1;
    }
    if (second_Floor == HIGH || irSensorValue2 == LOW) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("plz wait ");
      lcd.setCursor(0, 1);
      lcd.print("lift load check");
      delay(1000);
      lift_load_check();
      flame_lift();
      temp_lift();
      flagmedium = 1;
    }
    if (ground_floor == HIGH || irSensorValue1 == LOW) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("plz wait ");
      lcd.setCursor(0, 1);
      lcd.print("lift load check");
      delay(1000);
      lift_load_check();
      flame_lift();
      temp_lift();
      flagdown = 1;
      status = 1;
    }
    /////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    //...........................................................................................................
    if (flagdown == 1 || in0 == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Lift Call");
      lcd.setCursor(0, 1);
      lcd.print("Ground Floor");
      delay(1000);
      lcd.clear();
      digitalWrite(pinsOut[2], LOW);
      delay(500);
      digitalWrite(pinsOut[2], HIGH);
      delay(500);
      lift_down_ground();
    }
    if (flagup == 1 || in2 == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Lift Call");
      lcd.setCursor(0, 1);
      lcd.print("Second Floor");
      delay(1000);
      lcd.clear();
      digitalWrite(pinsOut[3], LOW);
      delay(500);
      digitalWrite(pinsOut[3], HIGH);
      delay(500);
      lift_up_last();
    }
    if (flagmedium == 1 || in1 == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Lift Call");
      lcd.setCursor(0, 1);
      lcd.print("First Floor");
      delay(1000);
      lcd.clear();
      digitalWrite(pinsOut[4], LOW);
      delay(500);
      digitalWrite(pinsOut[4], HIGH);
      delay(500);
      switch (status) {
        case 1:
          lift_medium_up();
          Serial.println("Sensor value is 1");
          break;
        case 2:
          lift_medium_ground();
          Serial.println("Sensor value is 2");
          break;
        default:
          Serial.println("Sensor value is not 1, 2, or 3");
          break;
      }
    }
  }
}

void ultra() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.0351 / 2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void lift_up_last() {
  if (temperatureC >= 50 || sensorValue <= 200) {
    lift_stop();
  } 
  else {
    ultra();
    if (distance >= 10) {
      digitalWrite(motor1A, HIGH);
      digitalWrite(motor1B, LOW);
      status = 2;
    } 
    else {
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, LOW);
      digitalWrite(channelM2, LOW);
      delay(300);
      digitalWrite(channelM2, HIGH);
      flagup = 0;
      in0 = 0;
      in1 = 0;
      in2 = 0;
    }
  }
}

void lift_down_ground() {
  if (temperatureC >= 50 || sensorValue <= 200) {
    lift_stop();
  } 
  else {
    ultra();
    if (distance <= 40) {
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, HIGH);
      status = 1;
    } 
    else {
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, LOW);
      digitalWrite(channelM0, LOW);
      delay(300);
      digitalWrite(channelM0, HIGH);
      flagdown = 0;
      in0 = 0;
      in1 = 0;
      in2 = 0;
    }
  }
}

void lift_medium_up() {
  if (temperatureC >= 50 || sensorValue <= 200) {
    lift_stop();
  } 
  else {
    ultra();
    if (distance >= 25) {
      digitalWrite(motor1A, HIGH);
      digitalWrite(motor1B, LOW);
    } 
    else {
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, LOW);
      digitalWrite(channelM1, LOW);
      delay(300);
      digitalWrite(channelM1, HIGH);
      flagmedium = 0;
      in0 = 0;
      in1 = 0;
      in2 = 0;
      status = 0;
    }
  }
}

void lift_medium_ground() {
  if (temperatureC >= 50 || sensorValue <= 200) {
    lift_stop();
  } 
  else {
    ultra();
    if (distance <= 25) {
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, HIGH);
    } 
    else {
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, LOW);
      flagmedium = 0;
      in0 = 0;
      in1 = 0;
      in2 = 0;
      status = 0;
    }
  }
}

void lift_stop() {
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);
  delay(1000);
  in0 = 0;
  in1 = 0;
  in2 = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting ");
  lcd.setCursor(0, 1);
  lcd.print("Temp= ");
  lcd.setCursor(6, 1);
  lcd.print(temperatureC);
  lcd.clear();
}

void temp_lift() {
  sensors.requestTemperatures();
  temperatureC = sensors.getTempCByIndex(0);
  if (temperatureC != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.println("°C");
  } 
  else {
    Serial.println("Error reading temperature!");
  }
  if (temperatureC <= 50) {
    Serial.println("temp ok");
    alert = 0;
  } 
  else {
    Serial.println("temp HIGH");
    alert = 1;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Lift Temp");
    lcd.setCursor(5, 1);
    lcd.print("HIGH"); 
  }
}

void flame_lift() {
  sensorValue = analogRead(ANALOG_PIN);
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue);
  if (sensorValue >= 200) {
    Serial.println("no fire");
    alert = 0;
  } 
  else {
    Serial.println("Fire detected");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Lift Fire");
    lcd.setCursor(0, 1);
    lcd.print("detected");
    digitalWrite(pinsOut[5], LOW);
    delay(500);
    digitalWrite(pinsOut[5], HIGH);
    delay(500);
    alert = 1;
  }
}

void weightCheck() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;  //increase value to slow down serial print activity
  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;
  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      i = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      newDataReady = 0;
      t = millis();
    }
  }
  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }
  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}

void lift_load_check() {
  for (int i = 1; i <= 500; i++) {
    weightCheck();
    Serial.println(i);
  }
  if (i < 1) {
    Serial.println("lift not Overload");
    lcd.setCursor(0, 0);
    lcd.print(" lift NOT Overload ");
    lcd.setCursor(5, 1);
    lcd.print(" Overload ");
    lcd.clear();
  }
  else { 
    lcd.setCursor(0, 0);
    lcd.print(" lift Overload ");
    lcd.clear();
    Serial.println("lift Overload");
    digitalWrite(pinsOut[6], LOW);
    delay(500);
    digitalWrite(pinsOut[6], HIGH);
    delay(500);
    flagup = 0;
    flagmedium = 0;
    flagdown = 0;
    lift_stop();
    delay(1000);
  }
}