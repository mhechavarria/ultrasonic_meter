/*
 * created by Rui Santos, https://randomnerdtutorials.com
 * 
 * Complete Guide for Ultrasonic Sensor HC-SR04
 *
    Ultrasonic sensor Pins:
        VCC: +5VDC
        Trig : Trigger (INPUT) - Pin11
        Echo: Echo (OUTPUT) - Pin 12
        GND: GND
 */
// include the library code:
#include <LiquidCrystal.h>
#include <Kalman.h>
#include "OneMsTaskTimer.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
OneMsTaskTimer_t myTask1 ={2000,  lcd_print, 0, 0};


// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(35, 36, 37, 38, 39, 40);
 
int trigPin = 19;    // Trigger
int echoPin = 18;    // Echo
long duration, cm, buffer; 
volatile long vSonido, Tamb;
double distancia, distanciaFiltrada;
unsigned status;
int i=0, promedio;

//Kalman miFiltro(0.125,32,1023,0);
Kalman miFiltro(0.3,20,1023,0);

 
void setup() {
  //Serial Port begin
  Serial.begin (115200);  
  status = bme.begin();  
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1);
  } 
  delay(250);  
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Distancia: ");
  lcd.setCursor(0, 1);
  lcd.print("     cm");
  lcd.setCursor(0, 1);
  lcd.print(cm);
  
  OneMsTaskTimer::add(&myTask1);
  OneMsTaskTimer::start();
   
}
 
void loop() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  Tamb = bme.readTemperature();
 
  // Convert the time into a distance
  cm = ((duration * vSonido)/2)/10000;
  distancia = (double) cm;
  distanciaFiltrada = miFiltro.getFilteredValue(cm);
  Serial.print(cm);
  //Serial.print("cm");
  //Serial.println();  
  //Serial.print("Fitrada: ");
  Serial.print(" ");
  Serial.print(distanciaFiltrada);
  Serial.print(" ");
  Serial.println(promedio);
  /*Serial.print("cm");
  Serial.println();
  Serial.println();*/
  if(i==10){
    i=0;
    promedio= round(buffer/10);
    buffer=0;
  }
  buffer += distanciaFiltrada;
  i++;
  delay(5);
}

void lcd_print(){
    vSonido = 331.3 + (0.606 * Tamb);
    lcd.clear();
    lcd.print("Distancia: ");      
    lcd.setCursor(0, 1);
    lcd.print(round(promedio));
    lcd.print(" cm - ");
    lcd.print(Tamb);
    lcd.print(" *C");
    
}
