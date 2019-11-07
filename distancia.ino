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

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(35, 36, 37, 38, 39, 40);
 
int trigPin = 19;    // Trigger
int echoPin = 18;    // Echo
long duration, cm;
double distancia, distanciaFiltrada;

Kalman miFiltro(0.125,32,1023,0);
 
void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Distancia: ");
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
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  distancia = (double) cm;
  distanciaFiltrada = miFiltro.getFilteredValue(distancia);
  Serial.print((int)distancia);
  //Serial.print("cm");
  //Serial.println();  
  //Serial.print("Fitrada: ");
  Serial.print(" ");
  Serial.print((int)distanciaFiltrada);
  /*Serial.print("cm");
  Serial.println();
  Serial.println();*/
  lcd.setCursor(0, 1);
  lcd.print("    ");
  lcd.print(cm);
  
  delay(1000);
}
