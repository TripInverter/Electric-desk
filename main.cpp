#include <Arduino.h>

const int buttonPin = 12;   // Pin für den Taster
const int motor1_IN1 = 2;   // IN1 Pin für Motor 1
const int motor1_IN2 = 3;   // IN2 Pin für Motor 1
const int motor2_IN3 = 4;   // IN3 Pin für Motor 2
const int motor2_IN4 = 5;   // IN4 Pin für Motor 2
/*
const int sensor1 = 8;  // Pins für Sensoren 1-4
const int sensor2 = 9;  //
const int sensor3 = 10;  // 
const int sensor4 = 11; //
*/

int buttonState = 0;        // Aktueller Zustand des Tasters
int lastButtonState = 0;    // Vorheriger Zustand des Tasters
unsigned long lastPressTime = 0;  // Zeitpunkt der letzten Betätigung
unsigned long debounceDelay = 50;  // Entprellzeit
int buttonPressCount = 0;   // Zähler für die Anzahl der Tastendrücke
const long pressTimeout = 500; // Timeout für Tastendrücke in Millisekunden

bool isMovingForward = false; // Flag, ob beide Motoren vorwärts laufen
bool isMovingBackward = false; // Flag, ob beide Motoren rückwärts laufen
int motorSpeed = HIGH;         // Startgeschwindigkeit auf 75% (PWM-Wert 191)

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);  // Taster-Pin mit internem Pull-Up-Widerstand
  pinMode(motor1_IN1, OUTPUT);      // Motor 1 IN1
  pinMode(motor1_IN2, OUTPUT);      // Motor 1 IN2
  pinMode(motor2_IN3, OUTPUT);      // Motor 2 IN3
  pinMode(motor2_IN4, OUTPUT);      // Motor 2 IN4
/*
  pinMode(sensor1, INPUT);      // Sensor Pin für Motor 1
  pinMode(sensor2, INPUT);      // Sensor Pin für Motor 2
  pinMode(sensor3, INPUT);      // Sensor Pin für Motor 3
  pinMode(sensor4, INPUT);      // Sensor Pin für Motor 4
*/

  Serial.begin(9600);
}

void loop() {
  int reading = digitalRead(buttonPin);  // Tasterzustand lesen

  // Entprellung
  if (reading != lastButtonState) {
    lastPressTime = millis();
  }

  if ((millis() - lastPressTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {  // Taster gedrückt
        // Zähler erhöhen, wenn der Button gedrückt wird
        buttonPressCount++;
        Serial.print("Taster wurde gedrückt: ");
        Serial.println(buttonPressCount);

        // Zähler zurücksetzen nach Timeout
        lastPressTime = millis();
      }

      // Wenn der Taster losgelassen wird, stoppe die Motoren
      if (buttonState == HIGH) {
        if (isMovingForward) {
          stopMotors();
        }
        if (isMovingBackward) {
          stopMotors();
        }
      }
    }
  }

  // Nach einem bestimmten Timeout (500ms) Zähler zurücksetzen
  if (millis() - lastPressTime > pressTimeout) {
    if (buttonPressCount == 2) {
      // Zweimaliges Drücken: Beide Motoren vorwärts
      moveMotorsForward();
    } 
    else if (buttonPressCount == 3) {
      // Dreifaches Drücken: Beide Motoren rückwärts
      moveMotorsBackward();
    }
    buttonPressCount = 0; // Zähler zurücksetzen
  }

  lastButtonState = reading;
}

// Funktion, um beide Motoren vorwärts zu bewegen
void moveMotorsForward() {
  if (!isMovingForward && !isMovingBackward) {
    isMovingForward = true;
    Serial.println("Beide Motoren fahren vorwärts");
    digitalWrite(motor1_IN1, HIGH);  // Motor 1 vorwärts
    digitalWrite(motor1_IN2, LOW);
    digitalWrite(motor2_IN3, HIGH);  // Motor 2 vorwärts
    digitalWrite(motor2_IN4, LOW);
  }
}

// Funktion, um beide Motoren rückwärts zu bewegen
void moveMotorsBackward() {
  if (!isMovingForward && !isMovingBackward) {
    isMovingBackward = true;
    Serial.println("Beide Motoren fahren rückwärts");
    digitalWrite(motor1_IN1, LOW);   // Motor 1 rückwärts
    digitalWrite(motor1_IN2, HIGH);
    digitalWrite(motor2_IN3, LOW);   // Motor 2 rückwärts
    digitalWrite(motor2_IN4, HIGH);
  }
}

// Funktion, um beide Motoren zu stoppen
void stopMotors() {
  digitalWrite(motor1_IN1, LOW);   // Motor 1 gestoppt
  digitalWrite(motor1_IN2, LOW);
  digitalWrite(motor2_IN3, LOW);   // Motor 2 gestoppt
  digitalWrite(motor2_IN4, LOW);
  isMovingForward = false;
  isMovingBackward = false;
  Serial.println("Beide Motoren gestoppt");
}

