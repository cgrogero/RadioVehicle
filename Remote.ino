/*
 * Remote (Transmitter)
 * 
 * Created by Cameron Rogero on 7/20/2021
 *  (Resources: HowToMechatronics, lastminuteengineers.com)
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h> 

#define buttonLF A0  // LF
#define buttonLR A1  // LR
#define buttonRF A2  // RF
#define buttonRR A3  // RR
#define buttonLED A4  // LEDs
#define buttonSpeed A5  // Speed
#define buttonSound 2  // Sound 

#define LED1 3 // Slow Speed
#define LED2 4 // Fast Speed
#define LED3 5 // Turbo Speed

RF24 radio(8,9); // CE, CSN
const byte address[6] = "00001";
long code = 0000000;
// Buttons
boolean buttonStateLF = 0;
boolean buttonStateLR = 0;
boolean buttonStateRF = 0;
boolean buttonStateRR = 0;
boolean buttonStateLED = 0;
boolean buttonStateSound = 0;

int SpeedState = 0;
int buttonCurrentSpeed;
int buttonPreviousSpeed = HIGH;
long timeSpeed = 0;
long debounceSpeed = 200;

void setup() {
  pinMode(buttonLF, INPUT);
  pinMode(buttonLR, INPUT);
  pinMode(buttonRF, INPUT);
  pinMode(buttonRR, INPUT);
  pinMode(buttonLED, INPUT);
  pinMode(buttonSpeed, INPUT);
  pinMode(buttonSound, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setRetries(15, 15);
  radio.openWritingPipe(address);
  radio.stopListening();
  delay(100);
}

void loop() {
  // Read Button States
  buttonStateLF = digitalRead(A0);
  buttonStateLR = digitalRead(A1);
  buttonStateRF = digitalRead(A2);
  buttonStateRR = digitalRead(A3);
  buttonStateLED = digitalRead(A4);
  buttonCurrentSpeed = digitalRead(A5);
  buttonStateSound = digitalRead(2);

  // Write Data
  bitWrite(code, 0, buttonStateLF);
  bitWrite(code, 1, buttonStateLR);
  bitWrite(code, 2, buttonStateRF);
  bitWrite(code, 3, buttonStateRR);
  bitWrite(code, 4, buttonStateLED);
  bitWrite(code, 5, buttonCurrentSpeed);
  bitWrite(code, 6, buttonStateSound);

  // Send Data
  if (!radio.write(&code, sizeof(code))) {
    Serial.println("Failed");
  }

  // Speed
  if (buttonCurrentSpeed == HIGH && buttonPreviousSpeed == LOW && millis() - timeSpeed > debounceSpeed) {
    if (SpeedState == 0 || SpeedState == 1) {
      SpeedState += 1;
    }
    else {
      SpeedState = 0;
    }
    timeSpeed = millis();    
  }
  buttonPreviousSpeed = buttonCurrentSpeed;
  if (SpeedState == 1) {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, LOW);
  }
  else if (SpeedState == 2) {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, HIGH);
  }
  else {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
  }

 // Delay at end
  delay(30);
}
