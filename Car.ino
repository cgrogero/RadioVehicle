/*
 * Car (Receiver)
 * 
 * Created by Cameron Rogero on 7/20/2021
 *  (Resources: HowToMechatronics, lastminuteengineers.com)
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define enA 6  
#define in1 7
#define in2 3   
#define in3 4
#define in4 2
#define enB 5
#define red A0
#define yellow A1
#define white A2
#define green A3
#define blue A4
#define sound A5

#include "pitches.h"
/////////////////////////////////////////////////////////
int tempo0 = 140;

int melody0[] = {

  // Happy Birthday
  // Score available at https://musescore.com/user/8221/scores/26906

  NOTE_C4,4, NOTE_C4,8, 
  NOTE_D4,-4, NOTE_C4,-4, NOTE_F4,-4,
  NOTE_E4,-2, NOTE_C4,4, NOTE_C4,8, 
  NOTE_D4,-4, NOTE_C4,-4, NOTE_G4,-4,
  NOTE_F4,-2, NOTE_C4,4, NOTE_C4,8,

  NOTE_C5,-4, NOTE_A4,-4, NOTE_F4,-4, 
  NOTE_E4,-4, NOTE_D4,-4, NOTE_AS4,4, NOTE_AS4,8,
  NOTE_A4,-4, NOTE_F4,-4, NOTE_G4,-4,
  NOTE_F4,-2,
 
};
int notes0 = sizeof(melody0) / sizeof(melody0[0]) / 2;

// this calculates the duration of a whole note in ms
int wholenote0 = (60000 * 4) / tempo0;

int divider0 = 0, noteDuration0 = 0;

// notes in the melody:
int melody1[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations1[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};
///////////////////////////////////////////////////////
// change this to make the song slower or faster
int tempo = 85;
// notes of the moledy followed by the duration.
// a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
// !!negative numbers are used to represent dotted notes,
// so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
int melody2[] = {
  // Game of Thrones
  // Score available at https://musescore.com/user/8407786/scores/2156716

  NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, //1
  NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16,
  NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16,
  NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16,
  NOTE_G4,-4, NOTE_C4,-4,//5

  NOTE_DS4,16, NOTE_F4,16, NOTE_G4,4, NOTE_C4,4, NOTE_DS4,16, NOTE_F4,16, //6
  NOTE_D4,-1, //7 and 8
  NOTE_F4,-4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_F4,4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_C4,-1, //11 and 12

  //repeats from 5
  NOTE_G4,-4, NOTE_C4,-4,//5

  NOTE_DS4,16, NOTE_F4,16, NOTE_G4,4, NOTE_C4,4, NOTE_DS4,16, NOTE_F4,16, //6
  NOTE_D4,-1, //7 and 8
  NOTE_F4,-4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_F4,4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_C4,-1, //11 and 12
  NOTE_G4,-4, NOTE_C4,-4,
  NOTE_DS4,16, NOTE_F4,16, NOTE_G4,4,  NOTE_C4,4, NOTE_DS4,16, NOTE_F4,16,

  NOTE_D4,-2,//15
  NOTE_F4,-4, NOTE_AS3,-4,
  NOTE_D4,-8, NOTE_DS4,-8, NOTE_D4,-8, NOTE_AS3,-8,
  NOTE_C4,-1,
  NOTE_C5,-2,
  NOTE_AS4,-2,
  NOTE_C4,-2,
  NOTE_G4,-2,
  NOTE_DS4,-2,
  NOTE_DS4,-4, NOTE_F4,-4, 
  NOTE_G4,-1,
  
  NOTE_C5,-2,//28
  NOTE_AS4,-2,
  NOTE_C4,-2,
  NOTE_G4,-2, 
  NOTE_DS4,-2,
  NOTE_DS4,-4, NOTE_D4,-4,
  NOTE_C5,8, NOTE_G4,8, NOTE_GS4,16, NOTE_AS4,16, NOTE_C5,8, NOTE_G4,8, NOTE_GS4,16, NOTE_AS4,16,
  NOTE_C5,8, NOTE_G4,8, NOTE_GS4,16, NOTE_AS4,16, NOTE_C5,8, NOTE_G4,8, NOTE_GS4,16, NOTE_AS4,16,
  
  REST,4, NOTE_GS5,16, NOTE_AS5,16, NOTE_C6,8, NOTE_G5,8, NOTE_GS5,16, NOTE_AS5,16,
  NOTE_C6,8, NOTE_G5,16, NOTE_GS5,16, NOTE_AS5,16, NOTE_C6,8, NOTE_G5,8, NOTE_GS5,16, NOTE_AS5,16,  
};
// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes2 = sizeof(melody2) / sizeof(melody2[0]) / 2;
// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;
int divider = 0, noteDuration2 = 0;

RF24 radio(8, 9); // CE, CSN
const byte address[6] = "00001";
long code = 0000000;
boolean motorAOn = 0;
boolean motorBOn = 0;
// Buttons
boolean buttonStateLF = 0;  // Left Forward
boolean buttonStateLR = 0;  // Left Rear
boolean buttonStateRF = 0;  // Right Forward
boolean buttonStateRR = 0;  // Right Rear
//boolean buttonStateSound = 0; 

int LEDState = LOW;
int buttonCurrentLED;
int buttonPreviousLED = HIGH;
long timeLED = 0;
long debounceLED = 200;

int SpeedState = 0;
int buttonCurrentSpeed;
int buttonPreviousSpeed = HIGH;
long timeSpeed = 0;
long debounceSpeed = 200;

//boolean buttonCurrentSound = 0;

int SoundState = 0;
int buttonCurrentSound;
int buttonPreviousSound = HIGH;
long timeSound = 0;
long debounceSound = 200;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(white, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(sound, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setRetries(15, 15);
  radio.openReadingPipe(0, address);
  radio.startListening();

  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration1 = 1000 / noteDurations1[thisNote];
    tone(sound, melody1[thisNote], noteDuration1);
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration1 * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(sound);
  }
  delay(1000);
}

void loop() {
  if (radio.available()) {   // If the NRF240L01 module received data
    radio.read(&code, sizeof(code)); // Read Sound Button
  }
  // Read Data
  buttonStateLF = bitRead(code, 0);
  buttonStateLR = bitRead(code, 1);
  buttonStateRF = bitRead(code, 2);
  buttonStateRR = bitRead(code, 3);
  buttonCurrentLED = bitRead(code, 4);
  buttonCurrentSpeed = bitRead(code, 5);
  buttonCurrentSound = bitRead(code, 6);
  
  // Forward
  if ((buttonStateLF) && (buttonStateRF) && !(buttonStateLR) && !(buttonStateRR))
  {
    Serial.println("FORWARD");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);   
    digitalWrite(in4, LOW);
    motorAOn = 1;  // Set the Speeds
    motorBOn = 1;
  }
  // Reverse
  else if ((buttonStateLR) && (buttonStateRR) && !(buttonStateLF) && !(buttonStateRF))
  {
    Serial.println("REVERSE");
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);   
    digitalWrite(in4, HIGH);
    motorAOn = 1;  // Set the Speeds
    motorBOn = 1;
  }
  // Rotate Right
  else if ((buttonStateLF) && (buttonStateRR) && !(buttonStateLR) && !(buttonStateRF))
  {
    Serial.println("ROTATE RIGHT");
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);   
    digitalWrite(in4, LOW);
    motorAOn = 1;  // Set the Speeds
    motorBOn = 1;
  }
  // Rotate Left
  else if ((buttonStateLR) && (buttonStateRF) && !(buttonStateLF) && !(buttonStateRR))
  {
    Serial.println("ROTATE LEFT");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);   
    digitalWrite(in4, HIGH);
    motorAOn = 1;  // Set the Speeds
    motorBOn = 1;
  }
  // Right Turn
  else if ((buttonStateLF) && !(buttonStateLR) && !(buttonStateRF) && !(buttonStateRR)) {
    // Left Motor Forward
    Serial.println("RIGHT");
    digitalWrite(in3, HIGH);   
    digitalWrite(in4, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    motorAOn = 0;  // Set the Speeds
    motorBOn = 1;
  }
  // Back Up Right
  else if ((buttonStateLR) && !(buttonStateLF) && !(buttonStateRF) && !(buttonStateRR)) {
    // Left Motor Reverse
    Serial.println("LEFT REVERSE");
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    motorAOn = 0;  // Set the Speeds
    motorBOn = 1;
  }
  // Left Turn
  else if ((buttonStateRF) && !(buttonStateRR) && !(buttonStateLF) && !(buttonStateLR)) {
    // Right Motor Forward
    Serial.println("LEFT");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    motorAOn = 1;  // Set the Speeds
    motorBOn = 0;
  }
  // Back Up Left
  else if ((buttonStateRR) && !(buttonStateRF) && !(buttonStateLF) && !(buttonStateLR)) {
    // Right Motor Reverse
    Serial.println("RIGHT REVERSE");
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    motorAOn = 1;  // Set the Speeds
    motorBOn = 0;
  }
  // Don't Move
  else {
    Serial.println("STOP");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    motorAOn = 0;  // Set the Speeds
    motorBOn = 0;
  }

///////////////////////////////////////////////////////////////////////////////
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

  // Send Data to Motor A
  if (motorAOn) {
    if (SpeedState == 0) {
      analogWrite(enA, 150); // Send PWM signal to motor A
    }
    else if (SpeedState == 1) {
      analogWrite(enA, 200); // Send PWM signal to motor A
    }
    else if (SpeedState == 2) {
      analogWrite(enA, 255); // Send PWM signal to motor A
    }
    else {
      analogWrite(enA, 0); // Send PWM signal to motor A
    }
  }
  else {
    analogWrite(enA, 0); // Send PWM signal to motor A
  }

  // Send Data to Motor B
  if (motorBOn) {
      if (SpeedState == 0) {
        analogWrite(enB, 150); // Send PWM signal to motor B
      }
      else if (SpeedState == 1) {
        analogWrite(enB, 200); // Send PWM signal to motor B
      }
      else if (SpeedState == 2) {
        analogWrite(enB, 255); // Send PWM signal to motor B
      }
      else {
        analogWrite(enB, 0); // Send PWM signal to motor B
      }
  }
  else {
    analogWrite(enB, 0); // Send PWM signal to motor B
  }

  ///////////////////////////////////////////////////////////////////////////////
  //LEDs
  if (buttonCurrentLED == HIGH && buttonPreviousLED == LOW && millis() - timeLED > debounceLED) {
    if (LEDState == HIGH) {
      LEDState = LOW;
    }
    else {
      LEDState = HIGH;
    }
    timeLED = millis();    
  }
  buttonPreviousLED = buttonCurrentLED;
  
  if (LEDState) {
    if (SpeedState == 1) {
      digitalWrite(red, HIGH);
      digitalWrite(yellow, HIGH);
      digitalWrite(white, LOW);
      digitalWrite(green, HIGH);
      digitalWrite(blue, HIGH);
    }
    else if (SpeedState == 2) {
      digitalWrite(red, HIGH);
      digitalWrite(yellow, HIGH);
      digitalWrite(white, HIGH);
      digitalWrite(green, HIGH);
      digitalWrite(blue, HIGH);
    }
    else {
      digitalWrite(red, HIGH);
      digitalWrite(yellow, LOW);
      digitalWrite(white, LOW);
      digitalWrite(green, LOW);
      digitalWrite(blue, HIGH);
    }
  }
  else {
    digitalWrite(red, LOW);
    digitalWrite(yellow, LOW);
    digitalWrite(white, LOW);
    digitalWrite(green, LOW);
    digitalWrite(blue, LOW);
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Sound

  if (buttonCurrentSound == HIGH && buttonPreviousSound == LOW && millis() - timeSound > debounceSound) {
    if (SoundState == 0 || SoundState == 1) {
      SoundState += 1;
    }
    else {
      SoundState = 0;
    }
    timeSound = millis();    
  }
  buttonPreviousSound = buttonCurrentSound;

  // Send Data to Speaker
  // Song 1
  if (SoundState == 1) {
    for (int thisNote = 0; thisNote < notes0 * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider0 = melody0[thisNote + 1];
    if (divider0 > 0) {
      // regular note, just proceed
      noteDuration0 = (wholenote0) / divider0;
    } else if (divider0 < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration0 = (wholenote0) / abs(divider0);
      noteDuration0 *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(sound, melody0[thisNote], noteDuration0 * 0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration0);

    // stop the waveform generation before the next note.
    noTone(sound);
    }
    SoundState = 0;
  }
  // Song 2
  else if (SoundState == 2){
    // iterate over the notes of the melody:
    for (int thisNote = 0; thisNote < 8; thisNote++) {
      // to calculate the note duration, take one second divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration1 = 1000 / noteDurations1[thisNote];
      tone(sound, melody1[thisNote], noteDuration1);
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration1 * 1.30;
      delay(pauseBetweenNotes);
      // stop the tone playing:
      noTone(sound);
    }
    delay(50);
    SoundState = 0;
  }
  // Song 3
  // iterate over the notes of the melody.
  // Remember, the array is twice the number of notes (notes + durations)
  else if (SoundState == 3) {
    for (int thisNote = 0; thisNote < notes2 * 2; thisNote = thisNote + 2) {
      // calculates the duration of each note
      divider = melody2[thisNote + 1];
      if (divider > 0) {
        // regular note, just proceed
        noteDuration2 = (wholenote) / divider;
      } else if (divider < 0) {
        // dotted notes are represented with negative durations!!
        noteDuration2 = (wholenote) / abs(divider);
        noteDuration2 *= 1.5; // increases the duration in half for dotted notes
      }
      // we only play the note for 90% of the duration, leaving 10% as a pause
      tone(sound, melody2[thisNote], noteDuration2 * 0.9);
      // Wait for the specief duration before playing the next note.
      delay(noteDuration2);
      // stop the waveform generation before the next note.
      noTone(sound);
    }
    delay(1000);
  }
  else {
    noTone(sound);
  }
}
