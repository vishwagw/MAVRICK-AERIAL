#include <Servo.h>;

const int servo1 = 10;       // first servo
const int servo2 = 11;       // second servo
const int joyH = 5;        // L/R Parallax Thumbstick
const int joyV = 4;        // U/D Parallax Thumbstick

int ServoVal;

Servo myservo1;
Servo myservo2;

void setup() {
  // put your setup code here, to run once:
  myservo1.attach(servo1);  // attaches the servo
  //myservo2.attach(servo2);  // attaches the servo

  // Inizialize Serial
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Display Joystick values using the serial monitor
    outputJoystick();

    // Read the horizontal joystick value  (value between 0 and 1023)
    ServoVal = analogRead(joyH);          
    ServoVal = map(ServoVal, 0, 1023, 0, 180);     // scale it to use it with the servo (result  between 0 and 180)

    myservo2.write(ServoVal);                         // sets the servo position according to the scaled value    

    // Read the horizontal joystick value  (value between 0 and 1023)
    ServoVal = analogRead(joyV);           
    ServoVal = map(ServoVal, 0, 1023, 70, 180);     // scale it to use it with the servo (result between 70 and 180)

    myservo1.write(ServoVal);                           // sets the servo position according to the scaled value

    delay(15); 

}

void outputJoystick(){

    Serial.print(analogRead(joyH));
    Serial.print ("---"); 
    Serial.print(analogRead(joyV));
    Serial.println ("----------------");
}

