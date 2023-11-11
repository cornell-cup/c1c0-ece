#include <XBOXONE.h>
#include <Servo.h>
#include <SPI.h>

/**
How to use Baby Yoda:
Setup requirements:
- Arduino Uno
- USB Host Shield 2.0
- USB to USB-C cable
- XBOX ONE (S) controller
- Solder?
Setup process:
- Make sure the 3.3 / 5V pin connections and the 5V VBUS PWR connection are soldered. If there's no solder on them, solder them.
- Connect the controller to the cable to the host shield to the Uno to your PC.
- Upload the sketch to the Arduino. 
- Turn the controller on. 
    (Note: This part can be finicky. If the controller is on and doesn't work, turn the Arduino off, 
    then turn the Arduino and controller on simultaneously. Repeat until it responds to input.)

Usage:
- The arrow keys control what servos the joysticks control. Pressing LEFT will make them control the left arm/elbow, RIGHT will make them control the right arm/elbow,
and UP will make them control the head/neck. DOWN will make them control nothing.
- The joysticks control what direction the servos rotate in. Holding right will make them rotate to the right and vice-versa.
- The triggers control the speed the servos rotate at. Pressing LT will make them rotate slower and RT will make them rotate faster. 
*/


USB Usb;
XBOXONE Xbox(&Usb);

//all servos are 0-180 degrees
Servo leftArm;
int leftArmState = 1500;
Servo leftElbow;
int leftElbowState = 1500;

Servo rightArm;
int rightArmState = 1500;
Servo rightElbow;
int rightElbowState = 1500;

Servo head;
int headState = 1500;
Servo neck;
int neckState = 1500;

//0: left arm, 1: right arm, 2: head
int joystickState = 0;
Servo currentLeft = leftArm;
Servo currentRight = leftElbow;
int *currentLeftState = &leftArmState;
int *currentRightState = &leftElbowState;
int speed = 1;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  //can't use pins 10, 11, 12, 13 on SPI
  //leftArmServo.attach(6);
  //leftElbowServo.attach(7);
  Serial.print(F("\r\nSetup complete"));
}
void loop() {
  Usb.Task();
  if (Xbox.XboxOneConnected) {s
    if (Xbox.getButtonClick(UP)) {
      joystickState = 2;
      Serial.println(F("Up"));
    }
    if (Xbox.getButtonClick(LEFT)) {
      joystickState = 0;
      Serial.println(F("Left"));
    }
    if (Xbox.getButtonClick(RIGHT)) {
      joystickState = 1;
      Serial.println(F("Right"));
    }
    if (Xbox.getButtonClick(DOWN)) {
      joystickState = 3;
      Serial.println(F("Down"));
    }
    if (Xbox.getButtonClick(LT)) {
      Serial.println(F("LT"));
      speed -= 1;
      if(speed < 1) {
        speed = 1;
      }
    }
    if (Xbox.getButtonClick(RT)) {
      Serial.println(F("RT"));
      speed += 1;
      if(speed > 10) {
        speed = 10;
      }
    }

    int16_t leftX = Xbox.getAnalogHat(LeftHatX);
    int16_t rightX = Xbox.getAnalogHat(RightHatX);
    if(joystickState == 0) {
      currentLeft = leftArm;
      currentRight = leftElbow;
      currentLeftState = &leftArmState;
      currentRightState = &leftElbowState;
    }
    else if(joystickState == 1) {
      currentLeft = rightArm;
      currentRight = rightElbow;
      currentLeftState = &rightArmState;
      currentRightState = &rightElbowState;
    }
    else if(joystickState == 2) {
      currentLeft = head;
      currentRight = neck;
      currentLeftState = &headState;
      currentRightState = &neckState;
    }
    //may need special handling for head / neck?
    //TODO: change speed depending on joystick magnitude?
    if (joystickState != 3 && (leftX > 7500 || leftX < -7500)) {
      if(leftX > 0) {
        *currentLeftState = min(2000, *currentLeftState + speed * 5);
      } 
      else {
        *currentLeftState = max(1000, *currentLeftState - speed * 5);
      }     
      //currentLeft.writeMicroseconds(*currentLeftState);
      Serial.println(*currentLeftState);
    }
    if (rightX > 7500 || rightX < -7500) {
      if(rightX > 0) {
        *currentRightState = min(2000, *currentRightState + speed * 5);
      } 
      else {
        *currentRightState = max(1000, *currentRightState - speed * 5);
      }     
      //currentRight.writeMicroseconds(*currentRightState);
      Serial.println(*currentRightState);
    }
    if(leftX > 7500 || leftX < -7500 || rightX > 7500 || rightX < -7500) {
      delay(100);
    }
  }
  delay(1);
}
