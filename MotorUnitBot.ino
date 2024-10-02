/**
   Motor Unit Bot

   It uses Adafruit_MotorShield v2 from Adafruit.

   Tesaki 4 DC Motor from the broken servo 6 V -> M1
   Shoulder 2 DC Motor 12 V -> M2
   Shoulder Arm 3 DC Motor 12 V -> M3
   Gripper -> M4
  

   Servo Wrist 9V 180 -> Servo 1 Pin 10  White, Grey Violet
   Servo Elbow 9V 360 -> Servo 2 Pin 9 Coaxial  Black White Grey
     Power from AC Adapter 5V 3.5 A

   Encoder (shoulder)
   Pin 12 connected to CLK on KY-040
   Pin 13 connected to DT  on KY-040
   + to Arduino 5V
   GND to Arduino GND

   Encoder (Clavicle)
   Pin 5 connected to CLK on KY-040
   Pin 6 connected to DT  on KY-040
   + to Arduino 5V
   GND to Arduino GND

   Button Gate
   Pull Up Resistor on 3.

   Tilt Sensor (Elbow)
    G R Y
    GND
    R - +5v
    Y -- Connected to 4

  // Protocol Commands
  // A1220 >> Open grip
  // A2255 >> Close Grip
  // A6090 >> 90 deg wrist A6010 --> A6180
  // A7150 will keep the shoulder at zero encoder angle arm vertical.
  //       So AA140 will pull it up
  // A8220 Wrist clockwise A9220 counter
  // AA090 -> Elbow is now servo.
  // AC150 -> Clavicle resting
  // Format A5000  Reset everything.

*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

bool debug = false;
const int laserPin = 8;
const int switchPin = 3;
const int tilt_pin = 4;

class ControlledServo {
public:
  Servo servo;
  int pos;
  int tgtPos;
  int direction = 1;
  int minPos=0;
  int maxPos=180;

  void loop() {
      // Update desired position.
      if (pos<tgtPos)
        direction = 1;
      else
        direction =-1;
  }

  void update() {

    loop();
    if (tgtPos != pos)
    {
      //Serial.print(pos);Serial.print("--");
      //Serial.println(tgtPos);
      servo.write(pos);

      pos+=direction;

      if (pos<minPos)
      {
        //Serial.print("Reset down:");
        //Serial.println(counter++);
        direction=-1;
      }

      if (pos>maxPos)
      {
        //Serial.print("Reset up:");
        //Serial.println(counter++);
        direction=1;
      }
    }

  }
};


class EncoderDC {
public:
  int pinA = 6;  // Connected to CLK on KY-040
  int pinB = 5;  // Connected to DT on KY-040
  int encoderPosCount;
  int pinALast;
  int aVal;
  boolean bCW;
  void setupEncoder(int cPinA, int cPinB) {
    pinA = cPinA;
    pinB = cPinB;
    pinMode (pinA, INPUT);
    pinMode (pinB, INPUT);
    encoderPosCount = 0;
    /* Read Pin A
      Whatever state it's in will reflect the last position
    */
    pinALast = digitalRead(pinA);
  }
  void updateEncoder() {
    aVal = digitalRead(pinA);
    if (aVal != pinALast) { // Means the knob is rotating
      // if the knob is rotating, we need to determine direction
      // We do that by reading pin B.
      if (digitalRead(pinB) != aVal) {  // Means pin A Changed first - We're Rotating Clockwise
        encoderPosCount ++;
        bCW = true;
      } else {// Otherwise B changed first and we're moving CCW
        bCW = false;
        encoderPosCount--;
      }
      if (debug) Serial.print ("Rotated: ");
      if (bCW) {
        if (debug) Serial.println ("clockwise");
      } else {
        if (debug) Serial.println("counterclockwise");
      }
      if (debug) Serial.print("Encoder Position: ");
      if (debug) Serial.println(encoderPosCount);

    }
    pinALast = aVal;
  }


  int getEncoderPos()
  {
    return encoderPosCount;
  }

  void resetEncoderPos()
  {
    encoderPosCount=0;
  }
};

EncoderDC clavicleEncoder;
EncoderDC shoulderEncoder;

ControlledServo wrist;
ControlledServo elbow;

int state = 0;
int controlvalue = 255;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *shoulder = AFMS.getMotor(2);
Adafruit_DCMotor *clavicle = AFMS.getMotor(3);
Adafruit_DCMotor *tesaki = AFMS.getMotor(1);
Adafruit_DCMotor *grip = AFMS.getMotor(4);

int elbowcounter = 0;
int tesakicounter = 0;
int grippercounter = 0;

int targetPosShoulder=0;
int targetPosClavicle=0;

float targetposelbow=1.72;

bool homing2=false;
bool homing=false;

int incomingByte = 0;

bool tilted=false;

char buffer[5];
// =========== DC Control using the encoder.
//int targetpos = 0;
int TORQUE=200;


int setThisTargetPos(int newtargetpos)
{
  return newtargetpos;
}


bool updatethisdc(Adafruit_DCMotor *dcmotor, int torque,int thistargetpos,int currentpos)
{
  if (thistargetpos != currentpos)
  {
    dcmotor->setSpeed(torque);

    if (thistargetpos < currentpos)
      dcmotor->run(FORWARD);
    else
      dcmotor->run(BACKWARD);

    return false;

  } else {
    dcmotor->setSpeed(0);
    return true;
  }

}

bool checktilted()
{
  int tiltVal = digitalRead(tilt_pin);
  if (tiltVal == HIGH) {
    return true;
  } else {
    return false;
  }
}



bool checkSwitch()
{
  // read the state of the pushbutton value:
   int switchState = digitalRead(switchPin);

   //Serial.println(switchState);

  if (switchState == HIGH) {
    return true;
  } else {
    return false;
  }
}

void setTargetPosElbow(float newtargetpos)
{
  targetposelbow = newtargetpos;
}

void readcommand(int &state, int &controlvalue)
{
  memset(buffer, 0, 5);
  int readbytes = Serial.readBytes(buffer, 4);

  if (readbytes == 4) {
    if (debug) Serial.println ( (int)buffer[0] );
    int action = 0;
    if (buffer[0] >= 65)  // send alpha hexa actions.
      action = buffer[0] - 65 + 10;
    else
      action = buffer[0] - 48;
    int a = buffer[1] - 48;
    int b = buffer[2] - 48;
    int c = buffer[3] - 48;

    controlvalue = atoi(buffer + 1);
    state = action;

    if (debug) {
      Serial.print("Action:");
      Serial.print(action);
      Serial.print("/");
      Serial.println(controlvalue);
    }
  }
}


void setup() {
  Serial.begin(9600); // set up Serial library at 9600 bps
  Serial.println("Motor Unit Neuron");

  AFMS.begin(); 

  pinMode(laserPin, OUTPUT);
  pinMode(switchPin, INPUT);
  pinMode(tilt_pin, INPUT);

  shoulderEncoder.setupEncoder(12,13);
  clavicleEncoder.setupEncoder(6,5);

  wrist.servo.attach(10);
  wrist.pos=90;
  wrist.tgtPos=90;
  elbow.servo.attach(9); // connector closed to the center of the board
  elbow.pos=90;
  elbow.tgtPos=90;
}

int counter=0;

void loop() {

  if (Serial.available() > 0)
  {

    char syncbyte = Serial.read();

    switch (syncbyte)
    {
      case 'I':
        Serial.println("MTRN2");
        break;
      case 'S':
        //startburst();
        break;
      case 'X':
        //stopburst();
        break;
      case 'D':
        debug = (!debug);
        break;
      case 'L':
        digitalWrite(laserPin, HIGH);
        break;
      case 'l':
        digitalWrite(laserPin, LOW);
        break;
      case 'Q':
        Serial.println( shoulderEncoder.getEncoderPos() );
        Serial.println( clavicleEncoder.getEncoderPos() );
        break;
      case 'U':
        //Serial.print(accel.cx);Serial.print(":");Serial.print(accel.cz);Serial.print(",");Serial.print(accel.cy);Serial.print(",");Serial.print("Angle:");Serial.println(getTilt()*180.0/PI);
        break;
      case '=':
        //Serial.println(checktilted());
        if (checktilted())
          targetPosShoulder=setThisTargetPos(90-150);
        else
          targetPosShoulder=setThisTargetPos(200-150);
        tilted = checktilted();
        homing=true;
        elbow.tgtPos=90;
        wrist.tgtPos=90;
        targetPosClavicle=setThisTargetPos(220-150);
        homing2=true;
        break;
      case 'A':
        readcommand(state,controlvalue);
        break;
      default:
        break;
    }

  }  

  // Update the servo wrist/elbow position.
  wrist.update();
  elbow.update();

  // Update DC encoders (KY-090)
  shoulderEncoder.updateEncoder();
  clavicleEncoder.updateEncoder();

  if (homing && checktilted() != tilted)
  {
    // Stop there
    shoulderEncoder.resetEncoderPos();
    targetPosShoulder=0;
    homing = false;
  }

  if (checkSwitch())
  {
    clavicleEncoder.resetEncoderPos();
    targetPosClavicle = setThisTargetPos(113-150);
    state=0;
    homing2=true;
  }

  if (homing2 && targetPosClavicle == clavicleEncoder.getEncoderPos())
  {
    clavicleEncoder.resetEncoderPos();
    targetPosClavicle=0;
    homing2=false;
  }
    
  updatethisdc(clavicle, 80,    targetPosClavicle,clavicleEncoder.getEncoderPos());
  updatethisdc(shoulder, TORQUE,targetPosShoulder,shoulderEncoder.getEncoderPos());

  switch (state)
  {
    case 1:
      //grip->write(controlvalue);
      grip->setSpeed(controlvalue);
      grip->run(FORWARD);
      grippercounter = 1;
      break;
    case 2:
      //grip->write(-controlvalue);
      grip->setSpeed(controlvalue);
      grip->run(BACKWARD);
      grippercounter = 1;
      break;
    case 3:
      // Go up
      shoulder->setSpeed(controlvalue);
      shoulder->run(FORWARD);
      break;
    case 4:
      shoulder->setSpeed(controlvalue);
      shoulder->run(BACKWARD);
      break;
    case 7:
      targetPosShoulder=setThisTargetPos(controlvalue - 150);
      break;
    case 5:
      grip->run(RELEASE);
      tesaki->run(RELEASE);
      shoulder->run(RELEASE);
      clavicle->run(RELEASE);
      elbow.servo.detach();
      wrist.servo.detach();
      state = 0;
      break;
    case 6:
      // Update desired position.
      wrist.tgtPos = controlvalue;
      break;
    case 8:
      tesakicounter = 1;
      tesaki->setSpeed(controlvalue);
      tesaki->run(BACKWARD);
      break;
    case 9:
      tesakicounter = 1;
      tesaki->setSpeed(controlvalue);
      tesaki->run(FORWARD);
      break;
    case 0x0a:
      // 150x10 is no movement. 360 servo.
      //elbow.writeMicroseconds(controlvalue * 10);
      elbow.tgtPos=controlvalue;
      elbowcounter = 1;
      break;
    case 0x0b:
      setTargetPosElbow(((float)controlvalue)*PI/180.0);
      elbowcounter = 1;
    case 0x0c:
      targetPosClavicle=setThisTargetPos(controlvalue - 150);
      break;
    default:
      // Do Nothing
      state = 0;
      break;

  }

  // Limit The movement of the wrist rolling.
  tesakicounter = tesakicounter + 1;

  if (tesakicounter > 50) {
    tesaki->setSpeed(0);
    tesakicounter = 0;
  }


  // Limit the gripper force.
  grippercounter = grippercounter + 1;

  if (grippercounter > 2000) {
   grip->setSpeed(0);
   grippercounter = 0;
  }

  //delay(10);
  state = 0;  // State is reset after the command has been given and processesd.
}

// Use for testing
void lofop() {

      shoulder->setSpeed(100);
      shoulder->run(BACKWARD);

      clavicle->setSpeed(100);
      clavicle->run(BACKWARD);

      //tesaki->setSpeed(200);
      //tesaki->run(FORWARD);

    if (counter<100)
    {
      grip->setSpeed(100);
      grip->run(FORWARD);
      counter++;
    }

    shoulderEncoder.updateEncoder();
    clavicleEncoder.updateEncoder();


    //checkSwitch();
    Serial.println(counter);

    Serial.println(shoulderEncoder.getEncoderPos());
    Serial.println( clavicleEncoder.getEncoderPos() );

    digitalWrite(laserPin, HIGH);

}
