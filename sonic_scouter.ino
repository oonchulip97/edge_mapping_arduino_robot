/*
 * Project: Sonic Scouter.
 * Objective: Map out the boundaries of a surrounding.
 * Last updated: 22nd August 2017 by Oon Chu Lip.
 */

#include <EEPROM.h>

const int trig1 = 8; // pin number for sensor 1 trigger pin
const int echo1 = 9; // pin number for sensor 1 echo pin
const int trig2 = 10; // pin number for sensor 2 trigger pin
const int echo2 = 11; // pin number for sensor 2 echo pin
const int trig3 = 12; // pin number for sensor 3 trigger pin
const int echo3 = 13; // pin number for sensor 3 echo pin

long duration1; // time for ultrasound to bounce back to sensor 1
int distance1; // distance to obstacle on the port side in centimeters
long duration2; // time for ultrasound to bounce back to sensor 2
int distance2; // distance to obstacle on the front in centimeters
long duration3; // time for ultrasound to bounce back to sensor 3
int distance3; // distance to obstacle on the starboard side in centimeters

int lastdistance1 = 10; // initialise variables to prevent timeout and prevent volatility
int lastdistance2 = 10;
int lastdistance3 = 10;

const int optimumdistance = 10; // distance to move along the wall
const int kp = 4; // constant for the proportional controller
const int ki = 0.08; // constant for the integral controller
const int kd = 50; // constant for the differential controller
int error, turn; // initialise variables for PID control
int speedA, speedB;
int integral = 0;
int lasterror = 0;
int derivative = 0;

const int dirA = 4; // pin number for motor A direction control
const int pwmA = 5; // pin number for motor A pwm control
const int dirB = 7; // pin number for motor B direction control
const int pwmB = 6; // pin number for motor B pwm control
int speedAB = 188; // initial speed for motor to move between 0 to 255
int delaytime = 3000; // time of delay in milliseconds

const int interruptA = 2;  // pin number for encoder A interrupt pin
const int interruptB = 3; // pin number for encoder B interrupt pin

long coder[2] = {0, 0}; // an array to hold wheel speed of left and right wheel encoders respectively for pid control

const byte rows = 30; // number of rows for the map array
const byte columns = 30; // number of columns for the map array
byte maplayout[rows][columns] = {}; // an array to represent the layout of the map which '0' represents empty spaces and '1' represents obstacles

const int buttonPin = 19; // pin number for button
int flag = 1; // initialises a placeholder for the logic of the button

long lastDebounceTime = 0; // the last time button was pressed in milliseconds
long debounceDelay = 1000; // time to ignore input from button in milliseconds

long mapcoder[2] = {0, 0}; // an array to hold wheel speed of left and right wheel encoders respectively for mapping purpose

byte currentxaxis = 14; // current position in the array which represent layout of the map
byte currentyaxis = 14;

char currentdirection = 'N'; // current direction for the movement of the car during mapping

int mapturnleftcounter = 0; // counter for mapping when turning left

long lastUpdateTime = 0; // the last time map is being updated into EEPROM
long updateDelay = 5000; // time interval between updating map into EEPROM in milliseconds

long lastResetTime = 100000; // the last time the variable mapturnleftcounter is being resetted
long resetDelay = 5000; // time interval between resetting mapturnleftcounter

// the setup function runs once when you press reset or power the board
void setup() {

  // setups sensor 1, 2 and 3
  pinMode(trig1, OUTPUT); // sets the sensor 1 trigger pin as an output
  pinMode(echo1, INPUT); // sets the sensor 1 echo pin as an input
  pinMode(trig2, OUTPUT); // sets the sensor 2 trigger pin as an output
  pinMode(echo2, INPUT); // sets the sensor 2 echo pin as an input
  pinMode(trig3, OUTPUT); // sets the sensor 3 trigger pin as an output
  pinMode(echo3, INPUT); // sets the sensor 3 echo pin as an input

  // setups motor A and B
  pinMode(dirA, OUTPUT); // initiates motor A direction control pin
  pinMode(dirB, OUTPUT); // initiates motor B direction control pin

  // setups interrupt mode of encoder A and B
  attachInterrupt(digitalPinToInterrupt(interruptA), leftwheelspeed, CHANGE); // initiates encoder A interrupt mode
  attachInterrupt(digitalPinToInterrupt(interruptB), rightwheelspeed, CHANGE); // initiates encoder B interrupt mode

  // enable internal pull-up resistor on the button pin
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop()
{
  if (digitalRead(buttonPin) == LOW && (millis() - lastDebounceTime > debounceDelay)) { // check button press here and change flagger
    if (flag == 0) {
      flag = 1;
    }
    else if (flag == 1) {
      flag = 2;
    }
    else if (flag == 2) {
      flag = 0;
    }
    lastDebounceTime = millis();
  }

  if (flag == 0) { // mapping begins and update map into EEPROM every 1 second

    preventtimeout();
    preventvolatility();

    coder[0] = 0;
    coder[1] = 0;

    if (distance2 < 20) // extra 10cm for response time
    {
      while (coder[0] <= 12) // suitable trigger for perpendicular rotation
      {
        Serial.print(" Left Wheel Spin For Turning Right: ");
        Serial.println(coder[0]);
        turnright(188, 188);
      }
      turnleft(188, 188); // remove inertial movement
      integral = 0; // reset pid controller
      lasterror = 0;
      mapturnright();
      mapturnleftcounter = 0;
    }

    {
      if (distance1 > 35)
      {
        while (coder[0] <= 5) // suitable trigger for perpendicular rotation
        {
          Serial.print(" Left Wheel Spin For Turning Left: ");
          Serial.println(coder[0]);
          moveforward(121, 250);
        }
        movebackward(121, 250); // remove inertial movement
        integral = 0; // reset pid controller
        lasterror = 0;
        mapturnleft();
      }
      else
      {
        movealongwall(distance1);
        mapturnleftcounter = 0;
      }
    }

    mapping();

    if (millis() - lastUpdateTime > updateDelay) {

      Serial.println("MAPPING");

      for (byte y = 0; y < rows; y++) { // place map into EEPROM
        for (byte x = 0; x < columns;  x++) {
          EEPROM.update((y * columns) + x, maplayout[y][x]);
        }
      }

      lastUpdateTime = millis();
    } else {
      return;
    }

  }

  else if (flag == 1) { // read map from EEPROM via serial communication

    stopmoving(); // stop moving

    Serial.println("The following map is retrieve from EEPROM: "); // print map from EEPROM
    for (byte y = 0; y < rows; y++) { // loops through array's rows
      for (byte x = 0; x < columns; x++) { // loops through columns of current row
        Serial.print(EEPROM.read((y * columns) + x));
        Serial.print(" ");
      }
      Serial.println(); // starts a new line for new row
    }
    Serial.println("* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * "); // ends array

    delay(200);

  }

  else if (flag == 2) { // initialises the map and reset EEPROM

    Serial.println("Map is initialised and EEPROM resetted.");

    // initialises array to represent layout of the map
    for (byte y = 0; y < rows; y++) { // loop through array's rows
      for (byte x = 0; x < columns;  x++) { // loop through columns of current row
        maplayout[y][x] = 0;
      }
    }

    for (byte y = 0; y < rows; y++) { // reset map in EEPROM
      for (byte x = 0; x < columns;  x++) {
        EEPROM.update((y * columns) + x, maplayout[y][x]);
      }
    }

    delay(200);

  }
}

// sensors check their respective distance
int checkdistance() {

  digitalWrite(trig1, LOW); // clears all the sensors trigger pin
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH); // sets each of the trigger pin on HIGH state to emit ultrasound for 10 microseconds only
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  duration1 = pulseIn(echo1, HIGH); // reads the echoPin, returns the sound wave travel time in microseconds
  distance1 = duration1 * 0.034 / 2; // calculate the respective distance which speed of sound is 0.034 centimeters per microseconds
  Serial.print("distance1: ");
  Serial.print(distance1);

  digitalWrite(trig2, LOW); // repeated for sensor 2
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);
  duration2 = pulseIn(echo2, HIGH);
  distance2 = duration2 * 0.034 / 2;
  Serial.print(" distance2: ");
  Serial.print(distance2);

  digitalWrite(trig3, LOW); // repeated for sensor 3
  delayMicroseconds(2);
  digitalWrite(trig3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig3, LOW);
  duration3 = pulseIn(echo3, HIGH);
  distance3 = duration3 * 0.034 / 2;
  Serial.print(" distance3: ");
  Serial.println(distance3);

  return distance1, distance2, distance3;
}

// make sure the car go straight from the wall with a certain distance
void movealongwall (int distance1) {
  error = distance1 - optimumdistance;
  if (error < 1 && error > -1) {
    integral = 0;
  } else {
    integral = integral + error;
  }
  derivative = error - lasterror;
  turn = kp * error + ki * integral + kd * derivative;
  turn = min(turn, 67);
  speedA = speedAB - turn;
  speedB = speedAB + turn;
  if (speedA < 121) {
    turnleft(0, 255);
    delay(10);
  } else if (speedB < 121) {
    turnright(255, 0);
    delay(10);
  } else {
    moveforward(speedA, speedB);
  }
  lasterror = error;
}


// ignores results due to timing-out of the sensors
int preventtimeout() {

  checkdistance();

  if (distance1 > 3400 || distance2 > 3400 || distance3 > 3400) {
    Serial.println("TIMEOUT");
    preventtimeout();
  }
  else {
    return distance1, distance2, distance3; // returns result if there is no time-out from sensors
  }

}

//ignores results from being too volatile
int preventvolatility() {

  checkdistance();

  if (abs(distance1 - lastdistance1) > 100 || abs(distance2 - lastdistance2) > 100 || abs(distance3 - lastdistance3) > 100) {
    lastdistance1 = distance1; // updating last distance before sensors recheck distance
    lastdistance2 = distance2;
    lastdistance3 = distance3;
    Serial.println("VOLATILE");
    preventvolatility();
  }
  else {
    return distance1, distance2, distance3; // returns result if result is not too volatile
  }
  
}

// car moves forward
void moveforward(int speedA, int speedB) {
  digitalWrite(dirA, HIGH); // establishes forward direction of motor A
  digitalWrite(dirB, HIGH); // establishes forward direction of motor B
  analogWrite(pwmA, speedA);   // spins motor A
  analogWrite(pwmB, speedB);   // spins motor B
}

// car moves backward
void movebackward(int speedA, int speedB) {
  digitalWrite(dirA, LOW); // establishes backward direction of motor A
  digitalWrite(dirB, LOW); // establishes backward direction of motor B
  analogWrite(pwmA, speedA);   // spins motor A
  analogWrite(pwmB, speedB); // spins motor B
}


// car stops
void stopmoving() {
  digitalWrite(dirA, HIGH); // establishes backward direction of motor A
  digitalWrite(dirB, HIGH); // establishes backward direction of motor B
  analogWrite(pwmA, 0);   // spins motor A
  analogWrite(pwmB, 0);   // spins motor B
}

// car turns left perpendicularly
void turnleft(int speedA, int speedB) {
  digitalWrite(dirA, LOW); // establishes backward direction of motor A
  digitalWrite(dirB, HIGH); // establishes forward direction of motor B
  analogWrite(pwmA, speedA);   // spins motor A
  analogWrite(pwmB, speedB);   // spins motor B
}

// car turns right perpendicularly
void turnright(int speedA, int speedB) {
  digitalWrite(dirA, HIGH); // establishes forward direction of motor A
  digitalWrite(dirB, LOW); // establishes backward direction of motor B
  analogWrite(pwmA, speedA);   // spins motor A
  analogWrite(pwmB, speedB);   // spins motor B
}

// counts encoder A interrupts
void leftwheelspeed() {
  coder[0] += 1; // increment left wheel speed by one
  mapcoder[0] += 1;
}

// counts encoder B interrupts
void rightwheelspeed() {
  coder[1] += 1; // increment right wheel speed by one
  mapcoder[1] += 1;
}

// outputs array to represent the layout of the map
void printArray(byte a[rows][columns]) {
  for (byte y = 0; y < rows; y++) { // loops through array's rows
    for (byte x = 0; x < columns; x++) { // loops through columns of current row
      Serial.print(a[y][x]);
      Serial.print(" ");
    }
    Serial.println(); // starts a new line for new row
  }
  Serial.println("* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * "); // ends array
}

// map according to the direction the car is moving
void mapping () {
  if (currentdirection == 'N') {
    towardsNorth();
  }
  else if (currentdirection == 'E' ) {
    towardsEast();
  }
  else if (currentdirection == 'W' ) {
    towardsWest();
  }
  else if (currentdirection == 'S' ) {
    towardsSouth();
  }
  Serial.println(currentdirection);
}

// map when car is moving towards North
void towardsNorth() {
  if (mapcoder[0] >= 10 && mapcoder[1] >= 10)
  {
    maplayout[currentyaxis][currentxaxis] = 1;
    if (currentyaxis > 0) {
      currentyaxis -= 1;
    }
    mapcoder[0] = 0; // reset encoder values for mapping purposes
    mapcoder[1] = 0;
  }
}

// map when car is moving towards East
void towardsEast() {
  if (mapcoder[0] >= 10 && mapcoder[1] >= 10)
  {
    maplayout[currentyaxis][currentxaxis] = 1;
    if (currentxaxis < columns) {
      currentxaxis += 1;
    }
    mapcoder[0] = 0; // reset encoder values for mapping purposes
    mapcoder[1] = 0;
  }
}

// map when car is moving towards West
void towardsWest() {
  if (mapcoder[0] >= 10 && mapcoder[1] >= 10)
  {
    maplayout[currentyaxis][currentxaxis] = 1;
    if (currentxaxis > 0) {
      currentxaxis -= 1;
    }
    mapcoder[0] = 0; // reset encoder values for mapping purposes
    mapcoder[1] = 0;
  }
}

// map when car is moving towards South
void towardsSouth() {
  if (mapcoder[0] >= 10 && mapcoder[1] >= 10)
  {
    maplayout[currentyaxis][currentxaxis] = 1;
    if (currentyaxis < rows) {
      currentyaxis += 1;
    }
    mapcoder[0] = 0; // reset encoder values for mapping purposes
    mapcoder[1] = 0;
  }
}

// changes direction the car is moving in map when it turns right
void mapturnright() {
  if (currentdirection == 'N') {
    currentdirection = 'E';
  }
  else if (currentdirection == 'E') {
    currentdirection = 'S';
  }
  else if (currentdirection == 'S') {
    currentdirection = 'W';
  }
  else if (currentdirection == 'W' ) {
    currentdirection = 'N';
  }
  mapcoder [0] = 0;
  mapcoder [1] = 0;
}

// changes direction the car is moving in map when it turns left
void mapturnleft() {
  if (mapturnleftcounter >= 2) { // initiates direction change after a perpendicular turn to left
    if (currentdirection == 'N') {
      currentdirection = 'W';
    }
    else if (currentdirection == 'E') {
      currentdirection = 'N';
    }
    else if (currentdirection == 'S') {
      currentdirection = 'E';
    }
    else if (currentdirection == 'W') {
      currentdirection = 'S';
    }
    mapturnleftcounter = 0; //reset counter
    mapcoder[0] = 0;
    mapcoder[1] = 0;
  }
  mapturnleftcounter += 1;
}

