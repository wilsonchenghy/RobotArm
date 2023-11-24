// zAxis points upward

#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>//https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library

#define Pi 3.141592654

Adafruit_PWMServoDriver pca= Adafruit_PWMServoDriver(0x40);

Servo servos[3];
int servoPins[3] = {5, 6, 7};

double theta1 = 85.0; // 85 most allign
double theta2 = 85.0;
double theta3 = 85.0; 

double theta1Prev = theta1;
double theta2Prev = theta2;
double theta3Prev = theta3;

//Constant for different parts of the robotArm
float armLength = 92.5;

float iP[3] = {150, 30, 0};
float fP[3] = {150, -30, 0};





void setup() {
  servos[0].attach(servoPins[0]);
  servos[1].attach(servoPins[1]);
  servos[2].attach(servoPins[2]);
  Serial.begin(9600);

  // can use servoDriver instead
  pca.begin();
  pca.setPWMFreq(60);

  // initialise the servo to a startup position
  servos[0].write(theta1);
  servos[1].write(theta2);
  servos[2].write(theta3);

  delay(800);
}


void loop() {
  
  // MovingSmoothly1(100, 0, 0);


  // MovingSmoothly2(iP, fP);

  // iP[0] = fP[0];
  // iP[1] = fP[1];
  // iP[2] = fP[2];


  drawParamatricEquation();


  // movingStraightLineWithPotentiometer();


  // movingTheArm(100, 0, 30);


  // forwardKinematics(theta1, theta2, theta3);

}





// set the magnitude of all three angle but not yet move it
void inverseKinematics(float x, float y, float z) {
  theta3 = atan(y/x);

  float newX = sqrt(x*x + y*y);

  double theta1A = atan(z/newX);

  float l1 = sqrt(z*z + newX*newX);

  theta2 = acos((-(l1*l1 - armLength*armLength - armLength*armLength))/(2*armLength*armLength));

  double theta1B = (Pi - theta2)/2;

  theta1 = theta1A + theta1B;

  // theta1 = radToAngle(theta1);
  // theta2 = radToAngle(theta2);
  // theta3 = radToAngle(theta3);

  // Serial.print("theta1: ");
  // Serial.println(theta1);
  // Serial.print("theta2: ");
  // Serial.println(theta2);
  // Serial.print("theta3: ");
  // Serial.println(theta3);
  // Serial.println(" ");

//   delay(1000);
}



void movingTheArm(float x, float y, float z) {
  inverseKinematics(x, y, z);
  
  // the angle needs e434to be converted to degrees first 
  theta1 = radToAngle(theta1);
  theta2 = radToAngle(theta2);
  theta3 = radToAngle(theta3);

  servos[0].write(theta1 -5); // -5 to consider the servo's initial offset
  servos[1].write(180-theta2 + (90-theta1) -5);
  servos[2].write(90-theta3 -5);
}



void movingStraightLineWithPotentiometer() {
  float potx = analogRead(A0);
  float poty = analogRead(A0);
  float potz = analogRead(A0);
  
  potx = map(potx, 0, 1023, 70, 190);
  poty = map(poty, 0, 1023, -100, 100);
  potz = map(potz, 0, 1023, 0, 140);

  movingTheArm(potx, 0, 30); // when z=0 -> 140
}



void MovingSmoothly1(float x, float y, float z) {
  inverseKinematics(x, y, z);

  // the angle needs to be converted to degrees first 
  theta1 = radToAngle(theta1);
  theta2 = radToAngle(theta2);
  theta3 = radToAngle(theta3);

  smoothMoveFromAToB(theta1Prev, theta1, 0);
  smoothMoveFromAToB(180-theta2Prev + (90-theta1Prev), 180-theta2 + (90-theta1), 1);
  smoothMoveFromAToB(90-theta3Prev, 90-theta3, 2);

  theta1Prev = theta1;
  theta2Prev = theta2;
  theta3Prev = theta3;
}


void MovingSmoothly2(float initialPosition[3], float finalPosition[3]) {

  float x1 = initialPosition[0], y1 = initialPosition[1], z1 = initialPosition[2];
  float x2 = finalPosition[0], y2 = finalPosition[1], z2 = finalPosition[2];

  movingTheArm(x1, y1, z1); // set the postion to the initialPosition first
  delay(1000);

  float ratioA = 0.0;
  float steps = 0.0;

  // float distance = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1));
  float displacementVector[] = {(x2-x1), (y2-y1), (z2-z1)};
  // float vectorFromAToB[] = {0.0, 0.0, 0.0}; 

  while(steps < (1.0 - 0.01)) {
    steps = 0.97*ratioA + 0.03*1;
    float vectorFromAToB[] = {(ratioA*displacementVector[0] + x1) ,(ratioA*displacementVector[1] + y1), (ratioA*displacementVector[2] + z1)}; // vector ratio theorem
    movingTheArm(vectorFromAToB[0], vectorFromAToB[1], vectorFromAToB[2]);
    Serial.println(vectorFromAToB[0]);
    ratioA = steps;
    delay(20);
    // if (ratioA > 0.5 - 0.01) { // bascially the ideas is for when a=0.5
    //   delay(10000); // pause at the midpoint, for finding out where is the midpoint
    // }
  }
}

// three functions need to be coded
void MovingSmoothlyWithPotentiometer(float initialPosition[3], float finalPosition[3]) {

  float x1 = initialPosition[0], y1 = initialPosition[1], z1 = initialPosition[2];
  float x2 = finalPosition[0], y2 = finalPosition[1], z2 = finalPosition[2];

  movingTheArm(x1, y1, z1); // set the postion to the initialPosition first
  delay(1000);

  float ratioA = 0.0;
  float steps = 0.0;

  // float distance = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1));
  float displacementVector[] = {(x2-x1), (y2-y1), (z2-z1)};
  // float vectorFromAToB[] = {0.0, 0.0, 0.0}; 

  while(steps < (1.0 - 0.01)) {
    steps = 0.97*ratioA + 0.03*1;
    float vectorFromAToB[] = {(ratioA*displacementVector[0] + x1) ,(ratioA*displacementVector[1] + y1), (ratioA*displacementVector[2] + z1)}; // vector ratio theorem
    movingTheArm(vectorFromAToB[0], vectorFromAToB[1], vectorFromAToB[2]);
    Serial.println(vectorFromAToB[0]);
    ratioA = steps;
    delay(20);
    // if (ratioA > 0.5 - 0.01) { // bascially the ideas is for when a=0.5
    //   delay(10000); // pause at the midpoint, for finding out where is the midpoint
    // }
  }
}


void Moving_In_a_Line(float initialPosition[3], float finalPosition[3]) {

  float x1 = initialPosition[0], y1 = initialPosition[1], z1 = initialPosition[2];
  float x2 = finalPosition[0], y2 = finalPosition[1], z2 = finalPosition[2];

  movingTheArm(x1, y1, z1); // set the postion to the initialPosition first
  delay(1000);

  float ratioA = 0.0;
  float steps = 0.0;

  // float distance = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1));
  float displacementVector[] = {(x2-x1), (y2-y1), (z2-z1)};
  // float vectorFromAToB[] = {0.0, 0.0, 0.0}; 

  while(steps < (1.0 - 0.01)) {
    steps = 0.97*ratioA + 0.03*1;
    float vectorFromAToB[] = {(ratioA*displacementVector[0] + x1) ,(ratioA*displacementVector[1] + y1), (ratioA*displacementVector[2] + z1)}; // vector ratio theorem
    movingTheArm(vectorFromAToB[0], vectorFromAToB[1], vectorFromAToB[2]);
    Serial.println(vectorFromAToB[0]);
    ratioA = steps;
    delay(20);
    // if (ratioA > 0.5 - 0.01) { // bascially the ideas is for when a=0.5
    //   delay(10000); // pause at the midpoint, for finding out where is the midpoint
    // }
  }
}


void Moving_In_a_Line_WithPotentiometer(float initialPosition[3], float finalPosition[3]) {

  float x1 = initialPosition[0], y1 = initialPosition[1], z1 = initialPosition[2];
  float x2 = finalPosition[0], y2 = finalPosition[1], z2 = finalPosition[2];

  movingTheArm(x1, y1, z1); // set the postion to the initialPosition first
  delay(1000);

  float ratioA = 0.0;
  float steps = 0.0;

  // float distance = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1));
  float displacementVector[] = {(x2-x1), (y2-y1), (z2-z1)};
  // float vectorFromAToB[] = {0.0, 0.0, 0.0}; 

  while(steps < (1.0 - 0.01)) {
    steps = 0.97*ratioA + 0.03*1;
    float vectorFromAToB[] = {(ratioA*displacementVector[0] + x1) ,(ratioA*displacementVector[1] + y1), (ratioA*displacementVector[2] + z1)}; // vector ratio theorem
    movingTheArm(vectorFromAToB[0], vectorFromAToB[1], vectorFromAToB[2]);
    Serial.println(vectorFromAToB[0]);
    ratioA = steps;
    delay(20);
    // if (ratioA > 0.5 - 0.01) { // bascially the ideas is for when a=0.5
    //   delay(10000); // pause at the midpoint, for finding out where is the midpoint
    // }
  }
}



bool init1 = true;

void drawParamatricEquation() {
  float startingPoint[3] = {100, 0, 10}; // randomly start at a point

  if (init1) { // only run once when first initislise
    movingTheArm(startingPoint[0], startingPoint[1], startingPoint[2]); 
    delay(1000);
  }
  double t = analogRead(A0); // parameter of a paramatric equation
  // t = map(t, 0, 1023, 0, 50); // for y=x or y=-x
  // t = map(t, 0, 1023, -18, 18); // for y=(x*x)/12
  // t = map(t, 0, 1023, 0, 20*Pi); // for y*y + x*x = 40*40
  t = map(t, 0, 1023, 2, 14); // for heart

  // movingTheArm(startingPoint[0] + t, 0, startingPoint[2] + t); // equation of y=x
  // movingTheArm(startingPoint[0] + t, 0, startingPoint[2] - t); // equation of y=-x
  // movingTheArm(startingPoint[0] + 2*t, 0, startingPoint[2] + (t*t)/3); // equation of y=(x*x)/12
  // movingTheArm(startingPoint[0] + 40*cos(t/(2*Pi)), 0, startingPoint[2] + 40*sin(t/(2*Pi))); // equation of y*y + x*x = 40*40
  movingTheArm(startingPoint[0] + 4*16*sin(t)*sin(t)*sin(t), 0, startingPoint[2] + 4*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t))); // equation of a heart


  init1 = false;
}



// just mainly for checking if the IK is correct
// accept angle as degree and in the function convert to radians
void forwardKinematics(double angle1, double angle2, double angle3) {
  angle1 = angleToRad(angle1);
  angle2 = angleToRad(angle2);
  angle3 = angleToRad(angle3);

  double angle2Corr = angle2 - Pi + angle1;

  float armX = cos(angle1)*armLength + cos(angle2Corr)*armLength;
  float forwardKinematicsX = armX*cos(angle3);
  float forwardKinematicsZ = sin(angle1)*armLength + sin(angle2Corr)*armLength;
  float forwardKinematicsY = forwardKinematicsX*tan(angle3);

  Serial.print("X: ");
  Serial.println(forwardKinematicsX);
  Serial.print("Y: ");
  Serial.println(forwardKinematicsY);
  Serial.print("Z: ");
  Serial.println(forwardKinematicsZ);
  Serial.println(" ");

  delay(1000);
}



// rad to angle/angle to rad functions
double radToAngle(double angle) {
  return (angle*180)/Pi;
}
double angleToRad(double angle) {
  return (angle/180)*Pi;
}



// constant for microsecond conversion
int zeroDegree = 544;
int oneEightyDegree = 2400;
int angleValue = 90; // servo start at 90 when first initilise

float angleSmoothed = 0;

void smoothMoveFromAToB(float initialAngle, float finalAngle, int servoID) {
  // convert to microsecond 
  initialAngle = map(initialAngle, 0, 180, 544, 2400);
  finalAngle = map(finalAngle, 0, 180, 544, 2400);
  
  if (finalAngle >= initialAngle) {
    while (initialAngle < (finalAngle-5)) { // -5 is a offset for terminating the while loop, or else, it may infinitely appraoch the number and don't exit the loop
      angleSmoothed = (0.97*initialAngle) + (0.03*finalAngle); // in microosecond
      // Serial.println(angleSmoothed);
      servos[servoID].writeMicroseconds(angleSmoothed);  
      initialAngle = angleSmoothed;
      delay(5);
    }
  } else {
    while (finalAngle < (initialAngle-5)) {  // -5 is a offset for terminating the while loop, or else, it may infinitely appraoch the number and don't exit the loop
      angleSmoothed = (0.97*initialAngle) + (0.03*finalAngle); // in microosecond
      // Serial.println(angleSmoothed);
      servos[servoID].writeMicroseconds(angleSmoothed);  
      initialAngle = angleSmoothed;
      delay(5);
    }
  }
}




