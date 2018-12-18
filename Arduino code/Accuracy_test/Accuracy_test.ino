/*
  J.Teda 21/04/2013

  Exsample of how to run Dynamixel in SERVO mode - tell servo what angle to move to.

  Robotis e-Manual ( http://support.robotis.com )

*/


/*
  Improvements:
  - finish x, y, z (make public variables private again by adding a new public function in the robotarm class to move along x, y, z)
    - make sure safety works
*/

#include <Dynamixel_Serial.h>       // Library needed to control Dynamixal servo
#include <MatrixMath.h>
#include <math.h>
#include <PS2X_lib.h>  //for v1.6

//PS2 data
PS2X ps2x; // create PS2 Controller Class
int error = 0;
byte type = 0;
byte vibrate = 0;

//Dynamixel data
#define SERVO_ID1 1               // ID of which we will set Dynamixel to
#define SERVO_ID2 2               // ID of which we will set Dynamixel to
#define SERVO_ID3 3               // ID of which we will set Dynamixel to
#define SERVO_ID4 4               // ID of which we will set Dynamixel to
#define SERVO_ID5 5               // ID of which we will set Dynamixel to
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

const int potPin = A2;    // select the input pin for the potentiometer

#define PI 3.1415926535897932384626433832795

//DH-parameters
float alpha[3] = {0, PI / 2, PI};
float a[4] = {0, 0, 22.0, 30.50};
float d[3] = {6.7, 0, 0};

//define transformation matrixes
#define N  (4)
mtx_type T01[N][N];
mtx_type T12[N][N];
mtx_type T02[N][N];
mtx_type T23[N][N];
mtx_type T03[N][N];
mtx_type T3EE[N][N];
mtx_type T0EE[N][N];
mtx_type P1EE[N];      // This is a row vector
mtx_type P0EE[N];      // This is a row vector


class robotarm {
  private:
    unsigned int current_pos_arr[3]; //array to store current servo step positions
    void read_joint_positions(); //reads the current position
    bool not_runable(bool moving); //check if robot is in a dangerous position
    int calc_pos_arr[3]; //array to store next calculated servo step positions used for trajectory

  public:
    // robotarm(); //possible constructor
    int MoveJ(unsigned int end_theta_1, unsigned int end_theta_2, unsigned int end_theta_3, float tf); //function that moves joints to desired position (input in steps between 0 and 4095)
    int MoveL(float end_x, float end_y, float end_z, float tf); //function that moves linearly to a desired position in cartesian space (coordinates in cm)
    void calc_forward(); //function for forward kinematics (calculates the current position of end-effector based on current joint angles)
    void calc_inverse(); //function for inverse kinematics
    void ChangeGripper(bool state);
    int move_along_XYZ(); //function to move the end-effecter along x-, y- and z-axis in cartesian space
    bool block_servo2_down = false; //variable to block the second servo if it goes below 0.


};

robotarm CrustCrawler; //create the object 'CrustCrawler' of the class 'robotarm'

void robotarm::ChangeGripper(bool state) {
  int servo4;
  int servo5;
  /*
        int potVal = 4.002932551319648*analogRead(potPin);    // read the value from the potentiometer and convert it to servo steps

        servo4 = 1992+(potVal/4); //from 1992 (175 degrees) to 3016 (~270 degrees)
        servo5 = 2084-(potVal/4); //from 2084 (183 degrees) to 1060 (~90 degrees)

        Serial.print("servo4 = ");
        Serial.println(servo4);
        Serial.print("servo5 = ");
        Serial.println(servo5);
  */

  if (state) { //open gripper
    servo4 = 2262; //(~270 degrees)
    servo5 = 1814; //(~90 degrees)
  }

  else { //close gripper
    //servo4 = 2120; //(completely closed: 175 degrees/1992 steps)
    //servo5 = 1958; //(completely closed: 183 degrees/2084 steps)

    //closed completely
    //servo4 = 1992; //(completely closed: 175 degrees/1992 steps)
    //servo5 = 2084; //(completely closed: 183 degrees/2084 steps)

    //closed for tusch
    servo4 = 2032; //(completely closed: 175 degrees/1992 steps)
    servo5 = 2064; //(completely closed: 183 degrees/2084 steps)
  }

  Dynamixel.servo(SERVO_ID4, servo4, 0x40); //  Move servo
  delay(5);
  Dynamixel.servo(SERVO_ID5, servo5, 0x40); //  Move servo
}

int robotarm::MoveJ(unsigned int end_theta_1, unsigned int end_theta_2, unsigned int end_theta_3, float tf) { //desired position is passed to the function

  //check if the current joint angles allow the MoveJ function to run
  if (not_runable(false)) return 0;

  read_joint_positions();    //read current position

  //joint 1
  int theta1_f = end_theta_1; //180 degrees (2048 for home)
  int a1_0 = current_pos_arr[0]; //start position
  //int a1_1 = 0; //start velocity
  float a1_2 = 3 / (tf * tf) * (theta1_f - a1_0);
  float a1_3 = -2 / (tf * tf * tf) * (theta1_f - a1_0);

  //joint 2
  int theta2_f = end_theta_2; //90 degrees (1024 for home)
  int a2_0 = current_pos_arr[1]; //start position
  //int a2_1 = 0; //start velocity
  float a2_2 = 3 / (tf * tf) * (theta2_f - a2_0);
  float a2_3 = -2 / (tf * tf * tf) * (theta2_f - a2_0);

  //joint 3
  int theta3_f = end_theta_3; //90 degrees (1024 for home)
  int a3_0 = current_pos_arr[2]; //start position
  //int a3_1 = 0; //start velocity
  float a3_2 = 3 / (tf * tf) * (theta3_f - a3_0);
  float a3_3 = -2 / (tf * tf * tf) * (theta3_f - a3_0);

  //move joints every 1/66 second
  for (float t = 0.025; t <= tf; t += 0.025) {
    calc_pos_arr[0] = a1_0 + a1_2 * (t * t) + a1_3 * (t * t * t);
    calc_pos_arr[1] = a2_0 + a2_2 * (t * t) + a2_3 * (t * t * t);
    calc_pos_arr[2] = a3_0 + a3_2 * (t * t) + a3_3 * (t * t * t);

    if (not_runable(true)) return 0; //check if the robot goes to a "dangerous position" before we move it

    Dynamixel.servo(SERVO_ID1, calc_pos_arr[0], 0x40); //  Move servo to max angle at max speed (1023)
    delay(4);
    Dynamixel.servo(SERVO_ID2, calc_pos_arr[1], 0x40); //  Move servo to max angle at max speed (1023)
    Dynamixel.servo(SERVO_ID3, calc_pos_arr[2], 0x40); //  Move servo to max angle at max speed (1023)

    delay(6);
  }
  Serial.println("Position reached");

  calc_forward();

  return 0;
}

void robotarm::calc_forward() { //function for forward kinematics

  read_joint_positions();    //read current position

  float thet1 = (current_pos_arr[0] * 0.0879120879120879 * PI) / 180; //position in radians
  float thet2 = (current_pos_arr[1] * 0.0879120879120879 * PI) / 180; //position in radians
  float thet3 = (current_pos_arr[2] * 0.0879120879120879 * PI) / 180; //position in radians

  //define matrixes
  //row 1
  T01[0][0] = cos(thet1);
  T01[0][1] = -sin(thet1);
  T01[0][2] = 0;
  T01[0][3] = a[0];

  //row 2
  T01[1][0] = sin(thet1) * cos(alpha[0]);
  T01[1][1] = cos(thet1) * cos(alpha[0]);
  T01[1][2] = -sin(alpha[0]);
  T01[1][3] = -sin(alpha[0]) * d[0];

  //row 3
  T01[2][0] = sin(thet1) * sin(alpha[0]);
  T01[2][1] = cos(thet1) * sin(alpha[0]);
  T01[2][2] = cos(alpha[0]);
  T01[2][3] = cos(alpha[0]) * d[0];

  //row 4
  T01[3][0] = 0;
  T01[3][1] = 0;
  T01[3][2] = 0;
  T01[3][3] = 1;

  //------------------------------------------------------

  //row 1
  T12[0][0] = cos(thet2);
  T12[0][1] = -sin(thet2);
  T12[0][2] = 0;
  T12[0][3] = a[1];

  //row 2
  T12[1][0] = sin(thet2) * cos(alpha[1]);
  T12[1][1] = cos(thet2) * cos(alpha[1]);
  T12[1][2] = -sin(alpha[1]);
  T12[1][3] = -sin(alpha[1]) * d[1];

  //row 3
  T12[2][0] = sin(thet2) * sin(alpha[1]);
  T12[2][1] = cos(thet2) * sin(alpha[1]);
  T12[2][2] = cos(alpha[1]);
  T12[2][3] = cos(alpha[1]) * d[1];

  //row 4
  T12[3][0] = 0;
  T12[3][1] = 0;
  T12[3][2] = 0;
  T12[3][3] = 1;


  //------------------------------------------------------

  //row 1
  T23[0][0] = cos(thet3);
  T23[0][1] = -sin(thet3);
  T23[0][2] = 0;
  T23[0][3] = a[2];

  //row 2
  T23[1][0] = sin(thet3) * cos(alpha[2]);
  T23[1][1] = cos(thet3) * cos(alpha[2]);
  T23[1][2] = -sin(alpha[2]);
  T23[1][3] = -sin(alpha[2]) * d[2];

  //row 3
  T23[2][0] = sin(thet3) * sin(alpha[2]);
  T23[2][1] = cos(thet3) * sin(alpha[2]);
  T23[2][2] = cos(alpha[2]);
  T23[2][3] = cos(alpha[2]) * d[2];

  //row 4
  T23[3][0] = 0;
  T23[3][1] = 0;
  T23[3][2] = 0;
  T23[3][3] = 1;


  //------------------------------------------------------

  //row 1
  T3EE[0][0] = 1;
  T3EE[0][1] = 0;
  T3EE[0][2] = 0;
  T3EE[0][3] = a[3];

  //row 2
  T3EE[1][0] = 0;
  T3EE[1][1] = 1;
  T3EE[1][2] = 0;
  T3EE[1][3] = 0;

  //row 3
  T3EE[2][0] = 0;
  T3EE[2][1] = 0;
  T3EE[2][2] = 1;
  T3EE[2][3] = 0;

  //row 4
  T3EE[3][0] = 0;
  T3EE[3][1] = 0;
  T3EE[3][2] = 0;
  T3EE[3][3] = 1;

  //find T02
  Matrix.Multiply((mtx_type*)T01, (mtx_type*)T12, N, N, N, (mtx_type*)T02);

  //find T03
  Matrix.Multiply((mtx_type*)T02, (mtx_type*)T23, N, N, N, (mtx_type*)T03);

  //find T0EE
  Matrix.Multiply((mtx_type*)T03, (mtx_type*)T3EE, N, N, N, (mtx_type*)T0EE);

  //add last column of T0EE to a vector
  P0EE[0] = T0EE[0][3];
  P0EE[1] = T0EE[1][3];
  P0EE[2] = T0EE[2][3];
  P0EE[3] = 1;

  //Matrix.Print((mtx_type*)T0EE, N, N, "T0EE");
  Matrix.Print((mtx_type*)P0EE, N, 1, "P0EE");

  Serial.println("Steps:");
  Serial.println(current_pos_arr[0]);
  Serial.println(current_pos_arr[1]);
  Serial.println(current_pos_arr[2]);
  Serial.println(" ");
}

void robotarm::calc_inverse() { //function for inverse kinematics
  //calculate theta 1:
  float joint1 = atan2 (P0EE[1], P0EE[0]);
  if (joint1 < 0) { //the joint value is not allowed to go below 0
    joint1 = 2 * PI + joint1;
  }

  //calculate T01
  //row 1
  T01[0][0] = cos(joint1);
  T01[0][1] = -sin(joint1);
  T01[0][2] = 0;
  T01[0][3] = a[0];

  //row 2
  T01[1][0] = sin(joint1) * cos(alpha[0]);
  T01[1][1] = cos(joint1) * cos(alpha[0]);
  T01[1][2] = -sin(alpha[0]);
  T01[1][3] = -sin(alpha[0]) * d[0];

  //row 3
  T01[2][0] = sin(joint1) * sin(alpha[0]);
  T01[2][1] = cos(joint1) * sin(alpha[0]);
  T01[2][2] = cos(alpha[0]);
  T01[2][3] = cos(alpha[0]) * d[0];

  //row 4
  T01[3][0] = 0;
  T01[3][1] = 0;
  T01[3][2] = 0;
  T01[3][3] = 1;

  Matrix.Invert((mtx_type*)T01, N);

  //find P1EE
  Matrix.Multiply((mtx_type*)T01, (mtx_type*)P0EE, N, N, 1, (mtx_type*)P1EE);

  float joint3 = acos ((P1EE[0] * P1EE[0] + P1EE[2] * P1EE[2] - a[2] * a[2] - a[3] * a[3]) / fabs(2 * a[2] * a[3]));
  float joint2 = atan2 (P1EE[2], P1EE[0]) + asin((sin(PI - joint3) * fabs(a[3])) / sqrt(P1EE[2] * P1EE[2] + P1EE[0] * P1EE[0]));

  //convert to steps and store servo angles in an array
  calc_pos_arr[0] = (joint1 * 180) / (PI * 0.0879120879120879);
  calc_pos_arr[1] = {(joint2 * 180) / (PI * 0.0879120879120879)};
  calc_pos_arr[2] = {(joint3 * 180) / (PI * 0.0879120879120879)};
}

bool robotarm::not_runable(bool moving) { //check if robot is in a dangerous position. If the passed bool is 'true' it means the robot is moving. If 'false' it is at rest
  if (moving) { //Confirm future position
    if ((1289 < calc_pos_arr[2] && calc_pos_arr[2] < 4096) || (2200 < calc_pos_arr[1] && calc_pos_arr[1] < 4096) || (3895 < calc_pos_arr[0] || calc_pos_arr[0] < 200)) {
      Serial.println("Illegal movement");
      block_servo2_down = false; //the second joint is allowed to go in any direction
      return true;
    }

    else if (5 > calc_pos_arr[1]) {
      Serial.println("Illegal movement");
      block_servo2_down = true; //the second joint is not allowed to go any further in negative direction
      return true;
    }

    else {
      //Serial.println("Function can run");
      block_servo2_down = false; //the second joint is allowed to go in any direction
      return false;
    }
  }

  else { //if not moving: check if it is safe to move from current position
    read_joint_positions();

    if ((1289 < current_pos_arr[2] && current_pos_arr[2] < 4096) || (2200 < current_pos_arr[1] && current_pos_arr[1] < 4096) || (3895 < current_pos_arr[0] || current_pos_arr[0] < 200)) {
      Serial.println("Not runable. Move servo to another position.");
      return true;
    }

    else {
      Serial.println("Function can run");
      return false;
    }
  }
}

int robotarm::MoveL(float end_x, float end_y, float end_z, float tf) { //function to move linearly to a desired position

  if (not_runable(false)) return 0;
  calc_forward(); //make sure it knows it's current position in cartesian space

  //x
  float x_f = end_x; //end position
  float a1_0 = P0EE[0]; //start position
  //float a1_1 = 0; //start velocity
  float a1_2 = 3 / (tf * tf) * (x_f - a1_0);
  float a1_3 = -2 / (tf * tf * tf) * (x_f - a1_0);

  //y
  float y_f = end_y; //end position
  float a2_0 = P0EE[1]; //start position
  //float a2_1 = 0; //start velocity
  float a2_2 = 3 / (tf * tf) * (y_f - a2_0);
  float a2_3 = -2 / (tf * tf * tf) * (y_f - a2_0);

  //z
  float z_f = end_z; //end position
  float a3_0 = P0EE[2]; //start position
  //float a3_1 = 0; //start velocity
  float a3_2 = 3 / (tf * tf) * (z_f - a3_0);
  float a3_3 = -2 / (tf * tf * tf) * (z_f - a3_0);

  //move joints every 1/66 second
  for (float t = 0.025; t <= tf; t += 0.025) {

    //update P0EE vector to next point on the path
    P0EE[0] = a1_0 + a1_2 * (t * t) + a1_3 * (t * t * t);
    P0EE[1] = a2_0 + a2_2 * (t * t) + a2_3 * (t * t * t);
    P0EE[2] = a3_0 + a3_2 * (t * t) + a3_3 * (t * t * t);

    calc_inverse(); //use inverse kinematics to convert desired position from cartesian to joint space. The calculated joint values will be stored in the "calc_pos_arr" array.

    if (not_runable(true)) return 0; //check if the robot goes to a "dangerous position" before we move it

    Dynamixel.servo(SERVO_ID1, calc_pos_arr[0], 0x40); //  Move servo to max angle at speed 40 of 1023
    delay(4);
    Dynamixel.servo(SERVO_ID2, calc_pos_arr[1], 0x40); //  Move servo to max angle at speed 40 of 1023
    Dynamixel.servo(SERVO_ID3, calc_pos_arr[2], 0x40); //  Move servo to max angle at speed 40 of 1023

    delay(6);
  }
  Serial.println("Position reached");

  return 0;
}

void robotarm::read_joint_positions() {
  current_pos_arr[0] = Dynamixel.readPosition(SERVO_ID1);
  while (current_pos_arr[0] == 61568) current_pos_arr[0] = Dynamixel.readPosition(SERVO_ID1); //read again if error

  current_pos_arr[1] = Dynamixel.readPosition(SERVO_ID2);
  while (current_pos_arr[1] == 61568) current_pos_arr[1] = Dynamixel.readPosition(SERVO_ID2); //read again if error

  current_pos_arr[2] = Dynamixel.readPosition(SERVO_ID3);
  while (current_pos_arr[2] == 61568) current_pos_arr[2] = Dynamixel.readPosition(SERVO_ID3); //read again if error
}



int robotarm::move_along_XYZ() {
  calc_inverse();
  if (not_runable(true)) return 0; //check if the robot goes to a "dangerous position" before we move it

  Dynamixel.servo(SERVO_ID1, CrustCrawler.calc_pos_arr[0], 0x40); //  Move servo to max angle at speed 40 of 1023
  delay(4);
  Dynamixel.servo(SERVO_ID2, CrustCrawler.calc_pos_arr[1], 0x40); //  Move servo to max angle at speed 40 of 1023
  Dynamixel.servo(SERVO_ID3, CrustCrawler.calc_pos_arr[2], 0x40); //  Move servo to max angle at speed 40 of 1023

  //Matrix.Print((mtx_type*)T0EE, N, N, "T0EE");
  Matrix.Print((mtx_type*)P0EE, N, 1, "P0EE");

  Serial.println("Steps:");
  Serial.println(CrustCrawler.calc_pos_arr[0]);
  Serial.println(CrustCrawler.calc_pos_arr[1]);
  Serial.println(CrustCrawler.calc_pos_arr[2]);
  Serial.println(" ");
  return 0;
}

void setup() {
  delay(1000);                                                            // Give time for Dynamixel to start on power-up
  Serial.begin(57600);

  //PS2 setup
  error = ps2x.config_gamepad(13, 11, 10, 12, true, true); //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error

  if (error == 0) {
    Serial.println("Found Controller, configured successful");
  }

  else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");


  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      Serial.println("GuitarHero Controller Found");
      break;
  }


  //Dynamixel setup
  Dynamixel.begin(SERVO_SET_Baudrate);                                    // We now need to set Ardiuno to the new Baudrate speed
  Dynamixel.setDirectionPin(SERVO_ControlPin);                            // Optional. Set direction control pin

  Dynamixel.setMode(SERVO_ID1, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);    // set mode to SERVO and set angle limits (adjust)
  Dynamixel.setMode(SERVO_ID2, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);    // set mode to SERVO and set angle limits (adjust)
  Dynamixel.setMode(SERVO_ID3, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);    // set mode to SERVO and set angle limits (adjust)
  Dynamixel.setMode(SERVO_ID4, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);    // set mode to SERVO and set angle limits
  Dynamixel.setMode(SERVO_ID5, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);    // set mode to SERVO and set angle limits

  //  CrustCrawler.MoveJ(2048, 1024, 1024, 4); //go to home positon
  //  CrustCrawler.MoveL(0, 0, 0, 4); //go to illegal position (safety test)

  CrustCrawler.calc_forward(); //make sure the robot knows where it is before we start moving it
  CrustCrawler.MoveJ(2292, 510, 1154, 3);


}


void loop() {

  if (error == 1) //skip loop if no controller found
    return;
  ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed

  if (ps2x.ButtonPressed(PSB_RED)) {   //circle pressed
    CrustCrawler.ChangeGripper(true);
  }

  if (ps2x.ButtonReleased(PSB_PINK)) {           //Firkant trykkes
    CrustCrawler.ChangeGripper(false);
  }

  if (ps2x.ButtonReleased(PSB_GREEN)) {           //Trekant pressed
    //  pick-up-bottle position
    CrustCrawler.MoveJ(1171, 689, 1096, 3);
    CrustCrawler.MoveJ(2184, 724, 1124, 3);

    movebottle();
  }

  if (ps2x.NewButtonState(PSB_BLUE)) {     //X pressed
    //CrustCrawler.MoveJ(2048, 1024, 1024, 3); //go to home positon
    //test();
    accuracy_test();
  }

  if (ps2x.Button(PSB_PAD_UP)) {
    P0EE[2] += 0.1;
    CrustCrawler.move_along_XYZ();
  }

  if (ps2x.Button(PSB_PAD_DOWN)) {
    if (!CrustCrawler.block_servo2_down) { //only go further down if the current position of servo 2 allows to
      P0EE[2] -= 0.1;
      CrustCrawler.move_along_XYZ();
    }
  }

  if (ps2x.Button(PSB_START)) {
    P0EE[0] += 0.1;
    CrustCrawler.move_along_XYZ();
  }

  if (ps2x.Button(PSB_SELECT)) {
    P0EE[0] -= 0.1;
    CrustCrawler.move_along_XYZ();
  }

  if (ps2x.Button(PSB_L1))   {
    P0EE[1] -= 0.1;
    CrustCrawler.move_along_XYZ();
  }

  if (ps2x.Button(PSB_R1))    {
    P0EE[1] += 0.1;
    CrustCrawler.move_along_XYZ();
  }
}

void movebottle() {

  float fast = 0.5;
  float slow = 1.1;

  CrustCrawler.ChangeGripper(true);
  CrustCrawler.MoveL(-36.32, -8.31, 10.16, fast);
  CrustCrawler.ChangeGripper(false); //grip the object
  CrustCrawler.MoveL(-36.32, -8.31, 15.16, fast); //lift the object
  CrustCrawler.MoveJ(1874, 618, 921, slow); //move to next pos
  CrustCrawler.MoveL(-36.08, 10.02, 9.69, fast); //lower to ground
  CrustCrawler.ChangeGripper(true);
  CrustCrawler.MoveL(-29.07, 8.31, 10.07, fast); //move back
  CrustCrawler.MoveL(-36.08, 10.02, 9.69, fast); //move forward again
  CrustCrawler.ChangeGripper(false);
  CrustCrawler.MoveJ(1874, 618, 921, fast); //lift from ground
  CrustCrawler.MoveL(-36.32, -8.31, 15.16, slow); //move to start position
  CrustCrawler.MoveL(-36.32, -8.31, 10.16, fast); //lower the object
  CrustCrawler.ChangeGripper(true);
  CrustCrawler.MoveJ(2184, 724, 1124, fast);

  movebottle();
}

void test(){

  float fart = 3;//(3=2.6 seconds)
/*
    CrustCrawler.MoveL(-36.93, 0.48, 24.43, fart);
    CrustCrawler.MoveL(-31.33, -9.02, 17.13, fart);
    CrustCrawler.MoveL(-23.93, 17.68, 24.23, fart);
    CrustCrawler.MoveL(-36.93, 0.48, 24.43, fart);

    CrustCrawler.MoveJ(2230, 793, 1045, fart);
    CrustCrawler.MoveJ(1632, 932, 1033, fart);
    CrustCrawler.MoveJ(2039, 715, 759, fart);

    */

  }

void accuracy_test(){
      float fart = 3;//(3=2.6 seconds)
    
      for(int i=0; i<5; i++){
          CrustCrawler.MoveL(-29.6, 0, -13.22, fart);
          CrustCrawler.MoveL(-29.6, 0, -16.22, fart);
          CrustCrawler.MoveL(-29.6, 0, -13.22, fart);
          CrustCrawler.MoveJ(2292, 510, 1154, fart);
      }
  }
