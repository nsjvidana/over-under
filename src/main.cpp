/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Nilson JVP                                                */
/*    Created:      10/13/2023, 10:22:01 AM                                   */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

vex::brain       Brain;

controller Controller1 = controller(primary);

motor LeftMotor = motor(PORT2, ratio6_1, false);
motor RightMotor = motor(PORT1, ratio6_1, true);

// drivetrain Drivetrain = drivetrain(LeftMotor, RightMotor, 319.19, 228.6, 190.5, mm, 1);

motor Feeder = motor(PORT16, ratio18_1, false);
motor FeederJoint = motor(PORT13, ratio36_1, false);


//in inches
double wheelRadius = 4;
double circumference = 2 * M_PI * wheelRadius;
double driveVelocity = 30;
double turnVelocity = 20;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  FeederJoint.setStopping(brakeType::brake);
  FeederJoint.setVelocity(200, percent);
  Feeder.setStopping(brakeType::brake);

  LeftMotor.setStopping(brakeType::brake);
  RightMotor.setStopping(brakeType::brake);
}


/***************************Autonomous driving utils***************************/

inline void driveFor(double distanceInInches) {
  double amountOfTurns = (distanceInInches / circumference)*2;

  LeftMotor.setVelocity(driveVelocity, percent);
  RightMotor.setVelocity(driveVelocity, percent);

  LeftMotor.spinFor(amountOfTurns*360, deg, false);
  RightMotor.spinFor(amountOfTurns*360, deg, true);
}

inline void turn90Deg(int direction) {
  double arcLength = (circumference*90)/360;
  double degreesToTurn = (arcLength / circumference)*360*2;

  LeftMotor.setVelocity(turnVelocity, percent);
  RightMotor.setVelocity(turnVelocity, percent);
  
  LeftMotor.spinFor(degreesToTurn*-direction, deg, false);
  RightMotor.spinFor(degreesToTurn*direction, deg, true);
}

inline void turnALittle(int direction) {
  double arcLength = (circumference*90)/360;
  double degreesToTurn = (arcLength / circumference)*360;

  LeftMotor.setVelocity(turnVelocity, percent);
  RightMotor.setVelocity(turnVelocity, percent);
  
  LeftMotor.spinFor(degreesToTurn*-direction, deg, false);
  RightMotor.spinFor(degreesToTurn*direction, deg, true);
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  Brain.Screen.print("Mode: Auto");

  turnALittle(-1);
  wait(1, sec);
  driveFor(18);
  
  // //drop ball
  FeederJoint.spinFor(1300, deg);
  Feeder.spinFor(-800, deg);
  FeederJoint.spinFor(-1300, deg);

  //ram ball
  driveVelocity = 80;
  driveFor(24);


  // driveFor(60);
  // turn90Deg(-1);
  // driveFor(-6);

  // //drop ball
  // FeederJoint.spinFor(1300, deg);
  // Feeder.spinFor(-800, deg);
  // FeederJoint.spinFor(-1300, deg);

  // //ram ball
  driveVelocity = 80;
  driveFor(24);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  Brain.Screen.clearLine(1);
  Brain.Screen.print("Mode: Drive");

  while (1) {
    //Drivetrain control
    int hAxis = Controller1.Axis1.position();
    int vAxis = Controller1.Axis3.position();
    LeftMotor.setVelocity(vAxis + hAxis, percent);
    RightMotor.setVelocity(vAxis - hAxis, percent);
    LeftMotor.spin(forward);
    RightMotor.spin(forward);

    //Feeder control (incomplete)
    //  When R1 is pressed, go backward.
    //  When R2 is pressed, go forward.
    //  When both are pressed, don't move.
    //  Hold X to spin feeder faster
    int feederSpeed = Controller1.ButtonX.pressing() ? 200 : 80;
    int feederDirection = Controller1.ButtonR2.pressing() - Controller1.ButtonR1.pressing();
    Feeder.setVelocity(feederSpeed*feederDirection, percent);

    //L1 and L2 for controlling feeder joint
    int feederJointDirection = Controller1.ButtonL1.pressing() - Controller1.ButtonL2.pressing();
    FeederJoint.setVelocity(50*feederJointDirection, percent);
    Feeder.spin(forward);
    FeederJoint.spin(forward);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}




//
// Main will set up the competition functions and callbacks.
//
int main() {

  // Run the pre-autonomous function.
  pre_auton();

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // autonomous();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
