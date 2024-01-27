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

vex::brain Brain;

controller Controller1 = controller(primary);

motor lBackWheel(PORT2, ratio6_1, false);
motor lFrontWheel(PORT3, ratio6_1, false);
motor rBackWheel(PORT1, ratio6_1, true);
motor rFrontWheel(PORT5, ratio6_1, true);
motor wall1(PORT4, ratio18_1, false);
motor wall2(PORT6, ratio18_1, true);

motor_group leftMotors(lBackWheel, lFrontWheel);
motor_group rightMotors(rBackWheel, rFrontWheel);
motor_group wall(wall1, wall2);

motor catapult(PORT10, ratio36_1, false);
limit cataLimSwitch(Brain.ThreeWirePort.A);

bool shootToggle = false;
int timeSinceCatapultReleased = 0;
brakeType driveTrainBrakeMode = brakeType::brake;

/**
 * Invoked when CatapultLimSwitch is pressed
*/
inline void updateShootToggle() {
    shootToggle = !shootToggle;
}

/**
 * Invoked when X butotn is pressed
*/

inline void toggleBrakes() {
    switch (driveTrainBrakeMode) {
        case brakeType::brake:
            driveTrainBrakeMode = brakeType::hold;
            Controller1.Screen.setCursor(30, 10);
            Controller1.Screen.print("brakes: ON       ");
            break;
        
        default:
            driveTrainBrakeMode = brakeType::brake;
            Controller1.Screen.setCursor(30, 10);
            Controller1.Screen.print("brakes: OFF       ");
            break;
    }
}

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
    leftMotors.setStopping(brake);
    rightMotors.setStopping(brake);
    leftMotors.stop();
    rightMotors.stop();

    catapult.setStopping(hold);
    wall.setStopping(hold);
    wall.stop();
    catapult.stop();

    catapult.setVelocity(100, percent);
    catapult.setPosition(0, deg); // zero out catapult at the not-charged position

    cataLimSwitch.pressed(updateShootToggle);
    Controller1.ButtonX.pressed(toggleBrakes);
    
    Controller1.Screen.setCursor(30, 10);
    Controller1.Screen.print("brakes: OFF       ");
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
    leftMotors.setVelocity(100, percent);
    rightMotors.setVelocity(100, percent);

    leftMotors.spin(forward);
    rightMotors.spin(forward);
    wait(3, sec);

    leftMotors.spin(reverse);
    rightMotors.spin(reverse);
    wait(500, msec);

    leftMotors.stop();
    rightMotors.stop();
}

/**
 * 4-wheel drive.
*/
inline void controlDriveTrain(int hAxis, int vAxis, int speed) {

    int lVel = ((vAxis + hAxis)/100.0) * speed;
    int rVel = ((vAxis - hAxis)/100.0) * speed;
    
    if(lVel == 0 && rVel == 0){
        leftMotors.stop(driveTrainBrakeMode);
        rightMotors.stop(driveTrainBrakeMode);
    }
    else {
        leftMotors.spin(forward, lVel, pct);
        rightMotors.spin(forward, rVel, pct);
    }

}

/**
 * Press R1 to charge catapult
*/
inline void controlCatapult(controller::button ctrlBtn) {
    
    if(ctrlBtn.pressing()) {
        if(!shootToggle) //charge catapult if don't shoot
            catapult.spin(forward);
        else {
            catapult.stop();//stop charging catapult
            if(timeSinceCatapultReleased >= 100) {//wait until catapult stopped
                shootToggle = !shootToggle;
                timeSinceCatapultReleased = 0;
            }
        }
    }
    else {
        catapult.stop();
    }

    if(shootToggle)
        timeSinceCatapultReleased += 20;
    
}

inline void controlWall(controller::button upBtn, controller::button downBtn, int velPct) {
    int dir = (int)upBtn.pressing() - (int)downBtn.pressing();
    wall.setVelocity(velPct * dir, pct);

    if(dir == 0)
        wall.stop();
    else
        wall.spin(forward);
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

    while(1) {
        controlDriveTrain(
            Controller1.Axis1.position(), //x-axis
            Controller1.Axis3.position(), //y-axis
            Controller1.ButtonR2.pressing() ? 100:35 //press R2 to boost
        );

        controlCatapult(Controller1.ButtonR1);

        controlWall(Controller1.ButtonL1, Controller1.ButtonL2, 70);

        wait(20, msec);
    }
}


int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();

    while(true) {
        wait(5, msec);
    }
}
