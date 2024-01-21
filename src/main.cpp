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

motor lBackWheel = motor(PORT1, ratio6_1, true);
motor rBackWheel = motor(PORT2, ratio6_1, false);
motor lFrontWheel = motor(PORT3, ratio6_1, true);
motor rFrontWheel = motor(PORT4, ratio6_1, false);
motor Catapult = motor(PORT10, ratio36_1, false);
pot CatapultAngle = pot(Brain.ThreeWirePort.A);
limit CataLimSwitch = limit(Brain.ThreeWirePort.A);

drivetrain Drivetrain = drivetrain(lBackWheel, rBackWheel, 319.19, 24.5, 293, mm, 1);

bool shootToggle = false;
int timeSinceCatapultReleased = 0;


inline void updateShootToggle() {
    shootToggle = !shootToggle;
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
    lBackWheel.setStopping(brakeType::hold);
    rBackWheel.setStopping(brakeType::hold);
    Catapult.setStopping(brakeType::hold);

    Catapult.setVelocity(99, percent);
    Catapult.setPosition(0, deg); // zero out catapult at the not-charged position
    lBackWheel.stop();
    rBackWheel.stop();
    Catapult.stop();

    CataLimSwitch.pressed(updateShootToggle);
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
    lBackWheel.setVelocity(100, percent);
    rBackWheel.setVelocity(100, percent);
    lBackWheel.spin(forward);
    rBackWheel.spin(forward);
    wait(5, sec);
    lBackWheel.spin(reverse);
    rBackWheel.spin(reverse);
    wait(500, msec);

    lBackWheel.stop();
    rBackWheel.stop();

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
        // Drivetrain control
        int speed = Controller1.ButtonL1.pressing() ? 200:100;
        int hAxis = Controller1.Axis1.position();
        int vAxis = Controller1.Axis3.position();
        int lVel = ((vAxis - hAxis)/100) * speed;
        int rVel = ((vAxis + hAxis)/100) * speed;
        if(abs(lVel) == 0)
            lBackWheel.stop();
        else {
            lBackWheel.setVelocity(lVel, percent);
            lBackWheel.spin(forward);
        }
        if(abs(rVel) == 0)
            rBackWheel.stop();
        else {
            rBackWheel.setVelocity(rVel, percent);
            rBackWheel.spin(forward);
        }
        
        //catapult control
        if(Controller1.ButtonR1.pressing()) {
            if(!shootToggle)
                Catapult.spin(forward);
            else {
                Catapult.stop();
                if(timeSinceCatapultReleased >= 100) {
                    shootToggle = !shootToggle;
                    lBackWheel.spin(reverse);
                    timeSinceCatapultReleased = 0;
                }
            }
        }
        else {
            Catapult.stop();
        }

        if(shootToggle)
            timeSinceCatapultReleased += 40;
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
