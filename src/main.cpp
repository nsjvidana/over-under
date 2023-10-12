/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       nlson                                                     */
/*    Created:      9/19/2023, 3:34:30 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

controller Controller1 = controller(primary);

//TODO: change ports/ratios value when actually building stuff
motor LeftMotor = motor(PORT2, ratio6_1, false);
motor RightMotor = motor(PORT1, ratio6_1, true);

//TODO maybe change the last parameter or smth
drivetrain DriveTrain = drivetrain(LeftMotor, RightMotor, 320, 203, 190, vex::distanceUnits::mm, 6);

motor Feeder = motor(PORT3, ratio18_1, false);

vex::timer Timer;


/**
 * 
*/
inline void autonomous() {
        
    //NOTE: half len is 6in
    //      Same for the feeder 
    //      Mat size is 2' x 2' (24" x 24")

    DriveTrain.driveFor(24,vex::distanceUnits::in);
    DriveTrain.turnFor(90, vex::rotationUnits::deg);
    DriveTrain.driveFor(60,vex::distanceUnits::in);
    DriveTrain.turnFor(90, vex::rotationUnits::deg);
    //extrude feeder and take the triball
    DriveTrain.driveFor(6, vex::distanceUnits::in);
    DriveTrain.turnFor(180, vex::rotationUnits::deg);
    //drop the triball and take the feeder back in to not bash the feeder
    DriveTrain.setDriveVelocity(99, percent);
    DriveTrain.driveFor(18, vex::distanceUnits::in); //ram into the ball

}


/**
 * Move the robot using the left joystick
*/
inline void controlDriveTrain() {
    int hAxis = Controller1.Axis4.position();
    int vAxis = Controller1.Axis3.position();
    LeftMotor.setVelocity(vAxis + hAxis, percent);
    RightMotor.setVelocity(vAxis - hAxis, percent);

    LeftMotor.spin(forward);
    RightMotor.spin(forward);
}

/**
 * Controll the feeder using the R1 and R2 buttons
 * Press R1 to reel in, R2 to release (this might change depending on team's decisions)
*/
inline void controlFeeder() {
    //Subtract the states to get the velocity's sign
    //  When R1 is pressed, go backward.
    //  When R2 is pressed, go forward.
    //  When both are pressed, don't move.
    int direction = Controller1.ButtonR2.pressing() - Controller1.ButtonR1.pressing();
    Feeder.setVelocity(80*direction, percent);
}

int main() {
    // Allow other tasks to run
    this_thread::sleep_for(10);

    Brain.Screen.print("Hello o/");
    Brain.Screen.newLine();

    //15s autonomous mode
    Brain.Screen.clearLine(2);
    Brain.Screen.print("Mode: Auto");
    while(timer().system() < 15000) {
        autonomous();
    }

    //switch to drive mode
    Brain.Screen.clearLine(2);
    Brain.Screen.print("Mode: Drive");
    while(1) {

        controlDriveTrain();
        controlFeeder();

        wait(1, msec);
    }

    return 0;
}
