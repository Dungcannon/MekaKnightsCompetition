// -- Config --
//[Name]              [Port]
//LeftDriveSmart      1
//RightDriveSmart     2
//DrivetrainInertial  3
//Drivetrain          1,2,3
//Harvester           4
//Catapult            5
//Frills              6 // This is now not using motors so Frills is pointless
//Solenoid            A 
// -- Config --

//should set two button system or a hold sys for solenoid.
//Solenoid.set(true);
#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;

motor LeftDriveSmart = motor(PORT1, ratio18_1, false);
motor RightDriveSmart = motor(PORT2, ratio18_1, true);
inertial DrivetrainInertial = inertial(PORT3);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 319.19, 320, 40, mm, 1);

controller Controller1 = controller(primary);
motor Harvester = motor(PORT4, ratio18_1, false);

motor Catapult = motor(PORT5, ratio18_1, false);

motor Frills = motor(PORT6, ratio18_1, false);

digital_out Solenoid = digital_out(Brain.ThreeWirePort.A);

//For Pneumatic
bool pistonSetState = false;  // Tracks set state of piston
bool waitingOnRelease = false;  // Monitor for release of button so that pistonSetState doesn't continually bounce back and forth every 20ms

//For calibrationvoid 
void calibrateDrivetrain() {
  wait(200, vex::msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  DrivetrainInertial.calibrate();
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, vex::msec);
  }

  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}// define variable for remote controller enable/disable


//For Auto before round start.
bool getHappy = true; 
void triggerHappy(int timems) 
{
  int i = 0;
  getHappy = true;
  while (getHappy == true) 
  {
    if (i >= timems) { getHappy = false; return;}
    Catapult.spin(forward);
    wait(10, msec);
    i+=10;
  }
}

void Pneumatic(bool on){
  // Solenoid = true;// wrong code
  // for auto, needs to toggle pneumatic. 
  return;
}

void Forward(int x){
  Drivetrain.driveFor(forward, x, mm);
  return;
}

void TurnTo(int x){
  Drivetrain.turnToHeading(x, degrees);
  return;
}

void pre_auton(void) {
  calibrateDrivetrain();
  Catapult.setVelocity(100, percent); // catapult shoot speed
  Catapult.setMaxTorque(100, percent); // catapult torque
  Catapult.setStopping(hold);
  Solenoid.set(false);
}



void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}



void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      // check the ButtonL1/ButtonL2 status to control Harvester
      if (Controller1.ButtonL1.pressing()) {
        Harvester.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        Harvester.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        Harvester.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
      // check the ButtonR1/ButtonR2 status to control Catapult
      if (Controller1.ButtonR1.pressing()) {
        Catapult.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        Catapult.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        Catapult.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
      // check the buttonX status to control Pneumatics
      if (Controller1.ButtonX.pressing()){
        Solenoid.set(true);
      }
      // check buttonB status
      if (Controller1.ButtonB.pressing()){ 
        Solenoid.set(false);
      }
    }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
