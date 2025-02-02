/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 10, D        
// ClawMotor            motor         3               
// ArmMotor             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

competition Competition;

motor bottomleft = motor(PORT13, ratio18_1, true);
motor bottomright = motor(PORT14, ratio18_1, false);
motor topleft = motor(PORT11, ratio18_1, true);
motor topright = motor(PORT12, ratio18_1, false);

motor intake1 = motor(PORT15, ratio18_1, false);
motor intake2 = motor(PORT16, ratio18_1, true);
motor intake3 = motor(PORT17, ratio18_1, true);

motor wall = motor(PORT1, ratio18_1, true);

controller Controller = controller(primary);

motor_group leftmotors = motor_group(topleft, bottomleft);
motor_group rightmotors = motor_group(topright, bottomright);
motor_group intake = motor_group(intake1,intake2,intake3);

drivetrain dt = drivetrain(leftmotors, rightmotors, 300, 320, 320, mm, 1);

digital_out* clinch;

bool tank = false;
double leftvalue;
double rightvalue;

void switchtype() {
  tank = !tank;
}

void useClinch() {
  clinch->set(!clinch->value());
}

void spinIntake(){
  intake.spin(forward);
}

void reverseIntake(){
  intake.spin(reverse);
}

void stopIntake(){
  intake.stop();
}

void wallIn(){
  wall.spinToPosition(0,degrees);
}

void wallOut(){
  wall.spin(forward);
}

void wallStop(){
  wall.stop();
}

void pre_auton(void) {
  vexcodeInit();

  leftmotors.setStopping(brake);
  rightmotors.setStopping(brake);
  wall.setStopping(brake);
  wall.setBrake(hold);
  wall.setPosition(0,degrees);

  intake.setVelocity(100,percent);

  clinch->set(true);

  Controller.ButtonLeft.pressed(switchtype);
  Controller.ButtonL1.pressed(useClinch);

  Controller.ButtonR1.pressed(spinIntake);
  Controller.ButtonR1.released(stopIntake);
  Controller.ButtonR2.pressed(reverseIntake);
  Controller.ButtonR2.released(stopIntake);

  Controller.ButtonX.pressed(wallOut);
  Controller.ButtonX.released(wallStop);
  Controller.ButtonB.released(wallIn);
}

void autonomous(void) {
  //set velocity
  dt.setDriveVelocity(20,percent);
  dt.setTurnVelocity(20,percent);

  //do wall stake
  dt.driveFor(forward,3,inches);
  dt.turnFor(32,degrees);
  dt.driveFor(forward,8.5,inches);
  dt.turnFor(15,degrees);
  dt.driveFor(forward,1.5,inches);
  wall.spin(forward);
  wait(2,sec);
  wall.stop();

  //get the stake
  dt.driveFor(reverse,14,inches);
  dt.turnFor(18, degrees);
  dt.driveFor(reverse,14,inches);
  useClinch();
  
  //get the first ring
  dt.driveFor(forward,3, inches);
  dt.turnFor(32,degrees);
  intake.spin(forward);
  dt.driveFor(forward,10,inches);
  
  //touch the bar
  dt.driveFor(reverse,5,inches);
  dt.turnFor(50,degrees);
  wall.spinToPosition(0,degrees,false);
  dt.setDriveVelocity(100,percent);
  dt.driveFor(forward,15,inches);
}

void usercontrol(void) {
  while (1) {
    if (tank) {
      bottomleft.setVelocity(Controller.Axis3.position(), percent);
      topleft.setVelocity(Controller.Axis3.position(), percent);
      topright.setVelocity(Controller.Axis2.position(), percent);
      bottomright.setVelocity(Controller.Axis2.position(), percent);
    } else {
      leftvalue = (Controller.Axis4.position() + Controller.Axis3.position());
      rightvalue = (Controller.Axis4.position() - Controller.Axis3.position()) * -1;
      bottomleft.setVelocity(leftvalue, percent);
      topleft.setVelocity(leftvalue, percent);
      topright.setVelocity(rightvalue, percent);
      bottomright.setVelocity(rightvalue, percent);
    }

    bottomleft.spin(forward);
    bottomright.spin(forward);
    topleft.spin(forward);
    topright.spin(forward);

    wait(20, msec); 
  }
}

int main() {
  digital_out c(Brain.ThreeWirePort.A);
  clinch = &c;

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
