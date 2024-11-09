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

controller Controller = controller(primary);

motor_group leftmotors = motor_group(topleft, bottomleft);
motor_group rightmotors = motor_group(topright, bottomright);

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

void pre_auton(void) {
  vexcodeInit();

  leftmotors.setStopping(brake);
  rightmotors.setStopping(brake);

  clinch->set(false);

  Controller.ButtonLeft.pressed(switchtype);
  Controller.ButtonR1.pressed(useClinch);
}

void autonomous(void) {
  // Autonomous code goes here
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
