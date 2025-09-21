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
motor topleft = motor(PORT11, ratio18_1, true);
motor middleleft = motor(PORT9, ratio18_1, true);

motor bottomright = motor(PORT14, ratio18_1, false);
motor topright = motor(PORT12, ratio18_1, false);
motor middleright = motor(PORT10, ratio18_1, true);

motor intake1 = motor(PORT15, ratio18_1, false);
motor intake2 = motor(PORT16, ratio18_1, true);
motor intake3 = motor(PORT17, ratio18_1, true);
motor intake4 = motor(PORT18, ratio18_1, true);

controller Controller = controller(primary);

motor_group leftmotors = motor_group(topleft, middleleft, bottomleft);
motor_group rightmotors = motor_group(topright, middleright, bottomright);

drivetrain dt = drivetrain(leftmotors, rightmotors, 300, 320, 320, mm, 1);

//digital_out* clinch;

bool tank = false;
double leftvalue;
double rightvalue;

void switchtype() {
  tank = !tank;
}

void useClinch() {
  //clinch->set(!clinch->value());
}

void toBasket(){
  intake3.setVelocity(100,percent);
  intake1.setVelocity(80,percent);
  intake2.setVelocity(80,percent);
  intake3.spin(reverse);
  intake1.spin(forward);
  intake2.spin(forward);
}

void stopBasket(){
  intake3.stop();
  intake1.stop();
  intake2.stop();
}

void basketToBottom(){
  intake3.setVelocity(100,percent);
  intake1.setVelocity(80,percent);
  intake2.setVelocity(80,percent);
  intake3.spin(forward);
  intake1.spin(reverse);
  intake2.spin(reverse);
}

void stopBottom(){
  intake3.stop();
  intake1.stop();
  intake2.stop();
}

void basketToTop(){
  intake3.setVelocity(100,percent);
  intake2.setVelocity(100,percent);
  intake4.setVelocity(100,percent);
  intake1.setVelocity(30,percent);
  intake3.spin(reverse);
  intake4.spin(forward);
  intake1.spin(reverse);
  intake2.spin(reverse);
}

void stopTop(){
  intake3.stop();
  intake4.stop();
  intake1.stop();
  intake2.stop();
}

void basketToMiddle(){
  intake3.setVelocity(100,percent);
  intake2.setVelocity(100,percent);
  intake4.setVelocity(100,percent);
  intake1.setVelocity(30,percent);
  intake3.spin(reverse);
  intake4.spin(reverse);
  intake1.spin(reverse);
  intake2.spin(reverse);
}

void stopMiddle(){
  intake3.stop();
  intake4.stop();
  intake1.stop();
  intake2.stop();
}

void pre_auton(void) {
  vexcodeInit();

  leftmotors.setStopping(brake);
  rightmotors.setStopping(brake);

  leftmotors.setVelocity(100,percent);
  rightmotors.setVelocity(100,percent);

  //intake.setVelocity(100,percent);

  //clinch->set(true);

  Controller.ButtonLeft.pressed(switchtype);
  //Controller.ButtonL1.pressed(useClinch);

  Controller.ButtonL1.pressed(toBasket);
  Controller.ButtonL1.released(stopBasket);

  Controller.ButtonR1.pressed(basketToTop);
  Controller.ButtonR1.released(stopTop);

  Controller.ButtonR2.pressed(basketToMiddle);
  Controller.ButtonR2.released(stopMiddle);

  Controller.ButtonL2.pressed(basketToBottom);
  Controller.ButtonL2.released(stopBottom);
}

void autonomous(void) {

}

void usercontrol(void) {
  while (1) {
    if (tank) {
      leftmotors.setVelocity(Controller.Axis3.position(), percent);
      rightmotors.setVelocity(Controller.Axis2.position(), percent);
    } else {
      leftvalue = (Controller.Axis4.position() + Controller.Axis3.position());
      rightvalue = (Controller.Axis4.position() - Controller.Axis3.position()) * -1;
      leftmotors.setVelocity(leftvalue, percent);
      rightmotors.setVelocity(rightvalue, percent);
    }

    leftmotors.spin(forward);
    rightmotors.spin(forward);

    wait(20, msec); 
  }
}

int main() {
  digital_out c(Brain.ThreeWirePort.A);
  //clinch = &c;

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
