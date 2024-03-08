#include "main.h"

#define lupPort -7
#define liddlePort -8
#define lownPort -9
#define rupPort 18
#define riddlePort 19
#define rownPort 20

#define cataPort 1
#define imuPort 2

#define intakePort 10

#define WingPort 'A'
#define ClimbPort 'B'

//Electronics

pros::Controller silly_geese(pros::E_CONTROLLER_MASTER);

pros::Motor lown(lownPort, pros::E_MOTOR_GEARSET_06); //right odom wheel
pros::Motor rown(rownPort, pros::E_MOTOR_GEARSET_06); //left odom wheel
pros::Motor liddle(liddlePort, pros::E_MOTOR_GEARSET_06); //right odom wheel
pros::Motor riddle(riddlePort, pros::E_MOTOR_GEARSET_06); //left odom wheel
pros::Motor rup(rupPort, pros::E_MOTOR_GEARSET_06); //right odom wheel
pros::Motor lup(lupPort, pros::E_MOTOR_GEARSET_06); //left odom wheel

pros::Motor cata(cataPort);

pros::MotorGroup lall ({lup, liddle, lown});
pros::MotorGroup rall ({rup, riddle, rown});

pros::Motor intake(intakePort, pros::E_MOTOR_GEARSET_06);

pros::ADIDigitalOut wings(WingPort);
pros::ADIDigitalOut climb(ClimbPort);

pros::IMU inertial(imuPort);
/////
// For installation, upgrading, documentations and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
ez::Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is used as the sensor
  {-7, -7, -8, -9}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is used as the sensor
  ,{18, 18, 19, 20}

  // IMU Port
  ,2

  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
  ,4.125

  // Cartridge RPM
  ,600

  // External Gear Ratio (MUST BE DECIMAL) This is WHEEL GEAR / MOTOR GEAR
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 84/36 which is 2.333
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 36/60 which is 0.6
  ,.6666666666666666666
);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0); // Sets the active brake kP. We recommend 0.1.
  chassis.opcontrol_curve_default_set(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    Auton("Example Drive\n\nDrive forward and come back.", drive_example),
    Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}



/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset(); // Resets PID targets to 0
  chassis.drive_imu_reset(); // Reset gyro position to 0
  chassis.drive_sensor_reset(); // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency

  // ez::as::auton_selector.selected_auton_call(); // Calls selected auton from autonomous selector
  //drive_and_turn();
swing_example();
}



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  

int left_steez;
int right_steez;

bool wing_toggle = 0;

int intake_steez = 0;

while (true)
{
  intake_steez = (silly_geese.get_digital(pros::E_CONTROLLER_DIGITAL_R1) - silly_geese.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) * 127;

  intake.move(intake_steez);

  //drive train logic
	left_steez = silly_geese.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * (12000.0 / 127);
	right_steez = silly_geese.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * (120000.0 / 127);

	lall.move(left_steez);
	rall.move(right_steez);

  if (silly_geese.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        if (wing_toggle)
        {
            wings.set_value(false);
            wing_toggle = 0;
        } else {
            wings.set_value(true);
            wing_toggle = 1;
        }
    }

}
}
