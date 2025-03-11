#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include <cmath>
#include "pros/rtos.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
// drivetrain settings


// all of the devices are in the header file devices.h. If this does not work, move them back to main.cpp





pros::MotorGroup left_motors({-18,-19,20}, pros::MotorGearset::green); // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({11,12 ,-13 }, pros::MotorGearset::green); // right motors on ports 4, 5, 6
pros::Motor Intake(10, pros::MotorGearset::blue);
pros::Motor LadyBrown(-2,pros::MotorGearset::green);
pros::adi::Pneumatics clamp_mech('A', false);
pros::adi::Pneumatics doinker('B', false);
pros::Rotation rotationSensor(1);



lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::OLD_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
// create an imu on port 5
pros::Imu imu(14);
pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);


// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);


// lateral PID controller
lemlib::ControllerSettings lateral_controller(23.5, // proportional gain (kP)
                                              0.01, // integral gain (kI)
                                              8.5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              7// maximum acceleration (slew)
);


// angular PID controller
lemlib::ControllerSettings angular_controller(2.53, // proportional gain (kP)     //good values 2.53 kp , 0.00014 integral gain, 17.15 kD
                                              0.00014, // integral gain (kI)
                                              17.15, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);


// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);


// create the chassis


lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
                        //,&throttle_curve,
                        //&steer_curve
);



const int numStates = 3;
int states[numStates] = {0, -28, -149}; // or try 0,300,2000
int currentState = 0;
int target = 0;

void nextState() {
    currentState += 1;
    if (currentState == 3) {
        currentState = 0;
    }
    target = states[currentState];
}

void nextStateAuton() {
    currentState+=2;
    target = states[currentState];

}



void liftControl() {
    double kp = 3.5;   //3.5 is a good value
    //double error = target - rotation_sensor.get_position();
    //double velocity = kp * error;
    // ladyBrown.move(velocity);
    LadyBrown.move(kp * (target - (rotationSensor.get_position()/100.0)));
}





















void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    rotationSensor.reset_position();

    pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });

    pros::lcd::initialize();
    chassis.calibrate();
    pros::lcd::set_text(1, "Hello PROS User!");


    pros::lcd::register_btn1_cb(on_center_button);


    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

            
            // delay to save resources
            pros::delay(20);
        }
    });
}






   


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


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

static void brake() {
    left_motors.brake();
    right_motors.brake();
}

static void brakein() {
    Intake.brake();
    
}

static void in() {
    Intake.move_velocity(600);
    
}

double wrapHeading(double heading) {

        while (heading >= 360) heading -= 360;
        while (heading < 0) heading += 360;
        return heading;
}




void autonomous() {

    



    chassis.setPose(0, 0, 0);
    // move 48" forwards
    chassis.moveToPoint(0, 12, 10000,{.maxSpeed = 30});

    pros::delay(500);

    nextStateAuton();

    pros::delay(500);


    nextState();

    //LadyBrown.move(-126);

    //pros::delay(600);

    //LadyBrown.brake();

    //LadyBrown.move(126);

    pros::delay(500);

    //LadyBrown.brake();

    
    chassis.moveToPoint(0,-30,10000,{.forwards=false,.maxSpeed=100});

    chassis.waitUntilDone();

    pros::delay(500);

    clamp_mech.toggle();

    chassis.turnToHeading(105,1000);
    
}
pros::Controller controller(pros::E_CONTROLLER_MASTER);


void opcontrol() {
    





    // loop forever
    while (true) {
        // get left y and right x positions


        
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 1 ; // multiply this by a constant to increase curvature
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 1; // multiply this by a constant to increase curvature


        // move the robot
        //lemlib::ExpoDriveCurve driveCurve(5, 12, 1.132); -> check what this does
        chassis.curvature(leftY, rightX); // curvature arcade double stick, for single stick change right x to left x
        // regular arcade chassis.arcade(leftY, rightX); - double stick, for single stick change right x to left x
        // tank chassis.tank(leftY, rightY); , and change right x to right y
        // throttle/ steer priority chassis.arcade(leftY, leftX, false, 0.75); and change right x to left x
       
       
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
           
            Intake.move_velocity(600);


            


        }


        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            Intake.move_velocity(-600);


            
        }


        else {
            Intake.brake();


            
        }


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            clamp_mech.toggle();  
        }
        

        

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            doinker.toggle();
        }


        if (controller.get_digital_new_press(DIGITAL_Y)) {
            nextState();
        }   

        


        


        
    }


        //if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            //lift.toggle();
        //}


        //else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            //lift.toggle();
        //}




       


       
       








        //lemlib::ExpoDriveCurve driveCurve(5, 12, 1.132); -> check what this does
        //lemlib::ExpoDriveCurve driveCurve(5, 12, 1.132);
        // input 4
        //std::cout << driveCurve.curve(4) << std::endl; // outputs 0 because input is within deadzone
    //  input 6
        //std::cout << driveCurve.curve(6) << std::endl; // outputs 12, the minimum output
        //std::cout << driveCurve.curve(127) << std::endl; // outputs 127, the maximum output
       


       
       


        // delay to save resources
        pros::delay(25);
    
}


















