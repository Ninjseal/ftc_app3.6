package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Purplecoder 27/01/2018.
 */

public abstract class AutonomousMode extends LinearOpMode {

    // Motors
    private DcMotor cubesMotor = null;
    protected DcMotor leftMotorF = null;
    protected DcMotor leftMotorB = null;
    protected DcMotor rightMotorF = null;
    protected DcMotor rightMotorB = null;
    // Servos
    private Servo servoArm = null;
    private Servo servoColor = null;
    private Servo servoCubesLeft = null;
    private Servo servoCubesRight = null;

    // Sensors

    //protected ModernRoboticsI2cGyro gyroSensor = null;
    protected ColorSensor colorSensor = null;
    //protected OpticalDistanceSensor odsSensor = null;
    //protected ModernRoboticsI2cRangeSensor rangeSensor = null;


    // Constants
    private static final double ARM_UP = 0.96;
    private static final double ARM_DOWN = 0.25;
    private static final double COLOR_FORWARD = 0.0;
    private static final double COLOR_BACK = 1.0;
    private static final double MID_SERVO = 0.5;
    private static final double CUBES_MIN = 0.65;
    private static final double CUBES_MAX = 0.8;

    protected ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        while (!isStopRequested() && opModeIsActive()) {
            sleep(50);
            idle();
        }
        //telemetry.addData("Gyro: ", gyroSensor.getHeading());
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for Start button to be pressed
        waitForStart();
        try {
            runOp();
        } catch(Exception e) {
            e.printStackTrace();
        } finally {
            exitOpMode();
        }
    }

    protected abstract void initOpMode() throws InterruptedException;
    protected abstract void runOp() throws InterruptedException;
    protected abstract void exitOpMode() throws InterruptedException;

    public void initHardware() {
        // Map the motors
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");
        cubesMotor = hardwareMap.dcMotor.get("cubes");
        // Map the servos
        servoArm = hardwareMap.servo.get("arm");
        servoColor = hardwareMap.servo.get("color");
        servoCubesLeft = hardwareMap.servo.get("cubes_left");
        servoCubesRight = hardwareMap.servo.get("cubes_right");
        // Map the sensors
        //gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        //colorSensor = hardwareMap.colorSensor.get("color");
        // Set the wheel motors
        leftMotorF.setDirection(DcMotor.Direction.REVERSE);
        leftMotorB.setDirection(DcMotor.Direction.REVERSE);
        rightMotorF.setDirection(DcMotor.Direction.FORWARD);
        rightMotorB.setDirection(DcMotor.Direction.FORWARD);
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cubesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cubesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the stopping method for wheels
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cubesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set the cubes mechanism direction
        cubesMotor.setDirection(DcMotor.Direction.FORWARD);
        // Set servo directions
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoColor.setDirection(Servo.Direction.FORWARD);
        servoCubesLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesRight.setDirection(Servo.Direction.REVERSE);
        // Set the motors power to 0
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        // Initialize servo positions
        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(MID_SERVO);
        servoCubesRight.setPosition(CUBES_MAX);
        servoCubesLeft.setPosition(CUBES_MAX);
        
        // Calibrate sensors
        colorSensor.enableLed(true);
        //gyroSensor.calibrate();

       // while(gyroSensor.getHeading() != 0);

        wait(1.0);
    }

    // Wait for a number of seconds
    protected void wait(double seconds) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            //telemetry.addData("Time Elapsed: ", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }
    }
    //Wait

    protected void ball_auto(boolean red_team){

        servoColor.setPosition(MID_SERVO);
        wait(2.0);

        servoArm.setPosition(ARM_DOWN);
        wait(2.0);

        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();

        wait(1.0);

        telemetry.addData("Red  ", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue ", blue);
        telemetry.update();

        if(red_team) {
            if(red > blue && red > green){
                servoColor.setPosition(COLOR_BACK);
            } else if(blue > red && blue > green){
                servoColor.setPosition(COLOR_FORWARD);
            }
        } else if(!red_team) {
            if (red > blue && red > green) {
                servoColor.setPosition(COLOR_FORWARD);
            } else if (blue > red && blue > green) {
                servoColor.setPosition(COLOR_BACK);
            }
        }

        wait(3.0);

        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(COLOR_BACK);

    }

    // This function stops all the wheels of the robot
    protected void stopWheels() {
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
    }
    //StopWheels

    // This function stops all the motors of the robot
    protected void stopMotors() {
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        cubesMotor.setPower(0);
    }
    //StopMotors

}
