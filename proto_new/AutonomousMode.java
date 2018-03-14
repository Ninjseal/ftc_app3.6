package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Purplecoder 27/01/2018.
 */

public abstract class AutonomousMode extends LinearOpMode {
    // Motors
    protected DcMotor cubesMotor = null;
    protected DcMotor leftMotorF = null;
    protected DcMotor leftMotorB = null;
    protected DcMotor rightMotorF = null;
    protected DcMotor rightMotorB = null;

    // Servos
    protected Servo servoArm = null;
    protected Servo servoColor = null;
    protected Servo servoCubesDownLeft = null;
    protected Servo servoCubesDownRight = null;
    protected Servo servoCubesUpLeft = null;
    protected Servo servoCubesUpRight = null;

    // Sensors
    protected ModernRoboticsI2cGyro gyroSensor = null;
    protected ColorSensor colorSensor = null;

    // Constants
    protected static final double ARM_UP = 0.96;
    protected static final double ARM_DOWN = 0.25;
    protected static final double COLOR_FORWARD = 0.0;
    protected static final double COLOR_BACK = 1.0;
    protected static final double MID_SERVO = 0.5;
    protected static final double COLOR_INIT = 0.85;
    protected static final double CUBES_RELEASE = 0.65;
    protected static final double CUBES_CATCH = 0.8;
    protected static final double LIFT_MAX = 5000;

    protected ElapsedTime runtime = new ElapsedTime();

    //Vuforia
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        while (!isStopRequested() && gyroSensor.isCalibrating() && opModeIsActive()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Gyro: ", gyroSensor.getHeading());
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

    protected void initHardware() {
        // Map the motors
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");
        cubesMotor = hardwareMap.dcMotor.get("cubes");
        // Map the servos
        servoArm = hardwareMap.servo.get("arm");
        servoColor = hardwareMap.servo.get("colorS");
        servoCubesDownLeft = hardwareMap.servo.get("cubes_down_left");
        servoCubesDownRight = hardwareMap.servo.get("cubes_down_right");
        servoCubesUpLeft = hardwareMap.servo.get("cubes_up_left");
        servoCubesUpRight = hardwareMap.servo.get("cubes_up_right");
        // Map the sensors
        gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        // Set the wheel motors
        leftMotorF.setDirection(DcMotor.Direction.FORWARD);
        leftMotorB.setDirection(DcMotor.Direction.FORWARD);
        rightMotorF.setDirection(DcMotor.Direction.REVERSE);
        rightMotorB.setDirection(DcMotor.Direction.REVERSE);
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
        servoCubesDownLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesDownRight.setDirection(Servo.Direction.REVERSE);
        servoCubesUpLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesUpRight.setDirection(Servo.Direction.REVERSE);
        // Set the motors power to 0
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        cubesMotor.setPower(0);
        // Initialize servo positions
        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(COLOR_INIT);
        //servoCubesDownLeft.setPosition(CUBES_RELEASE);
        //servoCubesDownRight.setPosition(CUBES_RELEASE);
        //servoCubesUpLeft.setPosition(CUBES_RELEASE);
        //servoCubesUpRight.setPosition(CUBES_RELEASE);

        // Calibrate sensors
        colorSensor.enableLed(true);
        gyroSensor.calibrate();
    }

    // Wait for a number of seconds
    protected void wait(double seconds) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            //telemetry.addData("Time Elapsed: ", "%2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
            idle();
        }
    }
    //Wait

    // Autonomous ball selection
    protected void ball_auto(boolean red_team){
        servoArm.setPosition(MID_SERVO);
        wait(0.5);

        servoColor.setPosition(MID_SERVO);
        wait(0.5);

        servoArm.setPosition(ARM_DOWN);
        wait(1.0);

        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();

        telemetry.addData("Red  ", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue ", blue);
        telemetry.update();

        if(red_team) {
            if(red > blue && red > green && opModeIsActive()){
                servoColor.setPosition(COLOR_BACK);
            } else if(blue > red && blue > green && opModeIsActive()){
                servoColor.setPosition(COLOR_FORWARD);
            }
        } else if(!red_team) {
            if (red > blue && red > green && opModeIsActive()) {
                servoColor.setPosition(COLOR_FORWARD);
            } else if (blue > red && blue > green && opModeIsActive()) {
                servoColor.setPosition(COLOR_BACK);
            }
        }

        wait(0.5);
        servoArm.setPosition(MID_SERVO);
        wait(0.5);
        servoColor.setPosition(COLOR_INIT);
        wait(0.5);
        servoArm.setPosition(ARM_UP);
        wait(0.5);
        servoColor.setPosition(COLOR_BACK);
    }
    //AutonomousBall

    // Lift the cubes to the target specified with a desired power
    protected void cubes_to_position(int power, int target){
        target = (int)Range.clip(target, 0, LIFT_MAX);

        cubesMotor.setTargetPosition(target);
        cubesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cubesMotor.setPower(Range.clip(power, -1,1));

        while(cubesMotor.isBusy() && opModeIsActive()){
            idle();
        }

        cubesMotor.setPower(0);
        cubesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //CubesToPosition

    // Toggle function for grabbing the cube
    protected void grab_cube(boolean grab){
        if(grab){
            servoCubesUpLeft.setPosition(CUBES_CATCH);
            servoCubesUpRight.setPosition(CUBES_CATCH);
            servoCubesDownLeft.setPosition(CUBES_CATCH);
            servoCubesDownRight.setPosition(CUBES_CATCH);
        } else {
            servoCubesUpLeft.setPosition(CUBES_RELEASE);
            servoCubesUpRight.setPosition(CUBES_RELEASE);
            servoCubesDownLeft.setPosition(CUBES_RELEASE);
            servoCubesDownRight.setPosition(CUBES_RELEASE);
        }
    }
    //GrabCube

    // Function for turning with angle amount in a direction(trigo = 1 for right and trigo = -1 for left)
    protected void gyro_turn(int angle, int direction) {
        double speed = 0.4;
        double mspeed = -0.04;
        //double fall_speed = 0.00015;

        if(direction == 1) // Rotatie spre dreapta
        {
            gyroSensor.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
            int curr = gyroSensor.getHeading();

            telemetry.addData("heading", "%3d deg", curr);
            telemetry.update();

            power_wheels(speed, speed, -speed, -speed);

            //runtime.reset();
            while(opModeIsActive() && (curr < angle)) {
                power_wheels(speed, speed, -speed, -speed);
                curr = gyroSensor.getHeading();
                telemetry.addData("heading", "%3d deg", curr);
                telemetry.update();

                if(curr > angle-10)
                    speed = 0.05;

                //if(speed > 0.05)
                    //speed -= fall_speed;
            }

            // Daca am depasit valoarea unghiului
            while(opModeIsActive() && (curr > angle)) {
                power_wheels(mspeed, mspeed, -mspeed, -mspeed);
                curr = gyroSensor.getHeading();
                telemetry.addData("heading", "%3d deg", curr);
                telemetry.update();
            }

            stopWheels();

            telemetry.addData("heading", "%3d deg", gyroSensor.getHeading());
            telemetry.update();
        }
        else if (direction == -1){  // Rotatie spre stanga
            gyroSensor.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
            int curr = gyroSensor.getHeading();

            telemetry.addData("heading", "%3d deg", curr);
            telemetry.update();

            //runtime.reset();
            while(opModeIsActive() && (curr < angle)) {
                power_wheels(-speed, -speed, speed, speed);
                curr = gyroSensor.getHeading();
                telemetry.addData("heading", "%3d deg", curr);
                telemetry.update();

                if(curr > angle-10)
                    speed = 0.05;

                //if(speed > 0.05)
                    //speed -= fall_speed;
            }

            // Daca a depasit valoarea unghiului
            while(opModeIsActive() && (curr > angle)) {
                power_wheels(-mspeed, -mspeed, mspeed, mspeed);
                curr = gyroSensor.getHeading();
                telemetry.addData("heading", "%3d deg", curr);
                telemetry.update();
            }

            stopWheels();

            telemetry.addData("heading", "%3d deg", gyroSensor.getHeading());
            telemetry.update();
        }
        wait(0.5);
    }
    //GyroTurn

    // Move forward to a target using encoders
    protected void move_with_encoders(int target, int direction, double power) {
        leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotorF.setTargetPosition(direction*target);
        leftMotorB.setTargetPosition(direction*target);
        rightMotorF.setTargetPosition(direction*target);
        rightMotorB.setTargetPosition(direction*target);

        power_wheels(power, power, power, power);

        while(opModeIsActive() && (leftMotorB.isBusy() || leftMotorF.isBusy() ||
                rightMotorB.isBusy() || rightMotorF.isBusy())){
            idle(); // Wait
        }
        stopWheels();

        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wait(0.5);
    }
    //MoveWithEncoders

    // Vuforia image recognition
    protected int activate_vuforia()  {
        int rez = 0, cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcO14uD/////AAAAmUILEtDJVkPbjdTr7NZGRE1UskK3HPIjKtoC7tONYCaYenUf9sRjducLbUEn0Cu8Eh3lrEVqM7VbF5MXcNfFyS39uTN3t7PtjK8HLcFpFsgTLRFwAGJlhcX+OFgqsjzPSiyE8v7Z+XIYwMhKf2Z2XmTQOCa6vXLL30nw3iLnE6J2Q5QFnNw/+AFLA881KCVYSeGBtujTRvfloxYCMYon30C1uwWB0txP4s7K1FukBiyfKScQFj7CwS+27BsSajo8lstaPwlSw5LssYO0cEbNQmi31q1meclqCkTL0nRVZcdj+UrfutQms0Ledjs6N8+bQg/qxo//KsFZ7pGZsCO3HIyrVKTEadLDzg+3KV7EJhNV";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 1.5)){

            if(runtime.seconds() > 1.5)
                break;

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            switch(vuMark) {
                case LEFT:
                    rez = 1;
                    break;

                case CENTER:
                    rez = 2;
                    break;

                case RIGHT:
                    rez = 3;
                    break;

                case UNKNOWN:
                    rez = 0;
                    break;
            }

            telemetry.addData("VuMark", "%d visible", rez);
            telemetry.update();
            //idle();
        }

        return rez;
    }
    //ActivateVuforia

    // Power the wheel motors with values given
    protected void power_wheels(double leftPowerF, double leftPowerB, double rightPowerF, double rightPowerB) {
        leftMotorF.setPower(Range.clip(leftPowerF, -1, 1));
        leftMotorB.setPower(Range.clip(leftPowerB, -1, 1));
        rightMotorF.setPower(Range.clip(rightPowerF, -1, 1));
        rightMotorB.setPower(Range.clip(rightPowerB, -1, 1));
    }
    //PowerWheels

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