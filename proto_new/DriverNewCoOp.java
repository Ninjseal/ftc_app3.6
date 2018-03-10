package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Alin on 12.23.2017
 */

@TeleOp(name = "DriverCoOp", group = "TeleOp")
//@Disabled
public class DriverNewCoOp extends LinearOpMode {

    // Motors
    private DcMotor relicLMotor = null;
    private DcMotor relicRMotor = null;
    private DcMotor cubesMotor = null;
    private DcMotor leftMotorF = null;
    private DcMotor leftMotorB = null;
    private DcMotor rightMotorF = null;
    private DcMotor rightMotorB = null;
    //Servos
    private Servo servoClaw = null;
    private Servo servoExtension  = null;
    private Servo servoArm = null;
    private Servo servoColor = null;
    private Servo servoCubesLeft = null;
    private Servo servoCubesRight = null;
    //Constants
    private static final double CLAW_UP = 0.5;
    private static final double CLAW_DOWN = 0.0;
    private static final double EXTENSION_UP = 0.0;
    private static final double EXTENSION_MID = 0.5;
    private static final double EXTENSION_DOWN = 1.0;
    private static final double ARM_UP = 0.96;
    private static final double ARM_DOWN = 0.25;
    private static final double COLOR_FORWARD = 0.0;
    private static final double COLOR_BACK = 1.0;
    private static final double MID_SERVO = 0.5;
    private static final double CUBES_MIN = 0.65;
    private static final double CUBES_MAX = 0.8;
    // Additional helper variables
    private double clipValue = 0.9;
    private double leftWheelsPower = 0, rightWheelsPower = 0;
    private double deadzone = 0.1;

    private double relicPower = 1;
    private double relicDeadzone = 0.5;

    private double cubesPower = 1;

    //private boolean switchServoCubesLeft = false;
    //private boolean switchServoCubesRight = false;
    private boolean switchServoCubes = false;

    //@Override
    public void runOpMode() throws InterruptedException {
        // Map the motors
        relicLMotor = hardwareMap.dcMotor.get("relic_left");
        relicRMotor = hardwareMap.dcMotor.get("relic_right");
        cubesMotor = hardwareMap.dcMotor.get("cubes");
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");
        // Map the servos
        servoClaw = hardwareMap.servo.get("claw");
        servoExtension = hardwareMap.servo.get("extension");
        servoArm = hardwareMap.servo.get("arm");
        servoColor = hardwareMap.servo.get("colorS");
        servoCubesLeft = hardwareMap.servo.get("cubes_left");
        servoCubesRight = hardwareMap.servo.get("cubes_right");
        // Set wheel motor directions
        leftMotorF.setDirection(DcMotor.Direction.FORWARD);
        leftMotorB.setDirection(DcMotor.Direction.FORWARD);
        rightMotorF.setDirection(DcMotor.Direction.REVERSE);
        rightMotorB.setDirection(DcMotor.Direction.REVERSE);
        // Set the stopping method for wheels
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Set the relic mechanism direction
        relicLMotor.setDirection(DcMotor.Direction.FORWARD);
        relicRMotor.setDirection(DcMotor.Direction.REVERSE);
        // Set the cubes mechanism direction
        cubesMotor.setDirection(DcMotor.Direction.FORWARD);
        // Set servo directions
        servoClaw.setDirection(Servo.Direction.REVERSE );
        servoExtension.setDirection(Servo.Direction.FORWARD);
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoColor.setDirection(Servo.Direction.FORWARD);
        servoCubesLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesRight.setDirection(Servo.Direction.REVERSE);
        // Set the motors power to 0
        relicLMotor.setPower(0);
        relicRMotor.setPower(0);
        cubesMotor.setPower(0);
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        // Initialize servo positions
        servoClaw.setPosition(CLAW_DOWN);
        servoExtension.setPosition(EXTENSION_UP);
        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(COLOR_BACK);
        servoCubesRight.setPosition(CUBES_MIN);
        servoCubesLeft.setPosition(CUBES_MIN);

        telemetry.addData("Say", "Hello Driver!");
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to run the motors.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Driving Mechanism
            // Update joystick value
            leftWheelsPower = -gamepad1.left_stick_y;
            rightWheelsPower = -gamepad1.right_stick_y;

            // Check the deadzone
            if (Math.abs(leftWheelsPower) < deadzone) leftWheelsPower = 0;
            if (Math.abs(rightWheelsPower) < deadzone) rightWheelsPower = 0;

            if(leftWheelsPower > clipValue) {
                leftWheelsPower = clipValue;
            }
            if(rightWheelsPower > clipValue) {
                rightWheelsPower = clipValue;
            }

            if(leftWheelsPower < -clipValue ) {
                leftWheelsPower = -clipValue;
            }
            if(rightWheelsPower < -clipValue) {
                rightWheelsPower = -clipValue;
            }

            leftMotorF.setPower(leftWheelsPower);
            leftMotorB.setPower(leftWheelsPower);
            rightMotorF.setPower(rightWheelsPower);
            rightMotorB.setPower(rightWheelsPower);


            // Servo Cubes Mechanism
            /*if(gamepad1.left_trigger != 0) {
                 servoCubesLeft.setPosition(CUBES_MAX);
                switchServoCubesLeft = true;
            } else if(switchServoCubesLeft) {
                servoCubesLeft.setPosition(CUBES_MIN);
                switchServoCubesLeft = false;
            }

            if(gamepad1.right_trigger != 0) {
                servoCubesRight.setPosition(CUBES_MAX);
                switchServoCubesRight = true;
            } else if(switchServoCubesRight) {
                servoCubesRight.setPosition(CUBES_MIN);
                switchServoCubesRight = false;
            }*/

            if (gamepad2.a && !switchServoCubes) {
                servoCubesRight.setPosition(CUBES_MAX);
                servoCubesLeft.setPosition(CUBES_MAX);
                switchServoCubes = true;
            }
            if (gamepad2.b && switchServoCubes) {
                servoCubesRight.setPosition(CUBES_MIN);
                servoCubesLeft.setPosition(CUBES_MIN);
                switchServoCubes = false;
            }

           /*if(gamepad2.x){
                servoCubesRight.setPosition(0.55);
                servoCubesLeft.setPosition(0.55);
            }*/

            // Cubes mechanism
            if (gamepad2.dpad_up) {
                cubesMotor.setPower(cubesPower);
            } else if (gamepad2.dpad_down) {
                cubesMotor.setPower(-cubesPower);
            } else {
                cubesMotor.setPower(0);  // Stop the motor
            }

            // Relic mechanism right Motor

           /* if (relicPower == 1) {
                relicRPower = -gamepad2.right_stick_y;
            }

                if (Math.abs(relicRPower) < deadzone) relicRPower = 0;

                if(relicRPower > relicPower) {
                    relicRPower = relicPower;
                }
                if(relicRPower < -relicPower) {
                    relicRPower = -relicPower;
                }

            if (gamepad2.dpad_up) {
                relicRMotor.setPower(relicPower);
            } else if (gamepad1.dpad_down) {
                relicRMotor.setPower(-relicPower);
            } else{
                relicRMotor.setPower(0);  // Stop the motor
            }
            */

            if(gamepad2.right_stick_y > relicDeadzone) {
                relicRMotor.setPower(relicPower);
            } else if(gamepad2.right_stick_y < -relicDeadzone) {
                relicRMotor.setPower(-relicPower);
            } else relicRMotor.setPower(0);


            //Relic mechanism left Motor

            if(gamepad2.left_stick_y > relicDeadzone) {
                relicLMotor.setPower(relicPower);
            } else if(gamepad2.left_stick_y < -relicDeadzone) {
                relicLMotor.setPower(-relicPower);
            } else relicLMotor.setPower(0);


            /*if (relicPower == 1) {
                relicLPower = -gamepad2.left_stick_y;
            }

                if (Math.abs(relicLPower) < deadzone) relicLPower = 0;

                if(relicLPower > relicPower) {
                    relicLPower = relicPower;
                }
                if(relicLPower < -relicPower) {
                    relicLPower = -relicPower;
                }


            if (gamepad2.dpad_right) {
                relicLMotor.setPower(relicPower);
            } else if (gamepad2.dpad_left) {
                relicLMotor.setPower(-relicPower);
            } else{
                relicLMotor.setPower(0);   // Stop the motor
            } */

             //Servo Extension Mechanism
            if (gamepad2.x) {
                servoExtension.setPosition(1.0);  // Servo Extension DOWN position
            } else if (gamepad2.y) {
                servoExtension.setPosition(0.0);  // Servo Extension MID position
            } else {
                servoExtension.setPosition(0.4);
            }

            // Servo Claw Mechanism
            if (gamepad1.a) {
                servoClaw.setPosition(CLAW_UP);  // Servo Claw UP position
            } else {
                servoClaw.setPosition(CLAW_DOWN);
            }

            /*// Servo Arm Mechanism
            if  (gamepad2.a) {
                servoArm.setPosition(ARM_DOWN);  // Servo Arm DOWN position
            } else{
                servoArm.setPosition(ARM_UP);  // Servo Arm UP position
            }*/
            /*// Servo Color Mechanism
            if (gamepad2.x) //(Blue){
                servoColor.setPosition(COLOR_RIGHT);  // Servo Color RIGHT position
             else if (gamepad2.y) //(Red){
                servoColor.setPosition(COLOR_LEFT);  // Servo Color LEFT position
             else{ servoColor.setPosition(MID_SERVO);}*/

            // Prepare robot for end-game
            if (gamepad1.x) {
                clipValue = 0.45;  // Set the clip value to .45
            } else if (gamepad1.y) {
                clipValue = 0.9;  // Set the clip value to .9
            }

            // Telemetry Data
            telemetry.addData("leftWheelsPower", leftWheelsPower);
            telemetry.addData("rightWheelsPower", rightWheelsPower);
            telemetry.addData("" + ">", "Press Stop to end test.");
            telemetry.update();
            idle();

        }
        stopMotors();  // Stop all the motors
    }

    // This function stops all the motors of the robot
    public void stopMotors(){
        relicLMotor.setPower(0);
        relicRMotor.setPower(0);
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        cubesMotor.setPower(0);
    }
    //StopMotors

}