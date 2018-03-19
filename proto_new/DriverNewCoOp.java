package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Robo-Sapiens on 23/03/2018.
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
    private Servo servoExtension = null;
    private Servo servoArm = null;
    private Servo servoColor = null;
    private Servo servoCubesDownLeft = null;
    private Servo servoCubesDownRight = null;
    private Servo servoCubesUpLeft = null;
    private Servo servoCubesUpRight = null;
    //Constants
    private static final double CLAW_UP = 0.0;
    private static final double CLAW_DOWN = 0.9;
    private static final double EXTENSION_UP = 0.0;
    private static final double EXTENSION_MID = 0.5;
    private static final double EXTENSION_DOWN = 1.0;
	private static final double EXTENSION_SPEED = 0.02;
    private static final double ARM_UP = 0.96;
    private static final double COLOR_FORWARD = 0.0;
    private static final double COLOR_BACK = 1.0;
    private static final double MID_SERVO = 0.5;
    private static final double CUBES_RELEASE = 0.6;
    private static final double CUBES_CATCH = 0.8;
    // Additional helper variables
    private double clipValue = 0.9;
    private double leftWheelsPower = 0, rightWheelsPower = 0;
    private double deadzone = 0.1;

    private double relicPower = 1;
    private double relicDeadzone = 0.5;

    private double cubesPower = 1;

    private double extensionPosition = 0.5;

    private boolean switchServoCubesUp = false;
    private boolean switchServoCubesDown = false;
    private boolean relic_mode = false;

    protected ElapsedTime runtime = new ElapsedTime();

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
        servoCubesDownLeft = hardwareMap.servo.get("cubes_down_left");
        servoCubesDownRight = hardwareMap.servo.get("cubes_down_right");
        servoCubesUpLeft = hardwareMap.servo.get("cubes_up_left");
        servoCubesUpRight = hardwareMap.servo.get("cubes_up_right");
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
		cubesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        //Set the relic mechanism direction
        relicLMotor.setDirection(DcMotor.Direction.FORWARD);
        relicRMotor.setDirection(DcMotor.Direction.REVERSE);
        // Set the cubes mechanism direction
        cubesMotor.setDirection(DcMotor.Direction.FORWARD);
        // Set servo directions
        servoClaw.setDirection(Servo.Direction.REVERSE);
        servoExtension.setDirection(Servo.Direction.FORWARD);
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoColor.setDirection(Servo.Direction.FORWARD);
        servoCubesDownLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesDownRight.setDirection(Servo.Direction.REVERSE);
        servoCubesUpLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesUpRight.setDirection(Servo.Direction.REVERSE);
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
        servoExtension.setPosition(EXTENSION_MID);
        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(COLOR_BACK);
        servoCubesDownLeft.setPosition(CUBES_CATCH);
        servoCubesDownRight.setPosition(CUBES_CATCH);
        servoCubesUpLeft.setPosition(CUBES_CATCH);
        servoCubesUpRight.setPosition(CUBES_CATCH);
		
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to run the motors.");
        telemetry.update();

        waitForStart();

        // Start timer for relic switch mode
        runtime.reset();

        while (opModeIsActive()) {
            /*** GAMEPAD 1 CODE ***/

            // Driving Mechanism
            // Update joystick value
            leftWheelsPower = -gamepad1.left_stick_y;
            rightWheelsPower = -gamepad1.right_stick_y;

            // Check the deadzone
            if (Math.abs(leftWheelsPower) < deadzone) leftWheelsPower = 0;
            if (Math.abs(rightWheelsPower) < deadzone) rightWheelsPower = 0;

            if (leftWheelsPower > clipValue) {
                leftWheelsPower = clipValue;
            }
            if (rightWheelsPower > clipValue) {
                rightWheelsPower = clipValue;
            }

            if (leftWheelsPower < -clipValue) {
                leftWheelsPower = -clipValue;
            }
            if (rightWheelsPower < -clipValue) {
                rightWheelsPower = -clipValue;
            }

            leftMotorF.setPower(leftWheelsPower);
            leftMotorB.setPower(leftWheelsPower);
            rightMotorF.setPower(rightWheelsPower);
            rightMotorB.setPower(rightWheelsPower);

            // Adjust robot's wheels speed
            if (gamepad1.x) {
                clipValue = 0.45;  // Set the clip value to .45
            } else if (gamepad1.y) {
                clipValue = 0.9;  // Set the clip value to .9
            }

            /*** GAMEPAD 2 CODE ***/

            // Switch Mode mechanism
            if ((gamepad2.left_bumper) && (runtime.seconds() > 1.5)) {
                runtime.reset();
                relic_mode = !relic_mode;
            }

            // Relic Mode
            if (relic_mode) {
                // Relic mechanism right Motor
                if (gamepad2.right_stick_y > relicDeadzone) {
                    relicRMotor.setPower(relicPower);
                } else if (gamepad2.right_stick_y < -relicDeadzone) {
                    relicRMotor.setPower(-relicPower);
                } else relicRMotor.setPower(0);

                //Relic mechanism left Motor
                if (gamepad2.left_stick_y > relicDeadzone) {
                    relicLMotor.setPower(relicPower);
                } else if (gamepad2.left_stick_y < -relicDeadzone) {
                    relicLMotor.setPower(-relicPower);
                } else relicLMotor.setPower(0);

                //Servo Extension Mechanism
                if (gamepad2.x) {
                    extensionPosition += EXTENSION_SPEED;
                    extensionPosition = Range.clip(extensionPosition, EXTENSION_UP, EXTENSION_DOWN);
                    //servoExtension.setPosition(EXTENSION_DOWN);  // Servo Extension DOWN position
                } else if (gamepad2.y) {
                    extensionPosition -= EXTENSION_SPEED;
                    extensionPosition = Range.clip(extensionPosition, EXTENSION_UP, EXTENSION_DOWN);
                    //servoExtension.setPosition(EXTENSION_UP);  // Servo Extension UP position
                } else
                    extensionPosition = EXTENSION_MID;
                // Servo Claw Mechanism
                if (gamepad2.a) {
                    servoClaw.setPosition(CLAW_DOWN);  // Strange ghiara / Prinde relicva
                } else if (gamepad2.b) {
                    servoClaw.setPosition(CLAW_UP);   // Ridica ghiara / Elibereaza relicva
                }

                servoExtension.setPosition(extensionPosition);  // Servo Extension position
            }

            // Normal Mode
            else {
                // Servo Cubes Mechanism
                if (gamepad2.a && !switchServoCubesDown) {
                    servoCubesDownLeft.setPosition(CUBES_CATCH);
                    servoCubesDownRight.setPosition(CUBES_CATCH);
                    switchServoCubesDown = true;
                }
                if (gamepad2.b && switchServoCubesDown) {
                    servoCubesDownLeft.setPosition(CUBES_RELEASE);
                    servoCubesDownRight.setPosition(CUBES_RELEASE);
                    switchServoCubesDown = false;
                }
                if (gamepad2.x && !switchServoCubesUp) {
                    servoCubesUpLeft.setPosition(CUBES_CATCH-0.05);
                    servoCubesUpRight.setPosition(CUBES_CATCH-0.05);
                    switchServoCubesUp = true;
                }
                if (gamepad2.y && switchServoCubesUp) {
                    servoCubesUpLeft.setPosition(CUBES_RELEASE);
                    servoCubesUpRight.setPosition(CUBES_RELEASE);
                    switchServoCubesUp = false;
                }

                // Cubes lifting mechanism
                if (gamepad2.dpad_up) {
                    cubesMotor.setPower(cubesPower);
                } else if (gamepad2.dpad_down) {
                    cubesMotor.setPower(-cubesPower);
                } else {
                    cubesMotor.setPower(0);  // Stop the motor
                }
            }

			// Telemetry Data
			telemetry.addData("RelicMode", relic_mode);
			telemetry.addData("leftWheelsPower", leftWheelsPower);
			telemetry.addData("rightWheelsPower", rightWheelsPower);
			telemetry.update();
			sleep(20);
			idle();
		}
		
		stopMotors();  // Stop all the motors
    }

	// This function stops all the motors of the robot
    public void stopMotors() {
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
