package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Alin on 18/01/2018
 */

@TeleOp(name = "DriverMotorsTest", group = "TeleOp")
//@Disabled
public class MotorsTest extends LinearOpMode {

    // Motors
    private DcMotor leftMotorF = null;
    private DcMotor leftMotorB = null;
    private DcMotor rightMotorF = null;
    private DcMotor rightMotorB = null;
    // Additional helper variables
    private double clipValue = 0.9;
    private double leftWheelsPower = 0, rightWheelsPower = 0;
    private double deadzone = 0.1;
    @Override
    public void runOpMode () throws InterruptedException {
        // Map the motors
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");
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
        // Set the motors power to 0
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

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
            rightMotorB.setPower(rightWheelsPower);
            rightMotorF.setPower(rightWheelsPower);

            // Prepare robot for end-game
            if (gamepad1.x) {
                clipValue = 0.3;  // Set the clip value to .3
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
    public void stopMotors() {
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
    }
    //StopMotors

}

