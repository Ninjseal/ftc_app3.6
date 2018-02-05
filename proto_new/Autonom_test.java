package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Alin on 12/23/2017.
 */

@Autonomous(name = "Autonomous_test", group = "TeleOp")
//@Disabled
public class Autonom_test extends LinearOpMode {

    // Motors
    private DcMotor cubesMotor = null;
    private DcMotor leftMotorF = null;
    private DcMotor leftMotorB = null;
    private DcMotor rightMotorF = null;
    private DcMotor rightMotorB = null;
    //Servos
    private Servo servoArm = null;
    private Servo servoColor = null;
    private Servo servoCubesLeft = null;
    private Servo servoCubesRight = null;
    //Constants
    private static final double ARM_UP = 0.96;
    private static final double ARM_DOWN = 0.3;
    private static final double COLOR_LEFT = 0.0;
    private static final double COLOR_RIGHT = 0.9;
    private static final double MID_SERVO = 0.5;
    private static final double CUBES_MIN = 0.15;
    private static final double CUBES_MAX = 0.35;
    // Additional helper variables
    private double leftWheelsPower = 0, rightWheelsPower = 0;
    private double deadzone = 0.1;

    private double cubesPower = 1;

    //private boolean switchServoCubesLeft = false;
    //private boolean switchServoCubesRight = false;
    private boolean switchServoCubes = false;

    //@Override
    public void runOpMode() throws InterruptedException {
        // Map the motors
        cubesMotor = hardwareMap.dcMotor.get("cubes");
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");
        // Map the servos
        servoArm = hardwareMap.servo.get("arm");
        servoColor = hardwareMap.servo.get("color");
        servoCubesLeft = hardwareMap.servo.get("cubes_left");
        servoCubesRight = hardwareMap.servo.get("cubes_right");
        // Set wheel motor directions
        leftMotorF.setDirection(DcMotor.Direction.REVERSE);
        leftMotorB.setDirection(DcMotor.Direction.REVERSE);
        rightMotorF.setDirection(DcMotor.Direction.FORWARD);
        rightMotorB.setDirection(DcMotor.Direction.FORWARD);
        // Set the stopping method for wheels
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set the cubes mechanism direction
        cubesMotor.setDirection(DcMotor.Direction.FORWARD);
        // Set servo directions
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoColor.setDirection(Servo.Direction.FORWARD);
        servoCubesLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesRight.setDirection(Servo.Direction.REVERSE);
        // Set the motors power to 0
        cubesMotor.setPower(0);
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        // Initialize servo positions
        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(MID_SERVO);
        servoCubesRight.setPosition(CUBES_MIN);
        servoCubesLeft.setPosition(CUBES_MIN);

        telemetry.addData("Say", "Hello Driver!");
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to run the motors.");
        telemetry.update();

        waitForStart();

        servoArm.setPosition(ARM_DOWN);

        sleep(2000);

        //servoArm.setPosition(0.5);

        //sleep(1000);

        servoColor.setPosition(1);
        sleep(1000);

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

