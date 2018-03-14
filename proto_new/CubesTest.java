package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Alin on 27.01.2018
 */

@TeleOp(name = "CubesGrabTest", group = "TeleOp")
//@Disabled
public class CubesTest extends LinearOpMode {

    //Servos
    private Servo servoCubesDownLeft = null;
    private Servo servoCubesDownRight = null;
    private Servo servoCubesUpLeft = null;
    private Servo servoCubesUpRight = null;

    //Constants
    private static final double CUBES_RELEASE = 0.65;
    private static final double CUBES_CATCH = 0.8;

    //Helpers
    private boolean switchServoCubesUp = false;
    private boolean switchServoCubesDown = false;

    //@Override
    public void runOpMode() throws InterruptedException {
        // Map the servos
        servoCubesDownLeft = hardwareMap.servo.get("cubes_down_left");
        servoCubesDownRight = hardwareMap.servo.get("cubes_down_right");
        servoCubesUpLeft = hardwareMap.servo.get("cubes_up_left");
        servoCubesUpRight = hardwareMap.servo.get("cubes_up_right");

        // Set servo directions
        servoCubesDownLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesDownRight.setDirection(Servo.Direction.REVERSE);
        servoCubesUpLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesUpRight.setDirection(Servo.Direction.REVERSE);

        // Initialize servo positions
        servoCubesDownLeft.setPosition(CUBES_RELEASE);
        servoCubesDownRight.setPosition(CUBES_RELEASE);
        servoCubesUpLeft.setPosition(CUBES_RELEASE);
        servoCubesUpRight.setPosition(CUBES_RELEASE);

        telemetry.addData(">", "Press Start to run the motors.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
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
                servoCubesUpLeft.setPosition(CUBES_CATCH);
                servoCubesUpRight.setPosition(CUBES_CATCH);
                switchServoCubesUp = true;
            }
            if (gamepad2.y && switchServoCubesUp) {
                servoCubesUpLeft.setPosition(CUBES_RELEASE);
                servoCubesUpRight.setPosition(CUBES_RELEASE);
                switchServoCubesUp = false;
            }

            telemetry.addData("CubesUpLeft: ", servoCubesUpLeft.getPosition());
            telemetry.addData("CubesUpRight: ", servoCubesUpRight.getPosition());
            telemetry.addData("CubesDownLeft: ", servoCubesDownLeft.getPosition());
            telemetry.addData("CubesDownRight: ", servoCubesDownRight.getPosition());
            telemetry.update();
            idle();

        }
    }
}