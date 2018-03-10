package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Alin on 27.01.2018,m
 */

@TeleOp(name = "ClawTest", group = "TeleOp")
//@Disabled
public class ClawTest extends LinearOpMode {

    //Servos
    private Servo servoClaw = null;

    //Constants
    private static final double CLAW_UP = 0.5;
    private static final double CLAW_DOWN = 0.0;


    //@Override
    public void runOpMode() throws InterruptedException {
        // Map the servos
        servoClaw = hardwareMap.servo.get("claw");

        // Set servo directions
        servoClaw.setDirection(Servo.Direction.FORWARD);

        // Initialize servo positions
        servoClaw.setPosition(CLAW_DOWN);


        waitForStart();

        while (opModeIsActive()) {


            // Servo Claw Mechanism
            if (gamepad2.x) {
                servoClaw.setPosition(CLAW_UP);  // Servo Claw UP position
            } else {
                servoClaw.setPosition(CLAW_DOWN);
            }

        }
    }
}