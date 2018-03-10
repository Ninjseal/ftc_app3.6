package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Alin on 27.01.2018,m
 */

@TeleOp(name = "ExtensonTest", group = "TeleOp")
//@Disabled
public class ExtensionTest extends LinearOpMode {

    //Servos
    private Servo servoExtension = null;

    //Constants
    private static final double EXTENSION_UP = 0.0;
    private static final double EXTENSION_MID = 0.5;
    private static final double EXTENSION_DOWN = 1.0;

    //@Override
    public void runOpMode() throws InterruptedException {

        // Map the servos
        servoExtension = hardwareMap.servo.get("extension");

        // Set servo directions
        servoExtension.setDirection(Servo.Direction.FORWARD);


        waitForStart();

        while (opModeIsActive()) {

            //Servo Extension Mechanism
            if (gamepad1.a) {
                servoExtension.setPosition(EXTENSION_DOWN);  // Servo Extension DOWN position
            } else if (gamepad1.b) {
                servoExtension.setPosition(EXTENSION_MID);  // Servo Extension MID position
            } else {
                servoExtension.setPosition(EXTENSION_UP);
            }
        }
    }
}