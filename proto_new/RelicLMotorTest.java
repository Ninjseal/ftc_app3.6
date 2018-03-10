package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Alin on 27.01.2018,m
 */

@TeleOp(name = "RelicLMotorTest", group = "TeleOp")
//@Disabled
public class RelicLMotorTest extends LinearOpMode {

    // Motors
    private DcMotor relicLMotor = null;


    private double relicPower = 1;


    //@Override
    public void runOpMode() throws InterruptedException {
        // Map the motors
        relicLMotor = hardwareMap.dcMotor.get("relic left");

        //Set the relic mechanism direction
        relicLMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set the motors power to 0
        relicLMotor.setPower(0);

        waitForStart();

        while (opModeIsActive()) {


            //Relic mechanism left Motor
            if (gamepad2.dpad_right) {
                relicLMotor.setPower(relicPower);
            } else if (gamepad2.dpad_left) {
                relicLMotor.setPower(-relicPower);
            } else {
                relicLMotor.setPower(0);   // Stop the motor
            }

        }
    }
}