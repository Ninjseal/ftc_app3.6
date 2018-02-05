package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Purplecoder on 27/01/2018
 */

@Autonomous(name = "Red_Autonom", group = "Autonomous")
//@Disabled
public class Red_test extends AutonomousMode {

    @Override
    protected void initOpMode() throws InterruptedException {
        initHardware();
    }

    protected void runOp() throws InterruptedException {

        ball_auto(true);
        wait(1000);

        telemetry.addData("Done!", "Exiting...");
        telemetry.update();
        sleep(1000);
    }

    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}
