package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Purplecoder on 27/01/2018.
 */

@Autonomous(name = "Blue_Autonom", group = "Autonomous")
//@Disabled
public class Blue_test extends AutonomousMode {

    @Override
    protected void initOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("Done!", "play");
        telemetry.update();
    }

    @Override
    protected void runOp() throws InterruptedException {
        ball_auto(false);

        wait(1000);

        telemetry.addData("Done!", "Exiting...");
        telemetry.update();
        sleep(1000);
    }

    @Override
    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}
