package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Purplecoder on 27/01/2018.
 */

@Autonomous(name = "Forward_test", group = "Autonomous")
@Disabled
public class forward_test extends AutonomousMode {

    @Override
    protected void initOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("Done!", "play");
        telemetry.update();
    }

    protected void runOp() throws InterruptedException {

        move_with_encoders(2000, 1, 1);

        telemetry.addData("Done!", "Exiting...");
        telemetry.update();
        sleep(1000);
    }

    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}
