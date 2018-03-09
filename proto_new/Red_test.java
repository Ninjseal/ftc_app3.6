package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Purplecoder on 27/01/2018.
 */

@Autonomous(name = "Red_Autonom", group = "Autonomous")
//@Disabled
public class Red_test extends AutonomousMode {

    @Override
    protected void initOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("Done!", "play");
        telemetry.update();

        wait(0.1);
        telemetry.addData("heading", "%3d deg", gyroSensor.getHeading());
        telemetry.update();
    }

    @Override
    protected void runOp() throws InterruptedException {

        ball_auto(true);

        int mark = activate_vuforia();

        grab_cube(true);

        wait(0.3);

        cubes_to_position(1, 1500);

        wait(0.3);

        gyro_test(90, 1);

        //rotate_ticks(800, 1);
        //move(-0.4, 0.4); // dreapta
        wait(0.3);
        switch (mark){
            case 1 :
                move_with_encoders(2500, 1, 1);
                break;
            case 2 :
                move_with_encoders(3500, 1, 1);
                telemetry.addData("Done!", "Exiting...");
                break;
            case 3 :
                move_with_encoders(4500, 1, 1);
                break;
            default:
                move_with_encoders(3500, 1, 1);
        }

        //move(-0.6, -0.6);
        //wait(2.0);

        //rotate_ticks(400, 1);

        //move(-0.3, 0.3); //dreapta
        gyro_test(180, 1);
        wait(0.3);

        cubes_to_position(-1, 0);

        wait(0.3);

        grab_cube(false);

        //move(-0.5, -0.5);
        wait(0.3);

        move_with_encoders(1000, 1, 0.8 );

        //wait(0.5);

        telemetry.addData("Done!", "Exiting...");
        telemetry.update();
        sleep(1000);
    }

    @Override
    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}