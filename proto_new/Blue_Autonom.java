package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Mariusica ;) on 09/03/2018.
 */

@Autonomous(name = "Blue_Autonom", group = "Autonomous")
//@Disabled
public class Blue_Autonom extends AutonomousMode {

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

        ball_auto(false);

        int mark = activate_vuforia();

        grab_cube(true);
        cubes_to_position(1, 1500);
        gyro_turn(90, -1);

        //rotate_ticks(800, 1);
        //move(-0.4, 0.4); // dreapta

        switch (mark){
            case 1 :
                move_with_encoders(3500, 1, 1);
                break;
            case 2 :
                move_with_encoders(3500, 1, 1);
                break;
            case 3 :
                move_with_encoders(3500, 1, 1);
                break;
            default:
                move_with_encoders(3500, 1, 1);
        }

        //move(-0.6, -0.6);
        //wait(2.0);

        //rotate_ticks(400, 1);

        //move(-0.3, 0.3); //dreapta
        gyro_turn(180, -1);

        cubes_to_position(-1, 0);

        grab_cube(false);

        move_with_encoders(900, 1, 1);

        move_with_encoders(400, - 1, 1);

        telemetry.addData("Done!", "Exiting...");
        telemetry.update();
        sleep(1000);
    }

    @Override
    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}
