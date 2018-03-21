package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Robo-Sapiens on 23/03/2018.
 */

@Autonomous(name = "Blue_Autonom2", group = "Autonomous")
//@Disabled
public class Blue_Autonom_Alt extends AutonomousMode {

    @Override
    protected void initOpMode() throws InterruptedException {
        initHardware();
    }

    @Override
    protected void runOp() throws InterruptedException {

        ball_auto(false);

        int mark = activate_vuforia();

        grab_cube(true);
        cubes_to_position(1200, 1, 1);

        gyro_turn(90, -1);

        switch (mark){
            case 1 :
                move_with_encoders(3000, 1, 1, 4);
                break;
            case 2 :
                move_with_encoders(3600, 1, 1, 5);
                break;
            case 3 :
                move_with_encoders(4000, 1, 1, 6);
                break;
            default:
                move_with_encoders(3600, 1, 1, 5);
        }

        gyro_turn(180, -1);

        move_with_encoders(900, 1, 1, 2);

        cubes_to_position(1200, -1, 1);
        grab_cube(false);

        move_with_encoders(500, -1, 1, 1.5);

        telemetry.addData("Done!", "Exiting...");
        telemetry.update();
        sleep(300);
    }

    @Override
    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}
