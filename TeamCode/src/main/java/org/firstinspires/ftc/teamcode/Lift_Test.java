package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Lift Test", group = "Linear Opmode")
public class Lift_Test extends BaseAutoOpMode {

    @Override
    public void runOpMode () {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        //Assigns hardware devices names and values

        GetHardware();
        GetIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        GB_SPEED_SERVO.setPosition(0.3);
       // DS_SERVO.setPosition(0.9);  //

        sleep(4000);

        GB_SPEED_SERVO.setPosition(0.5);
        //DS_SERVO.setPosition(0.5);

        sleep(4000);

        GB_SPEED_SERVO.setPosition(.8); //far right
       // DS_SERVO.setPosition(0);  //far right

        sleep(4000);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
