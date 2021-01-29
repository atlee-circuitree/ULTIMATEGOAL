package org.firstinspires.ftc.teamcode.trainingcodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutoOpMode;


@Autonomous(name = "training auto", group = "Linear Opmode")
public class TrainingAutonomous extends BaseAutoOpMode {

    @Override
    public void runOpMode () {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //Assigns hardware devices names and values

        GetHardware();
       // GetIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        encoderDrive(1, 48, 2);
        encoderDrive(1, -24, 2);
        rotate(90, 1);
      //  claw_servo.setPosition(1);

        //encoderDrive(DRIVE, 50, 5);
        //sleep(1000);
        //ResetEncoder();
        //encoderDrive(DRIVE, -50, 5);


        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
