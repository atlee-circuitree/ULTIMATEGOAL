package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous(name = "System Check Auto", group = "Linear Opmode")
public class SystemCheckAuto extends BaseAutoOpMode {

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

    //Start of System Checks for Drive Train, Servo(s), Buttons, and Sensors

        encoderDrive(DRIVE, 50, 3);
        encoderDrive(DRIVE, -50, 3);

        sleep(1000);

        REV_SERVO.setPosition(.1f);
        sleep(5000);
        REV_SERVO.setPosition(1f);
        sleep(5000);

        REV_SERVO.setPosition(0.2f);
        sleep(5000);
        REV_SERVO.setPosition(0.9f);
        sleep(5000);


        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
