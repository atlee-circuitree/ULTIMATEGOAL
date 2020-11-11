package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous(name = "System Check Auto", group = "Linear Opmode")
public class SystemCheckAuto extends BaseAutoOpModeEncoderTest {

    @Override
    public void runOpMode () {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();


        //Assigns hardware devices names and values

        GetHardware();
        GetIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        sleep(1000);
        encoderDrive(DRIVE, 50, 5);
        sleep(1000);
        ResetEncoder();
        encoderDrive(DRIVE, -50, 5);




        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
