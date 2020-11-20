package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;


@Autonomous(name = "System Check Auto", group = "Linear Opmode")
public class SystemCheckAuto extends BaseAutoOpMode {

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

        SetDriveMode(Mode.STOP_RESET_ENCODER);
        SetDriveMode(Mode.RUN_WITH_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d", front_left.getCurrentPosition(),
                rear_left.getCurrentPosition(), rear_right.getCurrentPosition(), front_right.getCurrentPosition());
        telemetry.update();

        encoderDrive(DRIVE,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

        //encoderDrive(DRIVE, 50, 5);
        //sleep(1000);
        //ResetEncoder();
        //encoderDrive(DRIVE, -50, 5);


        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
