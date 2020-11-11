package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


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

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                front_left.getCurrentPosition(),
                front_right.getCurrentPosition());
        telemetry.update();
        //ResetEncoder();
       // SetDriveMode(Mode.STOP_RESET_ENCODER);

        sleep(1000);
        encoderDrive(DRIVE,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

        //encoderDrive(DRIVE, 50, 5);
        //sleep(1000);
        //ResetEncoder();
        //encoderDrive(DRIVE, -50, 5);




        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
