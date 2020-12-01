package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;


@Autonomous(name = "System Check Auto", group = "Linear Opmode")
public class SystemCheckAuto extends BaseAutoOpMode {

    @Override
    public void runOpMode () {
        //Assigns hardware devices names and values

        GetHardware();
       // GetIMU();

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        belt_feed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ResetDriveEncoder();
       // belt_feed.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Encoders Reset");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /*
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d", front_left.getCurrentPosition(),
                rear_left.getCurrentPosition(), rear_right.getCurrentPosition(), front_right.getCurrentPosition());
        telemetry.update();

        encoderDrive(DRIVE,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
         */

        telemetry.addData("Path5", "Starting at %7d :%7d", belt_feed.getCurrentPosition(), front_left.getCurrentPosition());
        telemetry.update();

        encoderStrafeV4(1, 24, 10);

        sleep(5000);
        belt_feed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderStrafeV4(.5, -24, 8);


        //telemetry.addData("Velocity", "Starting at %7d", shooter_left.getVelocity());
      //  telemetry.update();
       // SetShooterMotors(Shoot.SHOOT_FAR);




        //belt_feed.setPower(1);



        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
