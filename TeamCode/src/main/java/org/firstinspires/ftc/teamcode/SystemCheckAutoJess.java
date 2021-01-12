package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "System Check Auto Jess", group = "Linear Opmode")
public class SystemCheckAutoJess extends BaseAutoOpModeJess {

    @Override
    public void runOpMode () {
        //Assigns hardware devices names and values

        GetHardware();
       // GetIMU();

        telemetry.addData("Status", "HI");
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        drive(DRIVE.forward);
        sleep(500);
        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
