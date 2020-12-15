package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "System Check Auto", group = "Linear Opmode")
public class SystemCheckAutoLea extends BaseAutoOpModeLea {

    @Override
    public void runOpMode () {
        //Assigns hardware devices names and values

        GetHardware();

        telemetry.addData("Status", "Da Robot Be Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        Drive(STRAFE.LEFT);
        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
