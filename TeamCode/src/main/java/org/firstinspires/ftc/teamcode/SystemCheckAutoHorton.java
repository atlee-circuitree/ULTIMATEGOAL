package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "System Check Auto", group = "Linear Opmode")

public class SystemCheckAutoHorton extends BaseAutoOpMode_Horton{


    @Override
    public void runOpMode () {
        //Assigns hardware devices names and values

        GetHardware();
        waitForStart();
        runtime.reset();


        telemetry.addData("status", "da robot be movin");
        drive(STRAFE.left);

        telemetry.addData("status",  "Da Robot be ready" );

        telemetry.update();
        waitForStart();

        drive(STRAFE.right);

        telemetry.addData("path" "complete");
        telemetry.update();


    }

}
