package org.firstinspires.ftc.teamcode.trainingcodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "System Check Auto Horton", group = "Linear Opmode")

public class SystemCheckAutoHorton extends BaseAutoOpMode_Horton{


    @Override
    public void runOpMode () {
        //Assigns hardware devices names and values

        GetHardware();
        waitForStart();
        runtime.reset();


        telemetry.addData("status", "da robot be movin");
        //drive(STRAFE.left);

        telemetry.addData("status",  "Da Robot be ready" );

        telemetry.update();
        waitForStart();

        //drive(STRAFE.right);

        telemetry.addData("path","complete");
        telemetry.update();


    }

}
