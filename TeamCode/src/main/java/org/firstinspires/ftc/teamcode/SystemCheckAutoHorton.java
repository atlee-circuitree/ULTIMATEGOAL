package org.firstinspires.ftc.teamcode;

import android.database.sqlite.SQLiteException;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "System Check Auto Horton", group = "Linear Opmode")

public class SystemCheckAutoHorton extends BaseAutoOpMode_Horton{


    @Override
    public void runOpMode () {
        //Assigns hardware devices names and values

        GetHardware();
        telemetry.addData("status",  "Da Robot be ready" );
        telemetry.update();
        waitForStart();
        runtime.reset();

        telemetry.addData("status", "da robot be movin");
        drive(STRAFE.left);
        sleep(1000);
        telemetry.update();
        telemetry.addData("path","complete");
        telemetry.update();


    }

}
