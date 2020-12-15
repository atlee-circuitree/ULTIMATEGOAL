package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "System Check Auto", group = "Linear Opmode")
public class SystemCheckAutoHorton extends BaseOpMode_Horton {

    @Override
    public void runOpMode () {
        //Assigns hardware devices names and values

        GetHardware();

        telemetry.addData( caption: "status" value: "Da Robot be ready" )

    }

}
