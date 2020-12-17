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

<<<<<<< HEAD
        telemetry.addData("status", "da robot be movin");
        drive(STRAFE.left);
=======
        telemetry.addData("status",  "Da Robot be ready" );
>>>>>>> 25704f4d7eec5147f1b640d394ddb82690d8c5e7

    }

}
