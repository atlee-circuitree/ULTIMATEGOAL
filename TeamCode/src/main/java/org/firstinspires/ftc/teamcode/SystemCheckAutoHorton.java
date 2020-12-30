package org.firstinspires.ftc.teamcode;



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

        drive(DRIVE.forward);
        sleep(500);
        drive(DRIVE.stop);
        drive(STRAFE.right);
        sleep(500);
        drive(DRIVE.stop);
        drive(DRIVE.reverse);
        sleep(500);
        drive(DRIVE.stop);
        drive(STRAFE.left);
        sleep(500);
        drive(DRIVE.stop);
        sleep(3000);
        drive(TURN.right);
        sleep(500);
        drive(DRIVE.stop);
        drive(TURN.left);
        sleep(500);
        drive(DRIVE.stop);






        telemetry.addData("path","complete");
        telemetry.update();



    }

}
