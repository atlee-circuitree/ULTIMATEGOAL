package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "System Check Auto Hudson", group = "Linear Opmode")
public class SystemCheckAutoHudson extends BaseAutoOpModeHudson {

    @Override
    public void runOpMode () {
        GetHardware();

        telemetry.addData("Status", "Robot is ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        encoderDrive(1,3,4);

        telemetry.addData("Path","Complete");
        telemetry.update();
    }

}
