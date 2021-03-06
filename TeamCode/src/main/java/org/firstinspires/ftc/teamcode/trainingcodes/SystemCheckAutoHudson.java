package org.firstinspires.ftc.teamcode.trainingcodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "System Check Auto Hudson", group = "Linear Opmode")
public class SystemCheckAutoHudson extends BaseAutoOpModeHudson {

    @Override
    public void runOpMode () {
        GetHardware();

        telemetry.addData("Status", "Robot is ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        Drive(drive_train.FORWARDS);
        sleep(1000);
        Drive(drive_train.STOP);
        Drive(STRAFE.LEFT);
        sleep(1000);
        Drive(drive_train.STOP);
        Drive(drive_train.BACKWARDS);
        sleep(1000);
        Drive(drive_train.STOP);
        Drive(STRAFE.RIGHT);
        sleep(1000);
        Drive(drive_train.STOP);
        Drive(drive_train.FORWARDS);
        sleep(1000);
        Drive(drive_train.STOP);

        telemetry.addData("Path","Complete");
        telemetry.update();
    }

}
