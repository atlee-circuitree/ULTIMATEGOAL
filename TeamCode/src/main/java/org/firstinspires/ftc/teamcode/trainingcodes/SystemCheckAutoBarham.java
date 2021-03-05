 package org.firstinspires.ftc.teamcode.trainingcodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


 @Autonomous(name = "System Check Auto Barham", group = "Linear Opmode")
public class SystemCheckAutoBarham extends BaseAutoOpModeBarham {

    @Override
    public void runOpMode () {
        //Assigns hardware devices names and values

        GetHardware();

        telemetry.addData("Status", "Da Robot Be Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

       


        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
