package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
* Simon's general test opmode
*/

@Autonomous(name = "Simon's Test Auto", group = "Linear Opmode")
public class SimonsTestAuto extends BaseAutoOpMode {

    @Override
    public void runOpMode () throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Assigns hardware devices names and values

        GetHardware();




        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        encoderStrafeV5(1,48,5.0);




    }

}
