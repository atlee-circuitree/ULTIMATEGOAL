package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

//Simon's encoder test opmode (created 11/19/2020)
@Autonomous(name = "Simon's Test Auto", group = "Linear Opmode")
public class SimonsTestAuto extends BaseAutoOpMode {

    @Override
    public void runOpMode () {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //Assigns hardware devices names and values

        GetHardware();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        EncoderDriveByInches(5,1);
        fieldOrientedRotate(90,0.5);


    }

}
