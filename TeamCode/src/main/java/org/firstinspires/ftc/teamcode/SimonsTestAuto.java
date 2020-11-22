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

        //Assigns hardware devices names and values

        GetHardware();
        //getCannonNavXValues();
        getCenteredNavXValues();
        initPID();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();  

        //encoderDrive(1,24,4);
        PIDrotate(90,0.5);

        while(opModeIsActive()){
            getCenteredNavXValues();
        }

    }

}
