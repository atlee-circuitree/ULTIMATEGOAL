package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
* Simon's general test opmode
*/



@Autonomous(name = "Simon's Test Functions", group = "Linear Opmode")
//@Disabled
public class SimonsTestFunctions extends BaseAutoOpMode {

    @Override
    public void runOpMode () throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Assigns hardware devices names and values

        GetHardware();
        getCenteredNavXValues();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        encoderPIDDrive(0.75,96,0,3.0);

        while(opModeIsActive()){
            telemetry.addData("Navx Yaw", navx_centered.getYaw());
            telemetry.update();
            sleep(500);
        }


    }

}


