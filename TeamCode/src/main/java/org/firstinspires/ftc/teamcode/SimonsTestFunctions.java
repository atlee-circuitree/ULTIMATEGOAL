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

        encoderDrive(0.7,60,4.0);
        while(!gamepad1.a){
            telemetry.addData("NavX Yaw", navx_centered.getYaw());
            telemetry.addData("Rotation 1","to 90");
            telemetry.update();
        }
        PIDrotate(90,2.0);
        while(!gamepad1.a){
            telemetry.addData("NavX Yaw", navx_centered.getYaw());
            telemetry.addData("Rotation 2","to 0");
            telemetry.update();
        }
        PIDrotate(0,2.0);
        while(!gamepad1.a){
            telemetry.addData("NavX Yaw", navx_centered.getYaw());
            telemetry.addData("Rotation 3","to -90");
            telemetry.update();
        }
        PIDrotate(-90,2.0);
        while(!gamepad1.a){
            telemetry.addData("NavX Yaw", navx_centered.getYaw());
            telemetry.addData("Rotation 4","to -175");
            telemetry.update();
        }
        PIDrotate(-175,2.0);
        while(!gamepad1.a){
            telemetry.addData("NavX Yaw", navx_centered.getYaw());
            telemetry.addData("Rotation 5","to -180");
            telemetry.update();
        }
        PIDrotate(-180,1.0);
        while(!gamepad1.a){
            telemetry.addData("NavX Yaw", navx_centered.getYaw());
            telemetry.addData("Press a to end","");
            telemetry.update();
        }



    }

}


