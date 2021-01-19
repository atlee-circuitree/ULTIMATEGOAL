package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
* Simon's general test opmode
*/



@Autonomous(name = "Simon's Test Functions", group = "Linear Opmode")
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

        lift_Motor.setPower(0);

        shooter_left.setVelocity(intake);
        shooter_right.setVelocity(intake);

        encoderDrive(0.7,60,4.0);

    }




    }


