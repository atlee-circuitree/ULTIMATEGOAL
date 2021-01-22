package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
* Simon's general test opmode
*/



@Autonomous(name = "Simon's Test Auto", group = "Linear Opmode")
public class SimonsTestAuto extends BaseAutoOpMode {

    private int ringCount = 0;

    @Override
    public void runOpMode () throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Assigns hardware devices names and values

        GetHardware();




        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        claw_servo.setPosition(.4);
        lift_Motor.setPower(0);
        encoderDrive(0.7,57,3.0);
        encoderStrafeV4(1,-12,2.0);
        belt_feed.setPower(-1);

        shooter_right.setVelocity(shooterFar);
        shooter_left.setVelocity(shooterFar);
        sleep(1000);
        belt_feed.setPower(1);
        sleep(7000);
        belt_feed.setPower(0);
        shooter_left.setPower(0);
        shooter_right.setPower(0);
        sleep(1000);

        encoderDrive(1,64,4.0);
        encoderStrafeV4(0.7,-6,1);
        arm_servo.setPosition(0.65);
        sleep(2500);
        claw_servo.setPosition(.7);
        sleep(200);
        arm_servo.setPosition(0.55);
        sleep(2000);

        /*
        PIDrotate(180,3.0);
        encoderDrive(1,122,5.0);

        arm_servo.setPosition(0.65);
        sleep(1000);
        claw_servo.setPosition(0.4);
        sleep(200);
        arm_servo.setPosition(0.55);
        sleep(1000);
        PIDrotate(0,3.0);
        encoderDrive(1,124,5.0);
        */

    }

}
