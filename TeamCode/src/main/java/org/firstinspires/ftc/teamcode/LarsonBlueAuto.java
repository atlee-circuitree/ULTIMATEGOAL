package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
* Simon's general test opmode
*/



@Autonomous(name = "Larson's Blue Auto", group = "Linear Opmode")
public class LarsonBlueAuto extends BaseAutoOpMode {

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

        belt_feed.setPower(-1);
        sleep(100);
        belt_feed.setPower(0);

        shooter_right.setVelocity(shooterFar);
        shooter_left.setVelocity(shooterFar);
        sleep(500);
        encoderStrafeV4(.7,-12,2.0);
        belt_feed.setPower(1);
        sleep(7000);
        belt_feed.setPower(0);
        shooter_left.setPower(0);
        shooter_right.setPower(0);
        sleep(500); //Larson - Changed from 1000 to 500 1/25/2021

        encoderDrive(1,1,1.0);
        encoderStrafeV4(0.7,-36,1);
        PIDrotate(180,3.0);


        if(lift_bottom_Left.getState() || lift_bottom_Right.getState()){
            lift_Motor.setPower(-0.8);}

        if (distance_sensor.getDistance(DistanceUnit.MM) < 300) {
            ringCount = 4;
        }

        else {
            while (lift_bottom_Left.getState() && lift_bottom_Right.getState()) {
                lift_Motor.setPower(-0.4);
            }
            lift_Motor.setPower(0);
            sleep(200);

            if (distance_sensor.getDistance(DistanceUnit.MM) < 260) {
                ringCount = 1;
            }
            else {
                ringCount = 0;
            }
        }

        if (ringCount == 0) {
            encoderDrive(1, -30, 3.0);
          //  encoderStrafeV4(1, 2, 1.5);

            arm_servo.setPosition(0.65);
            sleep(3000);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.47);
            sleep(2500);
            encoderStrafeV4(.7,12,2.0);
            encoderDrive(1,60,1.0);

        } else if (ringCount == 1) {
            encoderDrive(1, -54, 4.0);
            encoderStrafeV4(1, 35, 3.0);

            arm_servo.setPosition(0.65);
            sleep(3000);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.47);
            sleep(2500);
            encoderStrafeV4(.7,12,2.0);
            encoderDrive(1,108,1.0);

        } else if (ringCount == 4) {
            encoderDrive(1, -78, 5.0);
            encoderStrafeV4(1, 12, 1.5);
            arm_servo.setPosition(0.65);
            sleep(3000);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.47);
            sleep(2500);
            encoderStrafeV4(.7,12,2.0);
            encoderDrive(1,132,1.0);
        }


        /*

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
