package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
* Simon's general test opmode
*/



@Autonomous(name = "Larson'sBlueAuto", group = "Linear Opmode")
public class LarsonBlueAuto extends BaseAutoOpMode {

    private int ringCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Assigns hardware devices names and values

        GetHardware();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        claw_servo.setPosition(.37);
        lift_Motor.setPower(0);
        encoderDrive(1, 57, 3.0);

        shooter_right.setVelocity(shooterFar);
        shooter_left.setVelocity(shooterFar);
        sleep(700);
        belt_feed.setPower(1);
        sleep(500);
        encoderStrafeV4(0.7, 16, 4.0);
        sleep(1000);
        belt_feed.setPower(0);
        shooter_left.setPower(0);
        shooter_right.setPower(0);

        //encoderDrive(1,1,1.0);
        encoderStrafeV4(0.7, -60, 1);
        PIDrotate(180, 2.0);
        DriveTrain(Drive.STOP);

        while (lift_bottom_Left.getState() || lift_bottom_Right.getState()) {
            lift_Motor.setPower(-0.7);

            if (lift_bottom_Left.getState() || lift_bottom_Right.getState()) {
                lift_Motor.setPower(-0.8);
            }

            if (distance_sensor.getDistance(DistanceUnit.MM) < 300) {
                ringCount = 4;
            } else {
                while (lift_bottom_Left.getState() && lift_bottom_Right.getState()) {
                    lift_Motor.setPower(-0.4);
                }
                lift_Motor.setPower(0);
                sleep(200);

                if (distance_sensor.getDistance(DistanceUnit.MM) < 260) {
                    ringCount = 1;
                } else {
                    ringCount = 0;
                }
            }

            if (ringCount == 0) {
                //Drive Backwards to Position A
                encoderDrive(1, -20, 1.5);

                //Place Wobble Goal on Position A
                arm_servo.setPosition(0.65);
                sleep(3000);
                encoderStrafeV4(.7, 5, 2);
                claw_servo.setPosition(.7);
                sleep(200);
                arm_servo.setPosition(0.47);
                sleep(2500);
                
                //strafe back to position
                encoderStrafeV4(.7, -5, 2.0);

                //Drive Forward to second Wobble Goal
                encoderDrive(1, 70, 5.0);

                //Pick Up Second Wobble Goal
                arm_servo.setPosition(0.65);
                sleep(3000);
                claw_servo.setPosition(.37);
                sleep(200);
                arm_servo.setPosition(0.47);
                sleep(2500);

                //Drive backwards to Position A
                encoderDrive(1, -70, 5.0);
                encoderStrafeV4(.7, 5, 2.0);

                //Place second Wobble Goal in Position A
                arm_servo.setPosition(0.65);
                sleep(3000);
                claw_servo.setPosition(.7);
                sleep(200);
                arm_servo.setPosition(0.47);
                sleep(2500);

                //Drive to Parking White Line
                encoderDrive(1, 12, 2.0);

            } else if (ringCount == 1) {

                //Place Wobble Goal on Position B
                encoderDrive(1, -44, 4.0);
                encoderStrafeV4(.7, -24, 3.0);

                //Place Wobble Goal on Position B
                arm_servo.setPosition(0.65);
                sleep(3000);
                claw_servo.setPosition(.7);
                sleep(200);
                arm_servo.setPosition(0.47);
                sleep(2500);

                //Drive Forward to second Wobble Goal
                arm_servo.setPosition(0.65);
                sleep(300);
                encoderDrive(1, 94, 5.0);

                arm_servo.setPosition(0.65);
                sleep(3000);
                encoderStrafeV4(.7, 36, 2.0);

                //Pick Up Second Wobble Goal
                claw_servo.setPosition(.37);
                sleep(200);
                arm_servo.setPosition(0.47);
                sleep(2500);

                //Drive backwards to Position B
                encoderStrafeV4(.7, -36, 2.0);
                encoderDrive(1, -82, 5.0);
                
                //Place second Wobble Goal in Position B
                arm_servo.setPosition(0.65);
                sleep(3000);
                claw_servo.setPosition(.7);
                sleep(200);
                arm_servo.setPosition(0.47);
                sleep(2500);

                //Drive to Parking White Line
                encoderDrive(1, 36, 1.0);


            } else if (ringCount == 4) {

                //Place Wobble Goal on Position C
                encoderDrive(1, -68, 5.0);
                encoderStrafeV4(1, 12, 1.5);

                //Place Wobble Goal on Position C
                arm_servo.setPosition(0.65);
                sleep(3000);
                claw_servo.setPosition(.7);
                sleep(200);
                arm_servo.setPosition(0.47);
                sleep(2500);

                //Drive Forward to second Wobble Goal
                encoderStrafeV4(.7, -12, 2.0);
                encoderDrive(1, 118, 1.0);

                arm_servo.setPosition(0.65);
                sleep(3000);
                claw_servo.setPosition(.37);
                sleep(200);
                arm_servo.setPosition(0.47);
                sleep(2500);

                //Drive Reverse to place the second Wobble Goal
                encoderDrive(1, -94, 4.0);
                encoderStrafeV4(.7, 12, 2.0);

                //Place second wobble goal
                arm_servo.setPosition(0.65);
                sleep(3000);
                claw_servo.setPosition(.37);
                sleep(200);
                arm_servo.setPosition(0.47);
                sleep(2500);

                //Drive to Parking White Line
                encoderDrive(1, 60, 1.0);
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
}
