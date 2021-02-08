package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * The FINAL BLUE autonomous (Run this in matches)
 * encoderStrafeV4 POSITIVE distance = strafe to the LEFT, NEGATIVE distance = strafe to the RIGHT
 */



@Autonomous(name = "FINAL AUTO BLUE", group = "Linear Opmode")
public class FinalAutonomousBlue extends BaseAutoOpMode {

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
        arm_servo.setPosition(0.55);
        encoderDrive(1, 65, 3.0);
        PIDrotate(-1, 2);
        DriveTrain(Drive.STOP);

        shooter_right.setVelocity(1700);
        shooter_left.setVelocity(1700);
        sleep(700);
        belt_feed.setPower(1);
        sleep(500);
        encoderStrafeV4(0.5, -16, 4.0);
        sleep(1000);
        belt_feed.setPower(0);
        shooter_left.setPower(0);
        shooter_right.setPower(0);


        encoderStrafeV4(0.7, 36, 3);
        //belt_feed.setPower(-1);
        PIDrotate(180, 3.0);
        encoderDrive(0.5,4,1.0);
        DriveTrain(Drive.STOP);
        //Lower wobble
        arm_servo.setPosition(0.62);
        sleep(200);


        //changed while to if
        while(lift_bottom_Left.getState() || lift_bottom_Right.getState()){
            lift_Motor.setPower(-0.8);
        }
        lift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Changed while to if
        while (lift_Motor.getCurrentPosition() < 250) {
            lift_Motor.setPower(0.7);
        }
        lift_Motor.setPower(0);
        sleep(200);
        if(distance_sensor.getDistance(DistanceUnit.MM) < 300) {
            ringCount = 4;
        }
        else {
            while(lift_bottom_Left.getState() || lift_bottom_Right.getState()) {
                lift_Motor.setPower(-0.7);
            }
            lift_Motor.setPower(0);
            sleep(200);
            if (distance_sensor.getDistance(DistanceUnit.MM) < 260) {
                ringCount = 1;
            } else {
                ringCount = 0;
            }
        }
        //belt_feed.setPower(0);

        if (ringCount == 0) {
            //Drive Backwards to Position A
            encoderDrive(0.7, -14, 1.0);

            //Place Wobble Goal on Position A
            arm_servo.setPosition(0.65);
            sleep(200);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            //Go to second wobble
            encoderStrafeV4(0.4,12,1.5);
            PIDrotate(180, .5);
            encoderDrive(1,66,3.0);


            //Pick up second wobble
            arm_servo.setPosition(0.67);
            sleep(500);
            encoderStrafeV4(0.4,-4,1.0);
            claw_servo.setPosition(.4);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            //Go back to position A
            encoderDrive(1,-54,3.0);
            encoderStrafeV4(0.6,-8,1.0);

            //Drop second wobble
            arm_servo.setPosition(0.67);
            sleep(500);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.45);
            sleep(200);

            encoderStrafeV4(1,24,1.5);
            encoderDrive(1,-6,1.0);
            PIDrotate(180, 2.0);
            //encoderDrive(1,-8,1.0);


        } else if (ringCount == 1) {

            //Feeds 1 ring
            shooter_left.setVelocity(intake);
            shooter_right.setVelocity(intake);
            belt_feed.setPower(-1);
            sleep(250);
            encoderDrive(1,12,1.0);
            sleep(250);
            shooter_left.setVelocity(0);
            shooter_right.setVelocity(0);
            belt_feed.setPower(0);


            //Move to position B
            encoderDrive(1, -50, 4.0);
            encoderStrafeV4(.6, 14, 2.0);

            //Place Wobble Goal on Position B
            arm_servo.setPosition(0.65);
            sleep(200);
            claw_servo.setPosition(.7);
            sleep(200);
            //arm_servo.setPosition(0.6);
            //sleep(200);

            //Drive to second wobble
            encoderDrive(1,90,4.0);
            encoderStrafeV4(0.6,-8,1.0);

            //Grab wobble 2
            arm_servo.setPosition(0.65);
            sleep(700);
            claw_servo.setPosition(0.4);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            //Go back to position B
            encoderDrive(1,-82,3.0);
            encoderStrafeV4(0.6, 16,1.5);

            //Drop second wobble
            arm_servo.setPosition(0.65);
            sleep(500);
            claw_servo.setPosition(0.7);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            //Go park on line
            encoderStrafeV4(0.6, 36,1.5);
            PIDrotate(180,1.0);


        } else if (ringCount == 4) {

            //Drive Backwards to Position C
            encoderDrive(1, -61, 2.5);

            //Place Wobble Goal on Position C
            arm_servo.setPosition(0.65);
            sleep(200);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            encoderDrive(0.7,48,2.0);
            encoderDrive(1, 6, 1.0);
            PIDrotate(180,1.0);
        }
    }
}