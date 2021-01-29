package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.navXPIDController;

/**
 * Simon's general test opmode
 */



@Autonomous(name = "Final Autonomous Blue", group = "Linear Opmode")
public class FinalAutonomousBlue extends BaseAutoOpMode {

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
        encoderStrafeV4(0.3, -16, 4.0);
        sleep(1000);
        belt_feed.setPower(0);
        shooter_left.setPower(0);
        shooter_right.setPower(0);


        encoderStrafeV4(0.9, 36, 3);
        PIDrotate(180, 3.0);
        encoderDrive(0.5,2,1.0);
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

        if (ringCount == 0) {
            //Drive Backwards to Position A
            encoderDrive(0.7, -8, 1.0);

            //Place Wobble Goal on Position A
            arm_servo.setPosition(0.65);
            sleep(200);
            encoderStrafeV4(.7, -5, 2);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            //strafe back to position
            encoderStrafeV4(.7, 10, 2.0);
            PIDrotate(180,1.0);
            DriveTrain(Drive.STOP);

            //Drive Forward to second Wobble Goal
            encoderDrive(1, 62, 5.0);

            //Pick Up Second Wobble Goal
            arm_servo.setPosition(0.65);
            sleep(500);
            claw_servo.setPosition(.37);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(500);

            //Drive backwards to Position A
            encoderDrive(1, -70, 5.0);
            encoderStrafeV4(.7, -5, 2.0);

            //Place second Wobble Goal in Position A
            arm_servo.setPosition(0.65);
            sleep(1000);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.47);
            sleep(200);

            //Drive to Parking White Line and Rotate to face the goal.
            encoderStrafeV4(1,24,3.0);
            DriveTrain(Drive.STOP);

        } else if (ringCount == 1) {

            //Move to position B
            encoderDrive(1, -42, 4.0);
            encoderStrafeV4(.7, 18, 2);

            //Place Wobble Goal on Position B
            arm_servo.setPosition(0.65);
            sleep(200);
            claw_servo.setPosition(.7);
            sleep(200);

            //Drive Forward to second Wobble Goal
            arm_servo.setPosition(0.65);
            sleep(300);
            encoderDrive(1, 94, 5.0);
            encoderStrafeV4(.7, -18, 2.0);

            //Pick Up Second Wobble Goal
            claw_servo.setPosition(.37);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            //Drive backwards to Position B
            encoderStrafeV4(.7, 18, 2.0);
            encoderDrive(1, -82, 5.0);

            //Place second Wobble Goal in Position B
            arm_servo.setPosition(0.65);
            sleep(1000);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            //Drive to Parking White Line
            encoderDrive(1, 36, 1.0);


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

            //Strafe to pick up rings
            encoderStrafeV4(0.5,4,1.0);
            while(lift_bottom_Right.getState() & lift_bottom_Left.getState()){
                lift_Motor.setPower(-0.7);
            }
            //Feeder on to hopefully pick up 4 rings
            belt_feed.setPower(-1);
            shooter_right.setVelocity(intake);
            shooter_left.setVelocity(intake);
            sleep(200);

            //Drive Forward to second Wobble Goal
            encoderDrive(1, 75, 5.0);
            encoderStrafeV4(0.7,12,2.0);
            encoderDrive(0.7,25,2.0);
            encoderStrafeV4(0.7,-12,2.0);

            //Pick Up Second Wobble Goal
            arm_servo.setPosition(0.65);
            sleep(700);
            claw_servo.setPosition(.37);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            //Drive backwards to Position C
            encoderDrive(1, -100, 5.0);
            encoderStrafeV4(.7, 6, 1.0);

            //Place second Wobble Goal in Position C
            arm_servo.setPosition(0.65);
            sleep(500);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            //Drive to Parking White Line and Rotate to face the goal.
            encoderDrive(1, 36, 2.0);
            DriveTrain(Drive.STOP);
        }

    }
}