package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Simon's general test opmode
 * encoderStrafeV4 POSITIVE distance = strafe to the LEFT, NEGATIVE distance = strafe to the RIGHT
 */



@Autonomous(name = "Simon's test auto", group = "Linear Opmode")
public class SimonsTestAuto extends BaseAutoOpMode {

    private int ringCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Assigns hardware devices names and values
        //Position robot measured
                //Front: 12-11/16"
                //Rear: 12-1/2"

        GetHardware();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //navx_centered.zeroYaw();
       // sleep(500);

        //Drive to Shoot White Line
        claw_servo.setPosition(.37);
        lift_Motor.setPower(0);
        arm_servo.setPosition(0.55);
        encoderDrive(.6, 62, 3.0); //changed from .6 to 1
        PIDrotate(-1, .5);
        DriveTrain(Drive.STOP);

        //Shoot Left Peg
        shooter_right.setVelocity(1700);
        shooter_left.setVelocity(1700);
        sleep(700);
        belt_feed.setPower(1);
        sleep(700);
        belt_feed.setPower(0);

        //Shoot Middle Peg
        encoderStrafeV5(0.35, -6, 1.0); //changed from .35 to .5
        PIDrotate(0, 1);
        belt_feed.setPower(1);
        sleep(750);
        belt_feed.setPower(0);

       //Shoot Right Peg
        encoderStrafeV5(0.35, -5, 1.0);
        PIDrotate(0, 1);
        belt_feed.setPower(1);
        sleep(1500);
        belt_feed.setPower(0);
        shooter_left.setPower(0);
        shooter_right.setPower(0);
        PIDrotate(0, 0.75);
        DriveTrain(Drive.STOP);

        //Move towards Ring Stack
        encoderStrafeV5(0.4, 21, 3);
        PIDrotate(180, 2);



        //encoderDrive(.6,4, 1);
        //DriveTrain(Drive.STOP);

        //Lower wobble
        arm_servo.setPosition(0.62);
        sleep(200);

        //changed while to if
        while (lift_bottom_Left.getState() && lift_bottom_Right.getState()) {
            lift_Motor.setPower(-0.8);
        }
        lift_Motor.setPower(0);
        lift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Changed while to if
        while (lift_Motor.getCurrentPosition() < 250) {
            lift_Motor.setPower(0.7);
        }
        lift_Motor.setPower(0);
        sleep(200);
        if (distance_sensor.getDistance(DistanceUnit.MM) < 300) {
            ringCount = 4;
        } else {
            while (lift_bottom_Left.getState() && lift_bottom_Right.getState()) {
                lift_Motor.setPower(-0.7);
            }
            lift_Motor.setPower(0);
            sleep(200);
            if (distance_sensor.getDistance(DistanceUnit.MM) < 265) {
                ringCount = 1;
            } else {
                ringCount = 0;
            }
        }

        if (ringCount == 0) {
            //Drive Backwards to Position A
            PIDrotate(180, .5);
            encoderDrive(0.75, -11, 2.0);
            encoderStrafeV5(0.5, -5, 1.0);

            //Place Wobble Goal on Position A
            arm_servo.setPosition(0.65);
            sleep(200);
            claw_servo.setPosition(.7);
            sleep(200);
            //arm_servo.setPosition(0.6);
            //sleep(200);

            //Go to second wobble
            encoderStrafeV5(0.5, 10, 1.5);
            PIDrotate(180, 1);
            encoderDrive(0.6, 65, 3.0);
            DriveTrain(Drive.STOP);


            //Pick up second wobble
           // arm_servo.setPosition(0.67);
           // sleep(500);
            encoderStrafeV5(0.5, -12, 1.0);
            DriveTrain(Drive.STOP);
            claw_servo.setPosition(.4);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            //Go back to position A
            encoderDrive(0.6, -52, 3.0);
            DriveTrain(Drive.STOP);
            encoderStrafeV5(0.75, -6, 1.0);

            //Drop second wobble
            arm_servo.setPosition(0.65);
            sleep(500);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.45);
            sleep(200);

            encoderStrafeV5(.75, 4, 1.5);
            encoderDrive(1,-6,1);
            PIDrotate(180, 1);
            DriveTrain(Drive.STOP);



        } else if (ringCount == 1) {

            //Feeds 1 ring
            shooter_left.setVelocity(intake);
            shooter_right.setVelocity(intake);
            belt_feed.setPower(-1);
            sleep(250);
            encoderDrive(.8, 12, 1);
            //sleep(250);                                       //took off to see if we can save time
            shooter_left.setVelocity(0);
            shooter_right.setVelocity(0);
            belt_feed.setPower(0);


            //Move to position B
            PIDrotate(180, .5);
            encoderDrive(.7, -44, 4.0);
            DriveTrain(Drive.STOP);
            encoderStrafeV5(.6, 8, 2.0);
            DriveTrain(Drive.STOP);

            //Place Wobble Goal on Position B
            arm_servo.setPosition(0.65);
            sleep(200);
            claw_servo.setPosition(.7);
            sleep(200);
            //arm_servo.setPosition(0.6);
            //sleep(200);


            //Drive to second wobble
            PIDrotate(180,1);
            encoderDrive(.8,81,4.0);
            PIDrotate(180, .5);
            arm_servo.setPosition(0.65);
            encoderStrafeV5(0.4,-12,1.0);


            //Grab wobble 2
           // arm_servo.setPosition(0.65);
            sleep(100);
            claw_servo.setPosition(0.4);
            sleep(200);
            arm_servo.setPosition(0.6);
            sleep(200);

            //Go back to position B
            encoderDrive(.9,-72,3.0);
            //PIDrotate(180, .5);
            encoderStrafeV5(.7, 5,1 );

            //Drop second wobble
            //arm_servo.setPosition(0.65);
            sleep(500);
            claw_servo.setPosition(0.7);
            sleep(200);
            //arm_servo.setPosition(0.45);
            //sleep(200);

            //Go park on line
            //encoderStrafeV4(0.6, 4,.75);
            arm_servo.setPosition(0.45);
            encoderDrive(1, 12, 1.5);
            PIDrotate(180,1.0);

            /*
            while(lift_top.getState()) {
                lift_Motor.setPower(1);
            }
            lift_Motor.setPower(0);

            //Shoot Rings to top goal
            shooter_right.setVelocity(1700);
            shooter_left.setVelocity(1700);
            sleep(3000);
            belt_feed.setPower(1);
            sleep(2000);
*/

        } else if (ringCount == 4) {
            {
                while(lift_bottom_Left.getState() || lift_bottom_Right.getState()){
                    lift_Motor.setPower(-0.8);
                }

                //Feeds 4 ring
                shooter_left.setVelocity(intake);
                shooter_right.setVelocity(intake);
                belt_feed.setPower(-1);
                sleep(250);
                encoderDrive(1,16,1.0);
                sleep(250);
                shooter_left.setVelocity(0);
                shooter_right.setVelocity(0);
                belt_feed.setPower(0);

                while(lift_top.getState()) {
                    lift_Motor.setPower(1);
                }
                lift_Motor.setPower(0);

                //Drive Backwards to Position C
                PIDrotate(180, 1);
                encoderDrive(0.7,-92,2.0);

                //Place Wobble Goal on Position A
                arm_servo.setPosition(0.65);
                sleep(200);
                claw_servo.setPosition(.7);
                sleep(200);
                arm_servo.setPosition(0.55);
                sleep(200);

                //Move to Shoot
                encoderDrive(0.7,52,2.0);
                PIDrotate(0, 2);

                //Shoot Rings to top goal
                shooter_right.setVelocity(1700);
                shooter_left.setVelocity(1700);
                sleep(3000);
                belt_feed.setPower(1);
                sleep(2000);

                //Move to Line
                encoderDrive(.7,12, 2);
                PIDrotate(0, 1);

            }
        }
    }
}