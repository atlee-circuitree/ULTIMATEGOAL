package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.navXPIDController;

/**
 * The FINAL BLUE autonomous (Run this in matches)
 * encoderStrafeV4 POSITIVE distance = strafe to the LEFT, NEGATIVE distance = strafe to the RIGHT
 */



@Autonomous(name = "Simons Test Shooter", group = "Linear Opmode")
public class SimonsTestShooter extends BaseAutoOpMode {

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
        encoderDrive(0.7, 62, 3.0);
        PIDrotate(0, 1.0);
        DriveTrain(Drive.STOP);

        shooter_right.setVelocity(1700);
        shooter_left.setVelocity(1700);
        sleep(700);
        belt_feed.setPower(1);
        for(int i = 0; i < 2; i++) {
            sleep(700);
            belt_feed.setPower(0);
            encoderStrafeV5(0.3, -8, 4.0);
            PIDrotate(0,0.5);
            belt_feed.setPower(1);
        }
        sleep(1000);
        belt_feed.setPower(0);
        shooter_left.setPower(0);
        shooter_right.setPower(0);

        PIDrotate(0,0.5);
        encoderStrafeV5(0.4, 26, 3);
        PIDrotate(180, 3.0);
        encoderDriveNoRamp(0.4,3,1.0);
        //Lower wobble
        arm_servo.setPosition(0.62);
        sleep(200);


        //changed while to if
        while(lift_bottom_Left.getState() && lift_bottom_Right.getState()){
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
        if(distance_sensor.getDistance(DistanceUnit.MM) < 300) {
            ringCount = 4;
        }
        else {
            while(lift_bottom_Left.getState() && lift_bottom_Right.getState()) {
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
        while(opModeIsActive()){
            telemetry.addData("ringCount", ringCount);
            telemetry.addData("Distance", distance_sensor.getDistance(DistanceUnit.MM));
            telemetry.update();
            sleep(1000);
        }
    }
}