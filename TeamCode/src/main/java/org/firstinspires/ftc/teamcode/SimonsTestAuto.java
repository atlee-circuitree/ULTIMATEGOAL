package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.navXPIDController;

/**
 * Simon's general test opmode
 */



@Autonomous(name = "Simons Test Auto", group = "Linear Opmode")
public class SimonsTestAuto extends BaseAutoOpMode {

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

        claw_servo.setPosition(.4);
        sleep(200);
        arm_servo.setPosition(0.55);
        sleep(500);
        //Shooting
        encoderDrive(0.7,58,4.0);
        lift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //while(lift_Motor.getCurrentPosition() > -5){
        //    lift_Motor.setPower(-0.6);
        //}
        //lift_Motor.setPower(0);

        shooter_right.setVelocity(shooterFar);
        shooter_left.setVelocity(shooterFar);
        sleep(1000);
        belt_feed.setPower(1);
        sleep(500);
        encoderStrafeV4(0.25,-16,4.0);
        sleep(1000);
        belt_feed.setPower(0);
        shooter_left.setPower(0);
        shooter_right.setPower(0);

        //Wobble goaling
        telemetry.addData("DEBUG 0", " ");
        telemetry.update();
        encoderDrive(0.7,4,1.0);
        encoderStrafeV4(0.7,35,3.0);
        PIDrotate(180,3.0);
        DriveTrain(Drive.STOP);
       while (lift_bottom_Left.getState() || lift_bottom_Right.getState()) {
            lift_Motor.setPower(-0.7);
        }

        //changed while to if
        if(lift_bottom_Left.getState() || lift_bottom_Right.getState()){
            lift_Motor.setPower(-0.8);
        }
        telemetry.addData("DEBUG 1", " ");
        telemetry.update();
        lift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Changed while to if
        if (lift_Motor.getCurrentPosition() < 250) {
            lift_Motor.setPower(0.7);
        }
        telemetry.addData("DEBUG 2", " ");
        telemetry.update();
        lift_Motor.setPower(0);
        sleep(200);
        if(distance_sensor.getDistance(DistanceUnit.MM) < 300) {
            ringCount = 4;
        }
        else {
            telemetry.addData("DEBUG 3", " ");
            telemetry.update();
            if(lift_bottom_Left.getState() || lift_bottom_Right.getState()) {
                lift_Motor.setPower(-0.7);
            }
            telemetry.addData("DEBUG 4", " ");
            telemetry.update();
            lift_Motor.setPower(0);
            sleep(200);
            if (distance_sensor.getDistance(DistanceUnit.MM) < 260) {
                ringCount = 1;
            } else {
                ringCount = 0;
            }
        }
        while(opModeIsActive()){
            telemetry.addData("Distance",distance_sensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Rings", ringCount);
            telemetry.update();
        }


        if(ringCount == 0){
            encoderDrive(1,-12,2.0);
            encoderStrafeV4(0.7,-6,1.0);

            arm_servo.setPosition(0.65);
            sleep(2000);
            claw_servo.setPosition(0.7);
            sleep(100);
            arm_servo.setPosition(0.55);
            sleep(1000);
            PIDrotate(0,5.0);
            DriveTrain(Drive.STOP);
        }
        else if(ringCount == 1){
            encoderDrive(1,-24,2.5);
            encoderStrafeV4(0.7,-12,2.0);

            arm_servo.setPosition(0.65);
            sleep(2000);
            claw_servo.setPosition(0.7);
            sleep(100);
            arm_servo.setPosition(0.55);
            sleep(1000);
            encoderDrive(1,24,2.0);
            PIDrotate(0,5.0);
            DriveTrain(Drive.STOP);

        }
        else if(ringCount == 4){

            encoderDrive(1,-60,4.0);
            encoderStrafeV4(0.7,-6,1.0);

            arm_servo.setPosition(0.65);
            sleep(2000);
            claw_servo.setPosition(0.7);
            sleep(100);
            arm_servo.setPosition(0.55);
            sleep(1000);
            encoderDrive(1,60,4.0);
            PIDrotate(0,5.0);
            DriveTrain(Drive.STOP);

        }

    }
}