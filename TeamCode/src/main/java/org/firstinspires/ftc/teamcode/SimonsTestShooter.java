package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
* Simon's general test opmode
*/



@Autonomous(name = "Simon's Test Shooter", group = "Linear Opmode")
public class SimonsTestShooter extends BaseAutoOpMode {

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


        PIDrotate(-15,3.0);

        while(lift_bottom_Left.getState() && lift_bottom_Right.getState()){
            lift_Motor.setPower(-0.5);
        }
        lift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(lift_Motor.getCurrentPosition() < 1500){
            lift_Motor.setPower(0.6);
        }
        lift_Motor.setPower(0);

        shooter_right.setVelocity(shooterFar);
        shooter_left.setVelocity(shooterFar);
        sleep(2000);
        belt_feed.setPower(1);
        sleep(4000);
        belt_feed.setPower(0);
        shooter_left.setPower(0);
        shooter_right.setPower(0);
        sleep(1000);


    }

}
