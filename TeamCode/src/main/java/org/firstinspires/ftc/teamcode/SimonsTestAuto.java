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

        while(lift_bottom_Left.getState() && lift_bottom_Right.getState()){
            lift_Motor.setPower(-0.4);
        }
        lift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(lift_Motor.getCurrentPosition() < 300){
            lift_Motor.setPower(0.6);
        }
        lift_Motor.setPower(0);

        encoderDrive(0.7,30,1.5);
        PIDrotate(-50,1.7);
        DriveTrain(Drive.STOP);
        sleep(100);
        if(distance_sensor.getDistance(DistanceUnit.MM) < 300){
            ringCount = 4;
        }
        else{
            while(lift_bottom_Left.getState() && lift_bottom_Right.getState()){
                lift_Motor.setPower(-0.4);
            }
            lift_Motor.setPower(0);
            sleep(100);
            if(distance_sensor.getDistance(DistanceUnit.MM) < 260){
                ringCount = 1;
            }
            else{
                ringCount = 0;
            }
        }

        if(ringCount == 0){
            PIDrotate(0,1.7);
            arm_servo.setPosition(0.55);
            sleep(100);
            //was 48
            encoderDrive(1,46,3.0);
            encoderStrafeV4(0.7,12,1.5);

            arm_servo.setPosition(0.65);
            sleep(1000);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.55);

            PIDrotate(177,3.0);
            encoderDrive(1,69,4.5);
            arm_servo.setPosition(0.65);
            sleep(1700);
            claw_servo.setPosition(0.4);
            sleep(500);
            arm_servo.setPosition(0.55);
            sleep(2000);
            PIDrotate(0,3.0);
            encoderDrive(1,80,5.0);
            claw_servo.setPosition(0.7);
            sleep(500);


        }
        else if(ringCount == 1){
            PIDrotate(0,1.5);
            encoderDrive(1,75,4.0);
            encoderStrafeV4(1,35,3.0);

            arm_servo.setPosition(0.65);
            sleep(1000);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.55);

            PIDrotate(180,3.0);
            encoderStrafeV4(1,35,3.0);
            encoderDrive(1,75,4.0);

        }
        else if(ringCount == 4){
            PIDrotate(0,1.5);
            encoderDrive(1,98,5.0);
            encoderStrafeV4(1,12,1.5);
            arm_servo.setPosition(0.65);
            sleep(2500);
            claw_servo.setPosition(.7);
            sleep(200);
            arm_servo.setPosition(0.47);
            sleep(2500);
        }





    }

}
