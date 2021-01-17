package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * No, this is not final yet...
 */



@Autonomous(name = "FinalAutonomousRed", group = "Linear Opmode")
@Disabled
public class FinalAutonomousRed extends BaseAutoOpMode {

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



        while(lift_bottom_Left.getState() && lift_bottom_Right.getState()){
            lift_Motor.setPower(-0.4);
        }
        lift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(lift_Motor.getCurrentPosition() < 300){
            lift_Motor.setPower(0.6);
        }
        lift_Motor.setPower(0);

        encoderDrive(0.7,30,2);
        PIDrotate(-45,3.0);
        DriveTrain(Drive.STOP);
        sleep(200);
        if(distance_sensor.getDistance(DistanceUnit.MM) < 300){
            ringCount = 4;
        }
        else{
            while(lift_bottom_Left.getState() && lift_bottom_Right.getState()){
                lift_Motor.setPower(-0.4);
            }
            lift_Motor.setPower(0);
            sleep(200);
            if(distance_sensor.getDistance(DistanceUnit.MM) < 260){
                ringCount = 1;
            }
            else{
                ringCount = 0;
            }
        }

        while(opModeIsActive()){
            telemetry.addData("Distance",distance_sensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Rings: ",ringCount);
            telemetry.update();
        }




    }

}
