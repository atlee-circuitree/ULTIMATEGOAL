package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous(name = "System Check Auto", group = "Linear Opmode")
public class SystemCheckAuto extends BaseAutoOpMode {

    @Override
    public void runOpMode () {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        //Assigns hardware devices names and values

        GetHardware();
        GetIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        GB_SPEED_SERVO.setPosition(Set_Servo_Center);
        DS_SERVO.setPosition(Set_Servo_Center);
        //JX_SERVO.setPosition(Set_Servo_Center);   //Didn't put on robot yet

        sleep(600);

        GB_SPEED_SERVO.setPosition(0.1); //far left
        DS_SERVO.setPosition(1);  //far left

        sleep(500);

        GB_SPEED_SERVO.setPosition(1); //far right
        DS_SERVO.setPosition(0);  //far right

        sleep(1000);
        // encoderDrive(DRIVE, 50, 3);
        // encoderDrive(DRIVE, -50, 3);




        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
