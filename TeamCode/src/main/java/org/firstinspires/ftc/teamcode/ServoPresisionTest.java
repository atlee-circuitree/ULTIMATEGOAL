package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Servo Precision Test", group = "Linear Opmode")
public class ServoPresisionTest extends BaseAutoOpMode {

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
        DS_SERVO.setPosition(Set_Servo_Center);  //Value would be 0.5
        sleep(500);

        GB_SPEED_SERVO.setPosition(0.4);
        DS_SERVO.setPosition(0.6);
        sleep(500);

        GB_SPEED_SERVO.setPosition(0.3);
        DS_SERVO.setPosition(0.7);
        sleep(500);

        GB_SPEED_SERVO.setPosition(0.2);
        DS_SERVO.setPosition(0.8);
        sleep(500);

        GB_SPEED_SERVO.setPosition(0.1);
        DS_SERVO.setPosition(0.9);
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
