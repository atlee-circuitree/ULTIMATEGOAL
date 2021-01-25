package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
* Gets robot into starting position
*/



@Autonomous(name = "Starting Position", group = "Linear Opmode")
public class StartingPosition extends BaseAutoOpMode {

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

        arm_servo.setPosition(0.55);
        sleep(2000);
        claw_servo.setPosition(0.4);
        sleep(500);
        while(lift_top.getState()) {
            lift_Motor.setPower(1);
        }
        lift_Motor.setPower(0);
        arm_servo.setPosition(0.47);
        sleep(2000);
    }

}
