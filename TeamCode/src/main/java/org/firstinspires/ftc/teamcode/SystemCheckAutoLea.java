 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@Autonomous(name = "System Check Auto Lea", group = "Linear Opmode")
public class SystemCheckAutoLea extends BaseAutoOpModeLea {


    @Override
    public void runOpMode () {
        //Assigns hardware devices names and values

        GetHardware();
        InitializeIMU();

        telemetry.addData("Status", "Da Robot Be Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        Drive(STRAFE.LEFT);


        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
