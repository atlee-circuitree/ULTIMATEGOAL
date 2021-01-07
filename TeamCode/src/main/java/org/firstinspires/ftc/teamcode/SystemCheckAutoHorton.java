package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "System Check Auto Horton", group = "Linear Opmode")

public class SystemCheckAutoHorton extends BaseAutoOpMode_Horton {


    @Override
    public void runOpMode() {
        //Assigns hardware devices names and values

        GetHardware();
        telemetry.addData("status", "Da Robot be ready");
        telemetry.update();
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                back_left.getCurrentPosition(),
                back_right.getCurrentPosition(),
                front_left.getCurrentPosition(),
                front_right.getCurrentPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();

        encoderDrive(.25,12,12,20);
//forward and turning right
        /*
        telemetry.addData("status", "da robot be movin");
        encoderDrive(0.5,24,24, 2.0);
        encoderDrive(0.5,20, -20, 1.0);
        encoderDrive(0.5, 24, 24, 2.0);
        encoderDrive(0.5, 20, -20, 1.0);
        encoderDrive(0.5,24,24, 2.0);
        encoderDrive(0.5,20, -20, 1.0);
        encoderDrive(0.5, 24, 24, 2.0);
        encoderDrive(0.5, 20, -20, 1.0);
//reverse and turn left
        encoderDrive(0.5,-24,-24, 2.0);
        encoderDrive(0.5,-20, 20, 1.0);
        encoderDrive(0.5, -24, -24, 2.0);
        encoderDrive(0.5, -20, 20, 1.0);
        encoderDrive(0.5,-24,-24, 2.0);
        encoderDrive(0.5,-20, 20, 1.0);
        encoderDrive(0.5, -24, -24, 2.0);
        encoderDrive(0.5, -20, 20, 1.0);



*/
      //  telemetry.addData("path", "complete");
        //telemetry.update();
    }
}