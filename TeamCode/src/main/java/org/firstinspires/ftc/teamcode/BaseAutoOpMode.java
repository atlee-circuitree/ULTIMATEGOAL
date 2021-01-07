/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.navXPIDController;
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.IDataArrivalSubscriber;

/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */
@Disabled
public abstract class BaseAutoOpMode extends BaseOpMode {


    public navXPIDController yawPIDController;


    @Override

    public void GetHardware() {
        super.GetHardware();
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        yawPIDController = new navXPIDController( navx_centered, navXPIDController.navXTimestampedDataSource.YAW);


    }

    public void ResetDriveEncoder(){
        SetDriveMode(Mode.STOP_RESET_ENCODER);
        SetDriveMode(Mode.RUN_WITH_ENCODER);
    }



    public void encoderStrafe( double speed, double distance, double timeout) {
        int RevEncoderTarget;
        if (opModeIsActive()) {
            RevEncoderTarget = belt_feed.getCurrentPosition() + (int) (distance * OMNI_COUNTS_PER_INCH);

            belt_feed.setTargetPosition(RevEncoderTarget);

            belt_feed.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            front_left.setPower(-Math.abs(speed));
            front_right.setPower(Math.abs(speed));
            rear_left.setPower(Math.abs(speed));
            rear_right.setPower(-Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) && (belt_feed.isBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path3", "Running to %7d", RevEncoderTarget);
                telemetry.addData("Path4", "Running at %7d", belt_feed.getCurrentPosition());
                telemetry.update();
            }
            DriveTrain(Drive.STOP);
            belt_feed.setPower(0);

            SetDriveMode(Mode.RUN_WITH_ENCODER);
        }
    }
    public void encoderStrafeV2( double speed, double distance, double timeout) {
        int RevEncoderTarget;
        int FLTest;
        if (opModeIsActive()) {
            RevEncoderTarget = belt_feed.getCurrentPosition() + (int) (distance * OMNI_COUNTS_PER_INCH);
            FLTest = front_left.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * 100);

            belt_feed.setTargetPosition(RevEncoderTarget);
            front_left.setTargetPosition(FLTest);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            belt_feed.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            front_left.setPower(-Math.abs(speed));
            front_right.setPower(Math.abs(speed));
            rear_left.setPower(Math.abs(speed));
            rear_right.setPower(-Math.abs(speed));
            belt_feed.setPower(-Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) && (belt_feed.isBusy() && front_left.isBusy()))
            {
                // Display it for the driver.
                telemetry.addData("Path3", "Running to %7d :%7d", RevEncoderTarget, FLTest);
                telemetry.addData("Path4", "Running at %7d", belt_feed.getCurrentPosition(),
                        front_left.getCurrentPosition());
                telemetry.update();
            }
            DriveTrain(Drive.STOP);
            belt_feed.setPower(0);

            SetDriveMode(Mode.RUN_WITH_ENCODER);
        }
    }

    public void encoderStrafeV3( double speed, double distance, double timeout) {
        int RevEncoderTarget;
        int FLTest;
        if (opModeIsActive()) {
            RevEncoderTarget = belt_feed.getCurrentPosition() + (int) (distance * OMNI_COUNTS_PER_INCH);

          //  belt_feed.setTargetPosition(RevEncoderTarget);

            runtime.reset();

            if (belt_feed.getCurrentPosition() < RevEncoderTarget) {
                front_left.setPower(-Math.abs(speed));
                front_right.setPower(Math.abs(speed));
                rear_left.setPower(Math.abs(speed));
                rear_right.setPower(-Math.abs(speed));
            }
            else if (belt_feed.getCurrentPosition() > RevEncoderTarget) {
                front_left.setPower(Math.abs(speed));
                front_right.setPower(-Math.abs(speed));
                rear_left.setPower(-Math.abs(speed));
                rear_right.setPower(Math.abs(speed));
            }
            else {
                DriveTrain(Drive.STOP);
            }

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout))
            {
                // Display it for the driver.
                telemetry.addData("Path3", "Running to %7d", RevEncoderTarget);
                telemetry.addData("Path4", "Running at %7d", belt_feed.getCurrentPosition());
                telemetry.update();
            }
            DriveTrain(Drive.STOP);
            belt_feed.setPower(0);

          //  SetDriveMode(Mode.RUN_WITH_ENCODER);
        }
    }
    public void encoderStrafeV4( double speed, double distance, double timeout) {
        int RevEncoderTarget;

        if (opModeIsActive()) {
            RevEncoderTarget = belt_feed.getCurrentPosition() + (int) (distance * OMNI_COUNTS_PER_INCH);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeout) && (belt_feed.getCurrentPosition() < RevEncoderTarget)) {
                front_left.setPower(-Math.abs(speed));
                front_right.setPower(Math.abs(speed));
                rear_left.setPower(Math.abs(speed));
                rear_right.setPower(-Math.abs(speed));
                telemetry.addData("Path3", "Running to %7d", RevEncoderTarget);
                telemetry.addData("Path4", "Running at %7d", belt_feed.getCurrentPosition());
                telemetry.update();
            }
            DriveTrain(Drive.STOP);
            while (opModeIsActive() && (runtime.seconds() < timeout) && (belt_feed.getCurrentPosition() > RevEncoderTarget)) {
                front_left.setPower(Math.abs(speed));
                front_right.setPower(-Math.abs(speed));
                rear_left.setPower(-Math.abs(speed));
                rear_right.setPower(Math.abs(speed));
                telemetry.addData("Path3", "Running to %7d", RevEncoderTarget);
                telemetry.addData("Path4", "Running at %7d", belt_feed.getCurrentPosition());
                telemetry.update();
            }
            DriveTrain(Drive.STOP);

            //  SetDriveMode(Mode.RUN_WITH_ENCODER);
        }
    }
    public void encoderDrive( double speed, double distance, double timeoutS) {
        int newFLTarget;
        int newRLTarget;
        int newRRTarget;
        int newFRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = front_left.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newFRTarget = front_right.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRRTarget = rear_right.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newRLTarget = rear_left.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

            front_left.setTargetPosition(newFLTarget);
            front_right.setTargetPosition(newFRTarget);
            rear_left.setTargetPosition(newRLTarget);
            rear_right.setTargetPosition(newRRTarget);

            // Turn On RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rear_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rear_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));
            rear_left.setPower(Math.abs(speed));
            rear_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (rear_left.isBusy() && front_left.isBusy() && front_right.isBusy() && rear_right.isBusy()))
            {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newRLTarget, newFLTarget, newRRTarget, newFRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", front_left.getCurrentPosition(),
                        front_right.getCurrentPosition(),
                        rear_left.getCurrentPosition(),
                        rear_right.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            DriveTrain(Drive.STOP);

            // Turn off RUN_TO_POSITION
            SetDriveMode(Mode.RUN_WITH_ENCODER);
            //  sleep(250);   // optional pause after each move

        }
    }


    //You dont really need to worry about this Doug, it doesn't work currently - Simon (12/15/2020)
    public void encoderStrafeV5( double speed, double distance, double timeoutS) {
        int newFLTarget;
        int newRLTarget;
        int newRRTarget;
        int newFRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = front_left.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newFRTarget = front_right.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newRRTarget = rear_right.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH);
            newRLTarget = rear_left.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

            front_left.setTargetPosition(newFLTarget);
            front_right.setTargetPosition(newFRTarget);
            rear_left.setTargetPosition(newRLTarget);
            rear_right.setTargetPosition(newRRTarget);

            // Turn On RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rear_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rear_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            front_left.setPower(speed);
            front_right.setPower(-speed);
            rear_left.setPower(-speed);
            rear_right.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (rear_left.isBusy() && front_left.isBusy() && front_right.isBusy() && rear_right.isBusy()))
            {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newRLTarget, newFLTarget, newRRTarget, newFRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", front_left.getCurrentPosition(),
                        front_right.getCurrentPosition(),
                        rear_left.getCurrentPosition(),
                        rear_right.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            DriveTrain(Drive.STOP);

            // Turn off RUN_TO_POSITION
            SetDriveMode(Mode.RUN_WITH_ENCODER);
            //  sleep(250);   // optional pause after each move

        }
    }



    public void PIDrotate(double target, double cutoffTime) throws InterruptedException {

        final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
        final double TARGET_ANGLE_DEGREES = target;
        final double TOLERANCE_DEGREES = 2.0;
        final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
        final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
        final double YAW_PID_P = 0.005;
        final double YAW_PID_I = 0.0;
        final double YAW_PID_D = 0.0;
        //int DEVICE_TIMEOUT_MS =500;
        int problemChild = 0;
        ElapsedTime runtime = new ElapsedTime();

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        while (runtime.time() <= cutoffTime) {
            if(yawPIDController.waitForNewUpdate(yawPIDResult, 500)){
                double output = yawPIDResult.getOutput() * 2;
                if (output < 0) {
                    /* Rotate Left */
                    telemetry.addData("PID Left", yawPIDResult.getOutput());
                    front_left.setPower(-output);
                    front_right.setPower(output);
                    rear_left.setPower(-output);
                    rear_right.setPower(output);
                } else if (output > 0) {
                    /* Rotate Right */
                    telemetry.addData("PID Right", yawPIDResult.getOutput());
                    front_left.setPower(output);
                    front_right.setPower(-output);
                    rear_left.setPower(output);
                    rear_right.setPower(-output);
                } else {
                    telemetry.addData("PID On Point", yawPIDResult.getOutput());
                    front_left.setPower(0);
                    front_right.setPower(0);
                    rear_left.setPower(0);
                    rear_right.setPower(0);
                }
                telemetry.addData("NavX Yaw: ", navx_centered.getYaw());
                telemetry.update();
                problemChild++;
            } else{
                telemetry.addData("Timeout occured","");
                telemetry.update();
            }
        }


    }






    public void rotate(int degrees, double speed){

        double angle = navx_centered.getYaw();

        if(angle > degrees){
            while(angle >= degrees){
                telemetry.addData("Angle",angle);
                telemetry.update();
                angle = navx_centered.getYaw();
                front_left.setPower(-speed);
                rear_left.setPower(-speed);
                front_right.setPower(speed);
                rear_right.setPower(speed);
            }
        }
        else if(angle < degrees){
            while(angle <= degrees){
                telemetry.addData("Angle",angle);
                telemetry.update();
                angle = navx_centered.getYaw();
                front_left.setPower(speed);
                rear_left.setPower(speed);
                front_right.setPower(-speed);
                rear_right.setPower(-speed);
            }
        }
        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);
    }
}












