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
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.navXPIDController;

/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */
@Disabled
public abstract class BaseAutoOpMode extends BaseOpMode {


    private navXPIDController yawPIDController;

    @Override

    public void GetHardware() {
        super.GetHardware();
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    public void ResetEncoder(){
        SetDriveMode(Mode.STOP_RESET_ENCODER);
        SetDriveMode(Mode.RUN_WITH_ENCODER);
    }


    public void encoderStrafe( double velocity, double distance, double timeout) {
        int RobotTarget;
        if (opModeIsActive()) {
            RobotTarget = belt_feed.getCurrentPosition() + (int) (distance * OMNI_COUNTS_PER_INCH);

            belt_feed.setTargetPosition(RobotTarget);

            belt_feed.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            front_left.setVelocity(Math.abs(-velocity));
            front_right.setVelocity(Math.abs(velocity));
            rear_left.setVelocity(Math.abs(velocity));
            rear_right.setVelocity(Math.abs(-velocity));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) && (belt_feed.isBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", RobotTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", belt_feed.getCurrentPosition());
                telemetry.update();
            }
            DriveTrain(Drive.STOP);

            SetDriveMode(Mode.RUN_WITH_ENCODER);
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
            //Commented out one is for as soon as one motor gets the value they all stop
            //(runtime.seconds() < timeoutS) &&
            //(rear_left.isBusy() && front_left.isBusy() && front_right.isBusy() && rear_right.isBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newRLTarget, newFLTarget, newRRTarget, newFRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", front_left.getCurrentPosition(), front_right.getCurrentPosition(), rear_left.getCurrentPosition(), rear_right.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            DriveTrain(Drive.STOP);

            // Turn off RUN_TO_POSITION
            SetDriveMode(Mode.RUN_WITH_ENCODER);
        }
    }
    public void initPID(){
        double tolerance_degrees = 2.0;
        yawPIDController = new navXPIDController( navx_centered, navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        //yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(-1, 1);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, tolerance_degrees);
        yawPIDController.setPID(0.005, 0.0, 0.0);
        yawPIDController.enable(true);

    }


    public void PIDrotate(double target, double speed) {
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
        yawPIDController.setSetpoint(target);
        telemetry.addData("Setpoint","");
        telemetry.update();
                double output = yawPIDResult.getOutput();
                if ( output < 0 ) {
                    /* Rotate Left */
                    telemetry.addData("PID",yawPIDResult.getOutput());
                    front_left.setPower(-1);
                    front_right.setPower(1);
                    rear_left.setPower(-1);
                    rear_right.setPower(1);
                } else {
                    /* Rotate Right */
                    telemetry.addData("PID",yawPIDResult.getOutput());
                    front_left.setPower(1);
                    front_right.setPower(-1);
                    rear_left.setPower(1);
                    rear_right.setPower(-1);
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












