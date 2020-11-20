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

/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */
@Disabled
public abstract class BaseAutoOpMode extends BaseOpMode {


    @Override
    public void GetHardware() {
        super.GetHardware();
        /*front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


         */
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    public void ResetEncoder(){
        SetDriveMode(Mode.STOP_RESET_ENCODER);
        SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);
    }


/*
    public void UnfoldRobotV2() {

        //encoderDrive(DRIVE, 4, 1);
        lift_left.setPower(1);
        lift_right.setPower(-1);
        sleep(400);

        lift_left.setPower(0);
        lift_right.setPower(0);

        Clamp_Left.setPosition(0.4);
        Clamp_Right.setPosition(.5);
        Release_Servo.setPosition(1);
        Release_Servo2.setPosition(0);

        sleep(1000);

        encoderDrive(DRIVE, 5, 1);

      top_motor.setPower(1);
        craneSafetyTimer.reset();
        while(Top_Sensor_Rear.getState() && craneSafetyTimer.milliseconds() < 2000)
        {
            top_motor.setPower(1);
        }
        top_motor.setPower(0);

        Block_Pickup.setPosition(0.4f);
        feeder_motor.setPower(1);
        sleep(1000);
        feeder_motor.setPower(0);

    }
*/

    public void EncoderDriveByInches(double inches, double speed){

        final double     COUNTS_PER_MOTOR_REV    = 3892 ;    // eg: TETRIX Motor Encoder
        final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 3.937;     // For figuring circumference
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = (int)(inches * COUNTS_PER_INCH);

        front_left.setTargetPosition(target);
        front_right.setTargetPosition(target);
        rear_left.setTargetPosition(target);
        rear_right.setTargetPosition(target);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rear_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rear_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(speed);
        front_right.setPower(speed);
        rear_left.setPower(speed);
        rear_right.setPower(speed);

        while (front_left.isBusy() && front_right.isBusy()){
            telemetry.addData("Encoder Target", target);
            telemetry.addData("FL motor encoder", front_left.getCurrentPosition());
            telemetry.addData("FR motor encoder", front_right.getCurrentPosition());
            telemetry.update();
        }
        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void fieldOrientedRotate(int degrees, double speed){

        double angle = navx_device.getYaw();

        if(angle > degrees){
            while(angle != degrees){
                telemetry.addData("Angle",angle);
                telemetry.update();
                angle = navx_device.getYaw();
                front_left.setPower(-speed);
                rear_left.setPower(-speed);
                front_right.setPower(speed);
                rear_right.setPower(speed);
            }
        }
        else if(angle < degrees){
            while(angle != degrees){
                telemetry.addData("Angle",angle);
                telemetry.update();
                angle = navx_device.getYaw();
                front_left.setPower(speed);
                rear_left.setPower(speed);
                front_right.setPower(-speed);
                rear_right.setPower(-speed);
            }
        }

    }
}












