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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

public abstract class BaseOpMode_NavX_Test extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    
    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;
    public DcMotor shooter_left = null;
    public DcMotor shooter_right = null;
    public DcMotor belt_feed = null;

    public DigitalChannel bottom_touch = null;
    private Servo arm_servo;

    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device

    //public DigitalChannel top_touch = null;

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    int loop = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: GOBUILDA Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE           = 1;
    
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    public void GetHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        front_left = hardwareMap.get(DcMotor.class, "drive_FL");
        rear_left = hardwareMap.get(DcMotor.class, "drive_RL");
        front_right = hardwareMap.get(DcMotor.class, "drive_FR");
        rear_right = hardwareMap.get(DcMotor.class, "drive_RR");
        shooter_left = hardwareMap.get(DcMotor.class, "shooter_L");
        shooter_right = hardwareMap.get(DcMotor.class, "shooter_R");
        belt_feed = hardwareMap.get(DcMotor.class, "belt_Feed");

       // bottom_touch = hardwareMap.get(DigitalChannel.class,"bottom_touch");
      //  arm_servo = hardwareMap.get(Servo.class, "arm_servo");

        // set digital channel to input mode.

       // top_touch.setMode(DigitalChannel.Mode.INPUT);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        front_left.setDirection(DcMotor.Direction.REVERSE);
        rear_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        
        shooter_left.setDirection(DcMotor.Direction.FORWARD);
        shooter_right.setDirection(DcMotor.Direction.REVERSE);
        
        belt_feed.setDirection(DcMotor.Direction.FORWARD);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        GetIMU();

    }


    public void GetIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("IMU", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("IMU", "calibrated");
        telemetry.update();
    }




    public enum STRAFE {
        LEFT, RIGHT
    }


    public enum Mode {
        STOP_RESET_ENCODER,
        RUN_WITH_ENCODER,
        RUN_WITHOUT_ENCODERS,
    }


    public void Strafe (STRAFE direction){
        if (direction == STRAFE.LEFT) {
            front_left.setPower(-1);
            front_right.setPower(1);
            rear_left.setPower(1);
            rear_right.setPower(-1);
        }
        if (direction == STRAFE.RIGHT) {
            front_left.setPower(1);
            front_right.setPower(-1);
            rear_left.setPower(-1);
            rear_right.setPower(1);
        }
    }



    public void SetDriveMode(Mode DriveMode) {

        if (DriveMode == Mode.STOP_RESET_ENCODER) {

            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        if (DriveMode == Mode.RUN_WITH_ENCODER) {

            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if (DriveMode == Mode.RUN_WITHOUT_ENCODERS) {

            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    }
}






