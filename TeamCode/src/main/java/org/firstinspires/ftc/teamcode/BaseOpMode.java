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
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.navXPIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.text.DecimalFormat;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

public abstract class BaseOpMode extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;
    public DcMotorEx shooter_left = null;
    public DcMotorEx shooter_right = null;
    public DcMotor belt_feed = null;
    public DcMotor lift_Motor = null;
    public DigitalChannel lift_bottom_Left = null;
    public DigitalChannel lift_bottom_Right = null;
    public DigitalChannel lift_top = null;
    public AHRS navx_cannon;
    public AHRS navx_centered;


    public Servo arm_servo;
    public Servo claw_servo;


    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    int loop = 0;

    //turn motor at 200 ticks per second
    public double motorVelocity = 200;

    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // eg: GOBUILDA Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE           = 1;

    static final double     OMNI_COUNTS_PER_REV     = 8192; //For rev through bore encoder (This is the correct number)
    static final double     OMNI_WHEEL_DIAMETER     = 3.77953;
    public static final double      OMNI_COUNTS_PER_INCH    = (OMNI_COUNTS_PER_REV) / (OMNI_WHEEL_DIAMETER * Math.PI);

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
        //drive train
        front_left = hardwareMap.get(DcMotor.class, "drive_FL");
        rear_left = hardwareMap.get(DcMotor.class, "drive_RL");
        front_right = hardwareMap.get(DcMotor.class, "drive_FR");
        rear_right = hardwareMap.get(DcMotor.class, "drive_RR");
        //Shooter
        shooter_left = hardwareMap.get(DcMotorEx.class, "shooter_L");
        shooter_right = hardwareMap.get(DcMotorEx.class, "shooter_R");

        belt_feed = hardwareMap.get(DcMotor.class, "belt_Feed");
        lift_Motor = hardwareMap.get(DcMotor.class, "lift_M");

        arm_servo = hardwareMap.get(Servo.class, "arm_servo");
        claw_servo = hardwareMap.get(Servo.class, "claw_servo");

        lift_bottom_Left = hardwareMap.get(DigitalChannel.class,"lift_bottom_L");
        lift_bottom_Right = hardwareMap.get(DigitalChannel.class,"lift_bottom_R");
        lift_top = hardwareMap.get(DigitalChannel.class,"lift_top");


        // set digital channel to input mode.
        lift_top.setMode(DigitalChannel.Mode.INPUT);
        lift_bottom_Left.setMode(DigitalChannel.Mode.INPUT);
        lift_bottom_Right.setMode(DigitalChannel.Mode.INPUT);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        front_left.setDirection(DcMotor.Direction.REVERSE);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_right.setDirection(DcMotor.Direction.FORWARD);

        shooter_left.setDirection(DcMotor.Direction.FORWARD);
        shooter_right.setDirection(DcMotor.Direction.REVERSE);

        belt_feed.setDirection(DcMotor.Direction.FORWARD);
        lift_Motor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SetDriveMode(Mode.STOP_RESET_ENCODER);
        belt_feed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        belt_feed.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SetDriveMode(Mode.RUN_WITH_ENCODER);

        //GetIMU();

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
/*
    public void getGyro() {

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }
    }
    */

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     FLTarget;
        int     FRTarget;
        int     RLTarget;
        int     RRTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  FLSpeed;
        double  RLSpeed;
        double  FRSpeed;
        double  RRSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);

            FLTarget = front_left.getCurrentPosition() + moveCounts;
            RLTarget = rear_left.getCurrentPosition() + moveCounts;
            FRTarget = front_right.getCurrentPosition() + moveCounts;
            RRTarget = rear_right.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            front_left.setTargetPosition(FLTarget);
            front_right.setTargetPosition(FRTarget);
            rear_left.setTargetPosition(RLTarget);
            rear_right.setTargetPosition(RRTarget);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rear_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rear_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            front_left.setPower(speed);
            front_right.setPower(speed);
            rear_left.setPower(speed);
            rear_right.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (front_left.isBusy() && front_right.isBusy() && rear_left.isBusy() && rear_right.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                FLSpeed = speed - steer;
                RLSpeed = speed - steer;
                FRSpeed = speed + steer;
                RRSpeed = speed + steer;

               // max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));


                // Normalize speeds if either one exceeds +/- 1.0;

               // max = Math.max(Math.abs(FLSpeed), Math.abs(FRSpeed), Math.max(Math.abs(RRSpeed), Math.abs(RLSpeed));

                //max = Math.max(Math.abs(FLSpeed), (RLSpeed), (Math.abs(FRSpeed), (RRSpeed));

               // max = Math.max(Math.abs(FLSpeed), Math.abs(FRSpeed));
                max = Math.max(Math.abs(RRSpeed), Math.abs(RLSpeed));

                if (max > 1.0)
                {
                    FLSpeed /= max;
                    FRSpeed /= max;
                    RRSpeed /= max;
                    RLSpeed /= max;
                }

                front_left.setPower(FLSpeed);
                front_right.setPower(FRSpeed);
                rear_left.setPower(RLSpeed);
                rear_right.setPower(RRSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      FLTarget,  FRTarget, RLTarget, RRTarget);
                telemetry.addData("Actual",  "%7d:%7d",      front_left.getCurrentPosition(),
                        front_right.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  FLSpeed, FRSpeed, RLSpeed, RRSpeed);
                telemetry.update();
            }

            // Stop all motion;
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);

            // Turn off RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double  FLSpeed;
        double  RLSpeed;
        double  FRSpeed;
        double  RRSpeed;
        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            FLSpeed  = 0.0;
            RLSpeed = 0.0;
            FRSpeed  = 0.0;
            RRSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            FRSpeed = speed * steer;
            RRSpeed  = speed * steer;
            FLSpeed = -FRSpeed;
            RLSpeed   = -RRSpeed;

        }

        // Send desired speeds to motors.
        front_left.setPower(FLSpeed);
        front_right.setPower(FRSpeed);
        rear_left.setPower(RLSpeed);
        rear_right.setPower(RRSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", FLSpeed, FRSpeed, RLSpeed, RRSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }



    public enum STRAFE {
        LEFT, RIGHT
    }


    public enum Mode {
        STOP_RESET_ENCODER,
        RUN_WITH_ENCODER,
        RUN_WITHOUT_ENCODERS,
    }
    public enum Drive {
        STOP
    }

    public void DriveTrain (Drive Stop){
        if (Stop == Drive.STOP) {
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);
        }
    }

    public void encoderLift(double speed, double timeoutS) {
        int newLiftTarget;
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

    public void getCenteredNavXValues(){

        navx_centered = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx_centered"), AHRS.DeviceDataType.kProcessedData);
        //boolean connected = navx_device.isConnected();
        //telemetry.addData("1 navX-Device", connected ?
        //"Connected" : "Disconnected" );
        String gyrocal, magcal, yaw, pitch, roll, compass_heading;
        String fused_heading, ypr, cf, motion;
        DecimalFormat df = new DecimalFormat("#.##");


        gyrocal = (navx_centered.isCalibrating() ?
                "CALIBRATING" : "Calibration Complete");
        magcal = (navx_centered.isMagnetometerCalibrated() ?
                "Calibrated" : "UNCALIBRATED");
        yaw = df.format(navx_centered.getYaw());
        pitch = df.format(navx_centered.getPitch());
        roll = df.format(navx_centered.getRoll());
        ypr = yaw + ", " + pitch + ", " + roll;
        compass_heading = df.format(navx_centered.getCompassHeading());
        fused_heading = df.format(navx_centered.getFusedHeading());
        if (!navx_centered.isMagnetometerCalibrated()) {
            compass_heading = "-------";
        }
        cf = compass_heading + ", " + fused_heading;
        if ( navx_centered.isMagneticDisturbance()) {
            cf += " (Mag. Disturbance)";
        }
        motion = (navx_centered.isMoving() ? "Moving" : "Not Moving");
        if ( navx_centered.isRotating() ) {
            motion += ", Rotating";
        }


        telemetry.addData("2 GyroAccel", gyrocal );
        telemetry.addData("3 Y,P,R", ypr);
        telemetry.addData("4 Magnetometer", magcal );
        telemetry.addData("5 Compass,9Axis", cf );
        telemetry.addData("6 Motion", motion);

    }


    public void getCannonNavXValues(){

        navx_cannon = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx_cannon"), AHRS.DeviceDataType.kProcessedData);
        //boolean connected = navx_device.isConnected();
        //telemetry.addData("1 navX-Device", connected ?
        //"Connected" : "Disconnected" );
        String gyrocal, magcal, yaw, pitch, roll, compass_heading;
        String fused_heading, ypr, cf, motion;
        DecimalFormat df = new DecimalFormat("#.##");


            gyrocal = (navx_cannon.isCalibrating() ?
                    "CALIBRATING" : "Calibration Complete");
            magcal = (navx_cannon.isMagnetometerCalibrated() ?
                    "Calibrated" : "UNCALIBRATED");
            yaw = df.format(navx_cannon.getYaw());
            pitch = df.format(navx_cannon.getPitch());
            roll = df.format(navx_cannon.getRoll());
            ypr = yaw + ", " + pitch + ", " + roll;
            compass_heading = df.format(navx_cannon.getCompassHeading());
            fused_heading = df.format(navx_cannon.getFusedHeading());
            if (!navx_cannon.isMagnetometerCalibrated()) {
                compass_heading = "-------";
            }
            cf = compass_heading + ", " + fused_heading;
            if (navx_cannon.isMagneticDisturbance()) {
                cf += " (Mag. Disturbance)";
            }
            motion = (navx_cannon.isMoving() ? "Moving" : "Not Moving");
            if (navx_cannon.isRotating() ) {
                motion += ", Rotating";
            }


        telemetry.addData("2 GyroAccel", gyrocal );
        telemetry.addData("3 Y,P,R", ypr);
        telemetry.addData("4 Magnetometer", magcal );
        telemetry.addData("5 Compass,9Axis", cf );
        telemetry.addData("6 Motion", motion);

    }
}






