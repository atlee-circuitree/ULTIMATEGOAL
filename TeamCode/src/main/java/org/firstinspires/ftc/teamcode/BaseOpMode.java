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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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

    public Servo REV_SERVO = null;
    public Servo JX_SERVO  = null;
    public Servo ECO_SERVO = null;
    public Servo DS_SERVO  = null;

    //public DigitalChannel top_touch = null;

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    int loop = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: GOBUILDA Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93700787 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE           = 1;
    //static final double     STRAFE          = 1;  //DO NOT UN COMMENT (will screw up "public void STRAFE")
    static final double     TURN_SPEED      = 1;


    public void GetHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        front_left = hardwareMap.get(DcMotor.class, "drive_FL");
        rear_left = hardwareMap.get(DcMotor.class, "drive_RL");
        front_right = hardwareMap.get(DcMotor.class, "drive_FR");
        rear_right = hardwareMap.get(DcMotor.class, "drive_RR");

        REV_SERVO = hardwareMap.get(Servo.class, "REV");
        JX_SERVO = hardwareMap.get(Servo.class, "JX");
        ECO_SERVO = hardwareMap.get(Servo.class, "ECO");
        DS_SERVO = hardwareMap.get(Servo.class, "DS");

       // top_touch = hardwareMap.get(DigitalChannel.class, "top_touch");

        // set digital channel to input mode.

       // top_touch.setMode(DigitalChannel.Mode.INPUT);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        front_left.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);

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


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        if (degrees < 0) {   // turn right.
            curvedRotate(degrees, power, -power);
        } else if (degrees > 0) {   // turn left.
            curvedRotate(degrees, -power, power);
        } else return;

    }

    /**
     * Rotate Left or Right the number of degrees.  Does not support turning more than 180 degrees.
     * By passing in different left and right values (e.g. .5 and 1), the robot should travel instead of turning in place
     * @param degrees
     * @param leftPower
     * @param rightPower
     */

    public void curvedRotate(int degrees, double leftPower, double rightPower) {
        //double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // set power to rotate.
        front_left.setPower(leftPower);
        rear_left.setPower(leftPower);
        front_right.setPower(rightPower);
        rear_right.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() >= degrees) {
                telemetry.addData("Angle", getAngle());
                telemetry.update();
                if(getAngle() <= (degrees + 20)){

                    telemetry.addData("RotateRight", "SlowingDown");
                    telemetry.update();
                    front_left.setPower(0.2);
                    rear_left.setPower(0.2);
                    front_right.setPower(-0.2);
                    rear_right.setPower(-0.2);
                }


            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                telemetry.addData("Angle", getAngle());
                telemetry.update();
                if(getAngle() >= (degrees - 20)){
                    //double offset = (degrees - getAngle());
//                    front_left.setPower( Math.pow(leftPower * (offset/(degrees * 0.1)), 2 ));
//                    rear_left.setPower( Math.pow(leftPower * (offset/(degrees * 0.1)), 2 ));
//                    front_right.setPower( Math.pow(rightPower * (offset/(degrees * 0.1)), 2 ));
//                    rear_right.setPower( Math.pow(rightPower * (offset/(degrees * 0.1)), 2 ));

                    telemetry.addData("RotateLeft", "SlowingDown");
                    telemetry.addData("Angle", getAngle());
                    telemetry.update();
                    front_left.setPower(-0.2);
                    rear_left.setPower(-0.2);
                    front_right.setPower(0.2);
                    rear_right.setPower(0.2);
                }
            }

        // turn the motors off.
        front_left.setPower(0);
        rear_left.setPower(0);
        front_right.setPower(0);
        rear_right.setPower(0);

        // wait for rotation to stop.
        //simon u DONT NEED 1000 ms sleep

        //DOUG IK I DIDNT WANT TO DELET IT IF IT WS NECESRY

        //sleep(1000);

        // reset angle tracking on new heading.
        //resetAngle();
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }


    public void rotateNoSlowDown(int degrees, double power)
    {
        NoSlowDownRotate(degrees, power, power);
    }

    public void NoSlowDownRotate(int degrees, double leftPower, double rightPower) {
        resetAngle();

       /* if (degrees < 0) {   // turn right.
            leftPower = leftPower;
            rightPower = -rightPower;
        } else if (degrees > 0) {   // turn left.
            leftPower = -leftPower;
            rightPower = rightPower;
        } else return;

        // set power to rotate.
        front_left.setPower(leftPower);
        rear_left.setPower(leftPower);
        front_right.setPower(rightPower);
        rear_right.setPower(rightPower);


        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() >= degrees) {
                telemetry.addData("Angle", getAngle());
                telemetry.update();
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                telemetry.addData("Angle", getAngle());
                telemetry.update();
            }

       /* // turn the motors off.
        front_left.setPower(0);
        rear_left.setPower(0);
        front_right.setPower(0);
        rear_right.setPower(0);


        */

        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = leftPower;
            rightPower = -rightPower;
        } else if (degrees > 0) {   // turn left.
            leftPower = -leftPower;
            rightPower = rightPower;
        } else return;



        // set power to rotate.
        front_left.setPower(leftPower);
        rear_left.setPower(leftPower);
        front_right.setPower(rightPower);
        rear_right.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() >= degrees) {
                telemetry.addData("Angle", getAngle());
                telemetry.update();
                if(getAngle() <= (degrees + 20)){

                    telemetry.addData("RotateRight", "SlowingDown");
                    telemetry.update();
                    front_left.setPower(1);
                    rear_left.setPower(1);
                    front_right.setPower(-1);
                    rear_right.setPower(-1);
                }


            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                telemetry.addData("Angle", getAngle());
                telemetry.update();
                if(getAngle() >= (degrees - 20)){
                    //double offset = (degrees - getAngle());
//                    front_left.setPower( Math.pow(leftPower * (offset/(degrees * 0.1)), 2 ));
//                    rear_left.setPower( Math.pow(leftPower * (offset/(degrees * 0.1)), 2 ));
//                    front_right.setPower( Math.pow(rightPower * (offset/(degrees * 0.1)), 2 ));
//                    rear_right.setPower( Math.pow(rightPower * (offset/(degrees * 0.1)), 2 ));

                    telemetry.addData("RotateLeft", "SlowingDown");
                    telemetry.addData("Angle", getAngle());
                    telemetry.update();
                    front_left.setPower(-1);
                    rear_left.setPower(-1);
                    front_right.setPower(1);
                    rear_right.setPower(1);
                }
            }

        // turn the motors off.
        front_left.setPower(0);
        rear_left.setPower(0);
        front_right.setPower(0);
        rear_right.setPower(0);

        // wait for rotation to stop.
        //simon u DONT NEED 1000 ms sleep
        //sleep(1000);

        // reset angle tracking on new heading.
        //resetAngle();





    }
    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if(heading < -180) {
            heading = heading + 360;
        }
        else if(heading > 180){
            heading = heading - 360;
        }
        //heading = heading - reset_angle;
        return heading;
    }

    public enum DriveDirection {
        LEFT,
        RIGHT,
        STOP,

    }

    public enum STRAFE {
        LEFT, RIGHT
    }


    public enum Mode {
        STOP_RESET_ENCODER,
        RUN_WITH_ENCODER,
        RUN_WITHOUT_ENCODERS,
    }



    public void Drive(DriveDirection direction) {
        if (direction == DriveDirection.STOP) {
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);
        }
        if (direction == DriveDirection.RIGHT) {
            front_left.setPower(1);
            front_right.setPower(-1);
            rear_left.setPower(1);
            rear_right.setPower(-1);
        }
        if (direction == DriveDirection.LEFT) {
            front_left.setPower(-1);
            front_right.setPower(1);
            rear_left.setPower(-1);
            rear_right.setPower(1);
        }
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
    
    public void encoderDrive( double speed,
                              double distance,
                              double timeoutS) {
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
                    (front_left.isBusy() && front_right.isBusy() && rear_left.isBusy() && rear_right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newRLTarget, newFLTarget, newRRTarget, newFRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        front_left.getCurrentPosition(),
                        front_right.getCurrentPosition(),
                        rear_left.getCurrentPosition(),
                        rear_right.getCurrentPosition());
                telemetry.update();
                //RunSafetyCutoff();
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

            //  sleep(250);   // optional pause after each move
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






