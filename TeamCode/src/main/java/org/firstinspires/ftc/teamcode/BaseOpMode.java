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
    public DcMotor lift_left = null;
    public DcMotor lift_right = null;
    public DcMotor feeder_motor = null;
    public DcMotor top_motor = null;
    public Servo Clamp_Left = null;
    public Servo Clamp_Right = null;
  //  public Servo Feeder_Servo = null;
    public Servo Block_Pickup = null;
    public Servo Capstone = null;
    public Servo Release_Servo = null;
    public Servo Release_Servo2 = null;
    public DigitalChannel Top_Sensor_Front = null;
    public DigitalChannel Top_Sensor_Rear = null;
    public DigitalChannel bottom_touch = null;
    public DigitalChannel top_touch = null;

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    int loop = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE           = 1;
    static final double     STRAFE          = 1;
    static final double     TURN_SPEED      = 1;


    public void GetHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        lift_left = hardwareMap.get(DcMotor.class, "lift_left");
        lift_right = hardwareMap.get(DcMotor.class, "lift_right");
        feeder_motor = hardwareMap.get(DcMotor.class, "feeder_motor");
        top_motor = hardwareMap.get(DcMotor.class, "top_motor");
        Clamp_Left = hardwareMap.get(Servo.class, "Clamp_Left");
        Clamp_Right = hardwareMap.get(Servo.class, "Clamp_Right");
       // Feeder_Servo = hardwareMap.get(Servo.class, "Feeder_Servo");
        Block_Pickup = hardwareMap.get(Servo.class, "Block_Pickup");
        Capstone = hardwareMap.get(Servo.class, "Capstone");
        Release_Servo = hardwareMap.get(Servo.class, "Release_Servo");
        Release_Servo2 = hardwareMap.get(Servo.class, "Release_Servo2");
        Top_Sensor_Rear = hardwareMap.get(DigitalChannel.class, "Top_Sensor_Rear");
        Top_Sensor_Front = hardwareMap.get(DigitalChannel.class, "Top_Sensor_Front");
        bottom_touch = hardwareMap.get(DigitalChannel.class, "bottom_touch");
        top_touch = hardwareMap.get(DigitalChannel.class, "top_touch");


        // set digital channel to input mode.
        Top_Sensor_Front.setMode(DigitalChannel.Mode.INPUT);
        Top_Sensor_Rear.setMode(DigitalChannel.Mode.INPUT);
        bottom_touch.setMode(DigitalChannel.Mode.INPUT);
        top_touch.setMode(DigitalChannel.Mode.INPUT);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        front_left.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        lift_left.setDirection(DcMotor.Direction.REVERSE);
        lift_right.setDirection(DcMotor.Direction.REVERSE);
        feeder_motor.setDirection(DcMotor.Direction.REVERSE);
        top_motor.setDirection(DcMotor.Direction.FORWARD);

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

    //PID rotation variables
    Orientation lastAnglesPID = new Orientation();
    double cumulativeDegrees = 0;
    double                  globalAnglePID, power = .30, rotatePower = 1, correction, rotation;

    public void rotatePID_InPlace(int degrees, double power)
    {
        cumulativeDegrees += degrees;
        resetAnglePID();
        double targetDegrees = ((cumulativeDegrees % 360) - getAnglePID());
        PIDController pidRotate = new PIDController(0,0,0);
        rotatePID(targetDegrees, power, pidRotate);
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotatePID(double degrees, double power, PIDController pidRotate)
    {
        // restart imu angle tracking.
        //resetAnglePID();

        // If input degrees > 359, we cap at 359 with same sign as input.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. We compute the p and I
        // values based on the input degrees and starting power level. We compute the tolerance %
        // to yield a tolerance value of about 1 degree.
        // Overshoot is dependant on the motor and gearing configuration, starting power, weight
        // of the robot and the on target tolerance.

        pidRotate.reset();

        double p = Math.abs(power/degrees);
        double i = p / 100.0;
        pidRotate.setPID(p, i, 0);

        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1.0 / Math.abs(degrees) * 100.0);
        pidRotate.enable();

        // getAnglePID() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAnglePID() == 0)
            {
                front_left.setPower(power);
                rear_left.setPower(power);
                front_right.setPower(-power);
                rear_right.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAnglePID()); // power will be - on right turn.
                front_left.setPower(-power);
                rear_left.setPower(-power);
                front_right.setPower(power);
                rear_right.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAnglePID()); // power will be + on left turn.
                front_left.setPower(-power);
                rear_left.setPower(-power);
                front_right.setPower(power);
                rear_right.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        front_left.setPower(0);
        rear_left.setPower(0);
        front_right.setPower(0);
        rear_right.setPower(0);

        rotation = getAnglePID();

        telemetry.addData("2 global heading", globalAnglePID);
        telemetry.update();


        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        //resetAnglePID();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    void resetAnglePID()
    {
        lastAnglesPID = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //globalAnglePID = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAnglePID()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAnglePID = angles.firstAngle - lastAnglesPID.firstAngle;

        if (deltaAnglePID < -180)
            deltaAnglePID += 360;
        else if (deltaAnglePID > 180)
            deltaAnglePID -= 360;

        globalAnglePID += deltaAnglePID;

        lastAnglesPID = angles;

        return globalAnglePID;
    }

    public void rotate(int degrees, double power)
    {
        if (degrees < 0) {   // turn right.
            curvedRotate(degrees, power, -power);
        } else if (degrees > 0) {   // turn left.
            curvedRotate(degrees, -power, power);
        } else return;

    }
/* Panten working on PID
    public rotateToAngle(float targetAngle) {
        float error = targetAngle - getAngle();
        if (error > threshold)
            this.rotation =  error*kP
        return False
    else:
        this.rotation = 0
        return True

    }


    function move(fwd, rotation):
            // This function allows for joystick input
            this.fwd = fwd
    this.rotation = rotation

    function execute():
            // Execute function that should be called every loop
            this.robotdrive.drive(this.fwd, this.rotation)

            this.fwd = 0
            this.rotation = 0

*/
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
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        STOP,
        STRAFE_RIGHT,
        STRAFE_LEFT,
        BACK_LEFT,
        TURN_RIGHT
    }


    public enum Mode {
        STOP_RESET_ENCODER,
        RUN_WITH_ENCODER,
        RUN_WITHOUT_ENCODERS,
    }


    public enum LiftDirection {
        STOP,
        UP,
        DOWN
    }


    public void StopAllDrive() {
        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);
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
        if (direction == DriveDirection.FORWARD) {
            front_left.setPower(1);
            front_right.setPower(1);
            rear_left.setPower(1);
            rear_right.setPower(1);
        }
        if (direction == DriveDirection.BACKWARD) {
            front_left.setPower(-1);
            front_right.setPower(-1);
            rear_left.setPower(-1);
            rear_right.setPower(-1);


        }
    }

    public void RunSafetyCutoff() {
        if (Top_Sensor_Rear.getState() && top_motor.getPower() == 1) {
            top_motor.setPower(0);
        }
        else if(Top_Sensor_Front.getState() && top_motor.getPower() == -1)
        {
            top_motor.setPower(0);
        }
        else if(bottom_touch.getState() && lift_left.getPower() == -1) {
            lift_left.setPower(0);
            lift_right.setPower(0);
        }

    }

    public void EncoderDrive(DriveDirection direction, int EncoderValue) {
        if (direction == DriveDirection.STOP) {
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);

        }
        if (direction == DriveDirection.RIGHT) {
        //    SetDriveMode(Mode.STOP_RESET_ENCODER);
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);

        //    front_right.setTargetPosition(EncoderValue);
         //   front_left.setTargetPosition(EncoderValue);
         //   rear_right.setTargetPosition(EncoderValue);
         //   rear_left.setTargetPosition(EncoderValue);

         //   front_right.setPower(-1);
         //   rear_left.setPower(1);
        //    front_left.setPower(1);
         //   rear_right.setPower(-1);

         //   while(front_right.isBusy() || front_left.isBusy() || rear_right.isBusy() || rear_left.isBusy() ) {

            // sleep(1);
          //      RunSafetyCutoff();
          //  }

            while(front_left.getCurrentPosition() < EncoderValue) {
                front_left.setPower(1);
                front_right.setPower(-1);
                rear_left.setPower(-1);
                rear_right.setPower(1);
                // sleep(1);
                RunSafetyCutoff();
                idle();
            }





            SetDriveMode(Mode.STOP_RESET_ENCODER);

            //front_left.setTargetPosition(EncoderValue
        }

        if (direction == DriveDirection.LEFT) {

            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);


            while(front_left.getCurrentPosition() < EncoderValue) {
                front_left.setPower(-1);
                front_right.setPower(1);
                rear_left.setPower(-1);
                rear_right.setPower(1);
               // sleep(1);
                RunSafetyCutoff();
                idle();
            }

            SetDriveMode(Mode.STOP_RESET_ENCODER);


        }
        if (direction == DriveDirection.FORWARD) {

            //SetDriveMode(Mode.STOP_RESET_ENCODER);
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);


            while(front_left.getCurrentPosition() < EncoderValue) {
                front_left.setPower(1);
                front_right.setPower(1);
                rear_left.setPower(1);
                rear_right.setPower(1);
              //  sleep(1);
                RunSafetyCutoff();
                idle();
            }

            SetDriveMode(Mode.STOP_RESET_ENCODER);
        }
        if (direction == DriveDirection.BACKWARD) {
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);


            while(front_left.getCurrentPosition() > -EncoderValue) {
                front_left.setPower(-1);
                front_right.setPower(-1);
                rear_left.setPower(-1);
                rear_right.setPower(-1);
               // sleep(1);
                RunSafetyCutoff();
                telemetry.addData("Encoder", front_left.getCurrentPosition());
                telemetry.update();
                idle();
            }

            SetDriveMode(Mode.STOP_RESET_ENCODER);

            //front_left.setTargetPosition(EncoderValue
        }

        if (direction == DriveDirection.STRAFE_LEFT) {
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);


            while(front_left.getCurrentPosition() > -EncoderValue) {
                front_left.setPower(-1);
                front_right.setPower(1);
                rear_left.setPower(1);
                rear_right.setPower(-1);
                sleep(1);
                RunSafetyCutoff();
                telemetry.addData("Encoder", front_left.getCurrentPosition());
                telemetry.update();
                idle();
            }

            SetDriveMode(Mode.STOP_RESET_ENCODER);



        }
        if (direction == DriveDirection.STRAFE_RIGHT) {
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);


            while(front_left.getCurrentPosition() < EncoderValue) {
                front_left.setPower(1);
                front_right.setPower(-1);
                rear_left.setPower(-1);
                rear_right.setPower(1);
                sleep(1);
                RunSafetyCutoff();
                telemetry.addData("Encoder", front_left.getCurrentPosition());
                telemetry.update();
                idle();
            }

            SetDriveMode(Mode.STOP_RESET_ENCODER);


        }

        if (direction == DriveDirection.BACK_LEFT) {
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);

            while(front_left.getCurrentPosition() > -EncoderValue) {
                front_left.setPower(-1);
                front_right.setPower(.2);
                rear_left.setPower(.2);
                rear_right.setPower(-1);
                sleep(1);
                RunSafetyCutoff();
                telemetry.addData( "Encoder", front_left.getCurrentPosition());
                telemetry.update();
                idle();
            }
        }
        if (direction == DriveDirection.TURN_RIGHT) {
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);

            while(front_left.getCurrentPosition() < EncoderValue) {
                front_left.setPower(1);
                front_right.setPower(-1);
                rear_left.setPower(1);
                rear_right.setPower(-1);
                sleep(1);
                RunSafetyCutoff();
                telemetry.addData( "Encoder", front_left.getCurrentPosition());
                telemetry.update();
                idle();
            }
            SetDriveMode(Mode.STOP_RESET_ENCODER);
        }


    }

    public void driveByWire(double drive, double strafe, double turn) {

        telemetry.addData("front_left Encoder Position", front_left.getCurrentPosition());
        telemetry.addData("rear_left Encoder Position", rear_left.getCurrentPosition());
        telemetry.addData("front_right Encoder Position", front_right.getCurrentPosition());
        telemetry.addData("rear_right Encoder Position", rear_right.getCurrentPosition());

        double frontLeftPower;
        double frontRightPower;
        double rearLeftPower;
        double rearRightPower;

        double pi = 3.1415926;


     //   double drive = -gamepad1.left_stick_y;
      //  double strafe = gamepad1.left_stick_x * 1.5;
      //  double turn = gamepad1.right_stick_x;

       /*
       double gyroDegrees = getAngle();

       double gyroRadians = gyroDegrees * pi/180;
       double forwardTemp = drive * Math.cos(gyroRadians) + strafe * Math.sin(gyroRadians);
       strafe = drive * Math.sin(gyroRadians) + strafe * Math.cos(gyroRadians);
       drive = forwardTemp;
*/


        frontLeftPower = (drive + strafe + turn);
        rearLeftPower = (drive - strafe + turn);
        frontRightPower = (drive - strafe - turn);
        rearRightPower = (drive + strafe - turn);



        if (Math.abs(frontLeftPower) > 1 || Math.abs(rearLeftPower) > 1 || Math.abs(frontRightPower) > 1 || Math.abs(rearRightPower) >1) {

            double max = 0;
            max = Math.max(Math.abs(frontLeftPower),Math.abs(rearLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(rearRightPower), max);

            frontLeftPower /= max;
            rearLeftPower /= max;
            frontRightPower /= max;
            rearRightPower /= max;



        }


        front_left.setPower(frontLeftPower);
        rear_left.setPower(rearLeftPower);
        front_right.setPower(frontRightPower);
        rear_right.setPower(rearRightPower);





    }

    public void encoderDrive(double speed,
                             double distance,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = front_left.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newRightTarget = front_right.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            // newRightTarget = rear_right.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            // newLeftTarget = rear_left.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

            front_left.setTargetPosition(newLeftTarget);
            front_right.setTargetPosition(newRightTarget);
            rear_left.setTargetPosition(newLeftTarget);
            rear_right.setTargetPosition(newRightTarget);

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
                    (  front_left.isBusy() && front_right.isBusy() && rear_left.isBusy() && rear_right.isBusy()  )  ) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        front_left.getCurrentPosition(),
                        front_right.getCurrentPosition(),
                        rear_left.getCurrentPosition(),
                        rear_right.getCurrentPosition());
                telemetry.update();
                RunSafetyCutoff();
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


    public void Strafe(DriveDirection direction) {
        if (direction == DriveDirection.LEFT) {
            front_left.setPower(-1);
            front_right.setPower(1);
            rear_left.setPower(1);
            rear_right.setPower(-1);
        }
        if (direction == DriveDirection.RIGHT) {
            front_left.setPower(1);
            front_right.setPower(-1);
            rear_left.setPower(-1);
            rear_right.setPower(1);
        }
    }


    public void Lift(LiftDirection direction) {
        if (direction == LiftDirection.STOP) {
            lift_left.setPower(0);
            lift_right.setPower(0);
        }
        if (direction == LiftDirection.UP) {
            lift_left.setPower(1);
            lift_right.setPower(-1);
        }
        if (direction == LiftDirection.DOWN) {
            lift_left.setPower(-1);
            lift_right.setPower(1);
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






