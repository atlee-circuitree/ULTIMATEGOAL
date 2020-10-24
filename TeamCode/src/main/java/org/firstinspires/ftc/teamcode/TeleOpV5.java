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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Teleop made 10/22/2020
 * If you are looking at this in the far future I don't know if the config has changed any
 */

@TeleOp(name="TeleOp_V5", group="Linear Opmode")

public class   TeleOpV5 extends BaseOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left = null;
    private DcMotor rear_left = null;
    private DcMotor front_right = null;
    private DcMotor rear_right = null;

    public DigitalChannel bottom_touch = null;
    private Servo arm_servo;


    //This is code to test mechanum drive
    // declare motor speed variables
    double FR; double FL; double RR; double RL;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;
    // operational constants
    double joyScale = 1;
    double motorMax = 1; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    //Hold the position of the arm servo so we can increment it
    double arm_servo_pos = 0.5;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        front_left = hardwareMap.get(DcMotor.class, "drive_FL");
        rear_left = hardwareMap.get(DcMotor.class, "drive_RL");
        front_right = hardwareMap.get(DcMotor.class, "drive_FR");
        rear_right = hardwareMap.get(DcMotor.class, "drive_RR");

        bottom_touch = hardwareMap.get(DigitalChannel.class,"bottom_touch");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        front_left.setDirection(DcMotor.Direction.REVERSE);
        rear_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_right.setDirection(DcMotor.Direction.FORWARD);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //UpdateDrivetrain();
            UpdateArmServo();
            UpdateShooter();
            UpdateDriveTrain();



        }
    }

    public void UpdateArmServo() {
        //NOTE: should eventually add in hard button stop
        if(gamepad1.right_bumper){
            if(arm_servo_pos < 1.0){
                arm_servo_pos += 0.1;
                arm_servo.setPosition(arm_servo_pos);
            }
        }
        else if(gamepad1.left_bumper){
            if(arm_servo_pos > 0.0){
                arm_servo_pos -= 0.1;
                arm_servo.setPosition(arm_servo_pos);
            }
        }
    }
    public void UpdateShooter(){
        if(gamepad1.a){
            //Shooting mode
            shooter_left.setPower(1);
            shooter_right.setPower(1);
            //Need to find out good arm servo position
            //arm_servo.setPosition(0.5);
        }
        else if(gamepad1.b){
            //Feeder mode
            shooter_left.setPower(-0.25);
            shooter_right.setPower(-0.25);
            //Need to find good arm servo position
            //arm_servo.setPosition(0)
        }
        else if(gamepad1.x){
            //Stop motors
            shooter_left.setPower(0);
            shooter_right.setPower(0);
        }

    }

   //NOTE: Eventually turn this into either me or Larson's omnidirectional drive
    public void UpdateDriveTrain() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        // Reset speed variables
        FL = 0; FR = 0; RL = 0; RR = 0;

        // Get joystick values
        Y1 = -gamepad1.left_stick_y * joyScale; // invert so up is positive
        X1 = gamepad1.left_stick_x * joyScale;
        Y2 = -gamepad1.right_stick_y * joyScale; // Y2 is not used at present
        X2 = gamepad1.right_stick_x * joyScale;

        // Forward/back movement
        FL += Y1; FR += Y1; RL += Y1; RR += Y1;

        // Side to side movement
        FL += X1; FR -= X1; RL -= X1; RR += X1;

        // Rotation movement
        FL += X2; FR -= X2; RL += X2; RR -= X2;

        // Clip motor power values to +-motorMax
        FL = Math.max(-motorMax, Math.min(FL, motorMax));
        FR = Math.max(-motorMax, Math.min(FR, motorMax));
        RL = Math.max(-motorMax, Math.min(RL, motorMax));
        RR = Math.max(-motorMax, Math.min(RR, motorMax));

        // Send values to the motors
        front_left.setPower(FL);
        front_right.setPower(FR);
        rear_left.setPower(RL);
        rear_right.setPower(RR);

        // Send some useful parameters to the driver station
        telemetry.addData("FL", "%.3f", FL);
        telemetry.addData("FR", "%.3f", FR);
        telemetry.addData("RL", "%.3f", RL);
        telemetry.addData("RR", "%.3f", RR);
    }
}


