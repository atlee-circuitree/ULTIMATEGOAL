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

package org.firstinspires.ftc.teamcode.oldteleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseOpMode;


/**
 * Teleop made 10/22/2020
 * If you are looking at this in the far future I don't know if the config has changed any
 */

@TeleOp(name="TeleOp_V9", group="Linear Opmode")

public class TeleOpV9 extends BaseOpMode {

    // Declare OpMode members. Already declared in base op mode.

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        lift_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        belt_feed.setDirection(DcMotor.Direction.FORWARD);

        shooter_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            
            UpdateShooter();
            UpdateDriveTrain();
            UpdateBelt();
            UpdateLift();
            UpdateArmServo();
            ClawServo();
            shortcuts();
            getCenteredNavXValues();
        }
    }
    public  void ClawServo() {

        telemetry.addData("Bool clawPos",clawPos);
        telemetry.addData("RightStickButton",gamepad1.right_stick_button);
        if(gamepad1.x) {
            if(clawPos == true & gamepad1.x){
                claw_servo.setPosition(.7);
                clawPos = false;
            }
            else if(clawPos == false & gamepad1.x){
                claw_servo.setPosition(.4);
                clawPos = true;
            }
        }

    }

    public void UpdateArmServo() {
        //NOTE: should eventually add in hard button stop
        if(gamepad1.dpad_left){
            arm_servo.setPosition(0.463);
        }
        else if(gamepad1.dpad_right) {
            arm_servo.setPosition(0.58);
        }
        else if(gamepad1.b){
            arm_servo.setPosition(0.463);
        }
    }

    public void UpdateShooter(){
        //telemetry.addData("TargetVelocity", "Running to %7d", 2500);
        telemetry.addData("CurrentVelocityLeft", shooter_left.getVelocity());
        telemetry.addData("CurrentVelocityRight",shooter_right.getVelocity());
        if(gamepad1.right_trigger > 0) {
            shooter_right.setVelocity(shooterFar);
            shooter_left.setVelocity(shooterFar);
           // shooter_left.setTargetPosition(1000);  //Might not need for shooting.  It might turn off the motor after its reached the value
           // shooter_left.setTargetPositionTolerance(100); //Use later when we want the shooter to wait until its within a certain velocity before shooting
        }
        else if (gamepad1.left_trigger > 0) {
            shooter_left.setVelocity(intake);
            shooter_right.setVelocity(intake);
        }
        else if(gamepad1.b){
            //Stop motors
            shooter_left.setPower(0);
            shooter_right.setPower(0);
        }

    }
    public void UpdateBelt() {
        if (gamepad1.right_bumper & gamepad1.left_bumper) {
            belt_feed.setPower(0);
        }
        else if (gamepad1.left_bumper) {
            belt_feed.setPower(-1);
        }
        else if(gamepad1.right_bumper){
            belt_feed.setPower(1);
        }
    }
    public void UpdateLift() {
        if (gamepad1.dpad_up & lift_top.getState()) {
            lift_Motor.setPower(0.6);
        }
        else if (gamepad1.dpad_down & lift_bottom_Left.getState() & lift_bottom_Right.getState()) {
            lift_Motor.setPower(-0.6);
        }
        else {
            lift_Motor.setPower(0);
        }
    }
    public void shortcuts(){
        //feeder mode
        if(gamepad1.a){
            while(lift_bottom_Left.getState() & (lift_bottom_Right.getState())) {
                lift_Motor.setPower(-0.75);
            }
            shooter_left.setVelocity(intake);
            shooter_right.setVelocity(intake);
           // belt_feed.setPower(-1);

        }
        //shooter mode
        if(gamepad1.y){
            while(lift_top.getState()) {
                lift_Motor.setPower(0.75);
            }
            shooter_right.setVelocity(shooterFar);
            shooter_left.setVelocity(shooterFar);
            belt_feed.setPower(0);
            arm_servo.setPosition(0.47);
        }
    }


        //NOTE: Eventually turn this into either me or Larson's omnidirectional drive.

    public void UpdateDriveTrain() {
         telemetry.addData("Status", "Run Time: " + runtime.toString());
         telemetry.update();
         double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
         double robotAngle = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) + Math.PI / 4;
         double rightX = gamepad1.right_stick_x;
         final double v1 = -r * Math.cos(robotAngle) + rightX;
         final double v2 = -r * Math.sin(robotAngle) - rightX;
         final double v3 = -r * Math.sin(robotAngle) + rightX;
         final double v4 = -r * Math.cos(robotAngle) - rightX;

         front_left.setPower(v1);
         front_right.setPower(v2);
         rear_left.setPower(v3);
         rear_right.setPower(v4);
        }

    }


