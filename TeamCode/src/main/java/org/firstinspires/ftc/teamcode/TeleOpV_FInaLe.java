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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Simon's tiny brain couldn't handle field oriented drive so he made this
 */

@TeleOp(name="TeleOpV_FiNaLE", group="Linear Opmode")

public class TeleOpV_FInaLe extends BaseAutoOpMode {

    // declare motor speed variables
    double FR; double FL; double RR; double RL;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;
    // operational constants
    double joyScale = 1;
    double motorMax = 1; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode


    boolean clawPos = true;
    double LiftM;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        GetHardware();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        arm_servo.setPosition(0.55);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //getCenteredNavXValues();
            //getCannonNavXValues();
            Kill();
            UpdateShooter();
            UpdateDriveTrainSlow();
            UpdateBelt();
            UpdateLift();
            UpdateArmServo();
            ClawServo();
            //shortcuts();
            shortcutsV2();
            telemetry.addData("LiftM", LiftM);
            telemetry.addData("Lift motor encoder", lift_Motor.getCurrentPosition());
            telemetry.update();

        }
    }
    public  void ClawServo() {

        if(gamepad1.x) {
            if(clawPos == true & gamepad1.x){
                claw_servo.setPosition(.7);
                clawPos = false;
                sleep(200);
            }
            else if(clawPos == false & gamepad1.x){
                claw_servo.setPosition(.3);
                clawPos = true;
                sleep(200);
            }
        }

    }

    public void UpdateArmServo() {
        //NOTE: should eventually add in hard button stop
        //arm up
        if(gamepad1.left_bumper){
            arm_servo.setPosition(0.55); //0.463
        }
        //arm down
        else if(gamepad1.left_trigger > 0.2) {
            arm_servo.setPosition(0.65);
        }
    }

    public void UpdateShooter(){
        //telemetry.addData("TargetVelocity", "Running to %7d", 2500);
        telemetry.addData("CurrentVelocityLeft", shooter_left.getVelocity());
        telemetry.addData("CurrentVelocityRight",shooter_right.getVelocity());
        if(gamepad2.right_trigger > 0) {
            shooter_right.setVelocity(shooterFar);
            shooter_left.setVelocity(shooterFar);
        }
        else if (gamepad2.left_trigger > 0) {
            shooter_left.setVelocity(intake);
            shooter_right.setVelocity(intake);
        }
        else if(gamepad2.y){
            //Stop motors
            shooter_left.setPower(0);
            shooter_right.setPower(0);
        }

    }
    public void UpdateBelt() {
        if (gamepad2.right_bumper & gamepad2.left_bumper) {
            belt_feed.setPower(0);
        }
        else if (gamepad2.left_bumper) {
            belt_feed.setPower(-1);
        }
        else if(gamepad2.right_bumper){
            belt_feed.setPower(1);
        }
    }
    public void UpdateLift() {

        LiftM = 0;

        Y2 = -gamepad2.left_stick_y * joyScale;

        LiftM += Y2;

        LiftM = Math.max(-motorMax, Math.min(LiftM,motorMax));

        if(Y2 < 0){
            if(lift_bottom_Left.getState() | lift_bottom_Right.getState()){
                lift_Motor.setPower(LiftM);
            }
        }
        else if(Y2 > 0){
            if(lift_top.getState()){
                lift_Motor.setPower(LiftM);
            }
        }
        else{
            lift_Motor.setPower(0);
        }

    }
    public void shortcuts(){
        //feeder mode
        if(gamepad2.a){
            if(lift_bottom_Left.getState() | (lift_bottom_Right.getState())) { //changed from while to if 1/25/2021
                lift_Motor.setPower(-0.7);
                UpdateDriveTrain();
            }
            lift_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(lift_Motor.getCurrentPosition() < 50){ //changed from while to if 1/25/2021
                lift_Motor.setPower(0.7);
            }
            lift_Motor.setPower(0);
            shooter_left.setVelocity(intake);
            shooter_right.setVelocity(intake);
            belt_feed.setPower(-1);

        }
        //shooter mode
        if(gamepad2.y){
            if(lift_top.getState()) { //changed from while to if 1/25/2021
                lift_Motor.setPower(0.7);
                UpdateDriveTrain();
            }
            lift_Motor.setPower(0);
            shooter_right.setVelocity(shooterFar);
            shooter_left.setVelocity(shooterFar);
            belt_feed.setPower(0);
        }
    }

    public void shortcutsV2() throws InterruptedException {
        if(gamepad2.a){
            telemetry.addLine("Rotating...");
            telemetry.update();
            PIDrotate(0,1.5);
        }
    }

    public void Kill() {
        if (gamepad1.b | gamepad2.b) {
            //belt_feed.setPower(0);
            lift_Motor.setPower(0);
            shooter_left.setPower(0);
            shooter_right.setPower(0);
        }
    }


    //NOTE: Eventually turn this into either me or Larson's omnidirectional drive.
    public void UpdateDriveTrain(){
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //Larson Reversed the "y" and "x" 12/3/2020 5:25
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        front_left.setPower(v1*2);
        front_right.setPower(v2*2);
        rear_left.setPower(v3*2);
        rear_right.setPower(v4*2);
    }
    public void UpdateDriveTrainSlow() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //Larson Reversed the "y" and "x" 12/3/2020 5:25
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;


        if (gamepad1.right_stick_button) {
            front_left.setPower(v1/2);
            front_right.setPower(v2/2);
            rear_left.setPower(v3/2);
            rear_right.setPower(v4/2);
        }
        else if (gamepad1.left_stick_button) {
            front_left.setPower(v1 / 2);
            front_right.setPower(v2 / 2);
            rear_left.setPower(v3 / 2);
            rear_right.setPower(v4 / 2);
        }
        else {
            front_left.setPower(v1*2);
            front_right.setPower(v2*2);
            rear_left.setPower(v3*2);
            rear_right.setPower(v4*2);
        }
    }
    }


