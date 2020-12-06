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


/**
 * Teleop made 10/22/2020
 * If you are looking at this in the far future I don't know if the config has changed any
 */

@TeleOp(name="TeleOp_V12", group="Linear Opmode")

public class TeleOpV12 extends BaseOpMode {

    // Declare OpMode members. Already declared in base op mode.

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        GetHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            UpdateShooter();
            UpdateDriveTrainSlow();
            UpdateBelt();
            updateLift();
            UpdateArmServo();
            ClawServo();
            shortcuts();
            getCenteredNavXValues();
            Kill();
        }
    }

    /*   public  void ClawServo() {

           telemetry.addData("Bool clawPos",clawPos);
           telemetry.addData("RightStickButton",gamepad1.right_stick_button);
           if(gamepad1.x) {
               if(clawPos == true & gamepad2.x){
                   claw_servo.setPosition(.7);
                   clawPos = false;
               }
               else if(clawPos == false & gamepad2.x){
                   claw_servo.setPosition(.4);
                   clawPos = true;
               }
           }
       }

     */
    public void ClawServo() {
        //open claw
        if (gamepad1.a) {
            claw_servo.setPosition(.7);
        }
        //close claw
        else if (gamepad1.x) {
            claw_servo.setPosition(.4);
        }
    }

    public void UpdateArmServo() {
        //NOTE: should eventually add in hard button stop
        //Claw Up
        if (gamepad1.left_bumper) {
            arm_servo.setPosition(0.47);
        }
        //Claw Down
        else if (gamepad1.left_trigger > 0) {
            arm_servo.setPosition(0.65);
        }
    }

    public void UpdateShooter() {
        telemetry.addData("CurrentVelocityLeft", shooter_left.getVelocity());
        telemetry.addData("CurrentVelocityRight", shooter_right.getVelocity());
        if (gamepad2.right_trigger > 0) {
            SetShooterMotors(Shoot.SHOOT_FAR);
        } else if (gamepad2.left_trigger > 0) {
            shooter_left.setVelocity(intake);
            shooter_right.setVelocity(intake);
        }
    }

    public void UpdateBelt() {
        if (gamepad1.right_bumper & gamepad1.left_bumper) {
            belt_feed.setPower(0);
        } else if (gamepad2.left_bumper) {
            belt_feed.setPower(-1);
        } else if (gamepad2.right_bumper) {
            belt_feed.setPower(1);
        }
    }

    public void updateLift() {
        double ls;
        ls = -gamepad2.left_stick_y;

        if(ls < 0){
            if(lift_bottom_Left.getState() | lift_bottom_Left.getState()){
                lift_Motor.setPower(ls);
            }
        }
        if(ls > 0){
            if(lift_top.getState()){
                lift_Motor.setPower(ls);
            }
        }
        else{
            lift_Motor.setPower(0);
        }
    }

    public void shortcuts() {
        //feeder mode
        if (gamepad2.a) {
            while (lift_bottom_Left.getState() & (lift_bottom_Right.getState())) {
                lift_Motor.setPower(-0.75);
            }
            shooter_left.setVelocity(intake);
            shooter_right.setVelocity(intake);
            // belt_feed.setPower(-1);
        }
        //shooter mode
        if (gamepad1.y) {
            while (lift_top.getState()) {
                lift_Motor.setPower(0.75);
            }
            shooter_right.setVelocity(shooterFar);
            shooter_left.setVelocity(shooterFar);
            belt_feed.setPower(0);
            arm_servo.setPosition(0.47);
        }
    }

    public void Kill() {
        if (gamepad1.b || gamepad2.b) {
            belt_feed.setPower(0);
            lift_Motor.setPower(0);
            shooter_left.setPower(0);
            shooter_right.setPower(0);
        }
    }
    public void UpdateDriveTrain() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //Larson Reversed the "y" and "x" 12/3/2020 5:25
        double rightX = gamepad1.right_stick_y;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        front_left.setPower(v1);
        front_right.setPower(v2);
        rear_left.setPower(v3);
        rear_right.setPower(v4);
    }
    public void UpdateDriveTrainSlow() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) - Math.PI / 4;
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


