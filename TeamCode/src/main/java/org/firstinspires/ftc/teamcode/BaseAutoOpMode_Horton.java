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

import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.navXPIDController;

/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */
@Disabled
public abstract class BaseAutoOpMode_Horton extends BaseOpMode_Horton {

    public ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI );

    @Override
    public void GetHardware() {
        super.GetHardware();

    }

    public enum STRAFE {
        left, right

    }

    public enum DRIVE {
        forward, reverse, stop

    }

    public enum TURN {
        left, right,

    }

    public void drive(STRAFE Direction) {
        if (Direction == STRAFE.left) {
            front_left.setPower(-.5);
            front_right.setPower(.5);
            back_left.setPower(.5);
            back_right.setPower(-.5);

        }
        if (Direction == STRAFE.right) {
            front_left.setPower(.5);
            front_right.setPower(-.5);
            back_left.setPower(-.5);
            back_right.setPower(.5);
        }
    }
    public void drive(DRIVE Direction) {
        if (Direction == DRIVE.forward) {

            front_left.setPower(.5);
            front_right.setPower(.5);
            back_left.setPower(.5);
            back_right.setPower(.5);
        }
        if (Direction == DRIVE.reverse) {
            front_left.setPower(-.5);
            front_right.setPower(-.5);
            back_left.setPower(-.5);
            back_right.setPower(-.5);
        }

        if (Direction == DRIVE.stop) {
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);
            sleep(500);
        }
    }
    public void drive(TURN Direction){
        if (Direction == TURN.right) {
            front_left.setPower(.5);
            front_right.setPower(-.5);
            back_left.setPower(.5);
            back_right.setPower(-.5);
        }

        if (Direction == TURN.left) {
            front_left.setPower(-.5);
            front_right.setPower(.5);
            back_left.setPower(-.5);
            back_right.setPower(.5);
        }
    }
    public void encoderDrive(double speed,
                                double leftInches, double rightInches,
                                double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = front_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = front_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = back_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = back_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            front_left.setTargetPosition(newFrontLeftTarget);
            front_right.setTargetPosition(newFrontRightTarget);
            back_left.setTargetPosition(newBackLeftTarget);
            back_right.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));
            back_left.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (front_left.isBusy() && front_right.isBusy() && back_left.isBusy() && back_right.isBusy()))
            {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        back_right.getCurrentPosition(),
                        back_left.getCurrentPosition(),
                        front_left.getCurrentPosition(),
                        front_right.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

            // Turn off RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             sleep(250);   // optional pause after each move
        }
    }
    public void encoderSTRAFE(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = front_left.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = front_right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = back_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackRightTarget = back_right.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);

            front_left.setTargetPosition(newFrontLeftTarget);
            front_right.setTargetPosition(newFrontRightTarget);
            back_left.setTargetPosition(newBackLeftTarget);
            back_right.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            front_left.setPower(-Math.abs(speed));
            front_right.setPower(Math.abs(speed));
            back_left.setPower(Math.abs(speed));
            back_right.setPower(-Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (front_left.isBusy() && front_right.isBusy() && back_left.isBusy() && back_right.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        back_right.getCurrentPosition(),
                        back_left.getCurrentPosition(),
                        front_left.getCurrentPosition(),
                        front_right.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

            // Turn off RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move

        }
    }
}


























