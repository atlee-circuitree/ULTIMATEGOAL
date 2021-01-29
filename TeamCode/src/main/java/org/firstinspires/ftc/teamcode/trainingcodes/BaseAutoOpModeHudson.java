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

package org.firstinspires.ftc.teamcode.trainingcodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.navXPIDController;

/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */
@Disabled
public abstract class BaseAutoOpModeHudson extends BaseOpModeHudson {


    public navXPIDController yawPIDController;


    @Override

    public void GetHardware() {
        super.GetHardware();
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    public enum STRAFE {
        LEFT, RIGHT
    }

    public enum drive_train {
        FORWARDS, BACKWARDS, STOP
    }

    public enum Servo {
        LEFT, RIGHT
    }

    public void arm_servo(Servo Direction) {
        if (Direction == Servo.LEFT) {
            servo.setPosition(0.1);
        }
        if (Direction == Servo.RIGHT) {
            servo.setPosition(0.9);
        }
    }

    public void Drive(drive_train Direction) {
        if (Direction == drive_train.STOP){
            front_left.setPower(0);
            front_right.setPower(0);
            back_right.setPower(0);
            back_left.setPower(0);
            sleep(100);
        }
        if (Direction == drive_train.FORWARDS) {
            front_left.setPower(1);
            front_right.setPower(1);
            back_right.setPower(1);
            back_left.setPower(1);
        }
        if (Direction == drive_train.BACKWARDS) {
            front_left.setPower(-1);
            front_right.setPower(-1);
            back_left.setPower(-1);
            back_right.setPower(-1);
        }
    }

    public void Drive(STRAFE Direction) {
        if (Direction == STRAFE.LEFT) {
            front_left.setPower(-1);
            front_right.setPower(-1);
            back_right.setPower(1);
            back_left.setPower(-1);
        }
        if (Direction == STRAFE.RIGHT) {
            front_left.setPower(1);
            front_right.setPower(-1);
            back_left.setPower(-1);
            back_right.setPower(1);
        }
    }
}