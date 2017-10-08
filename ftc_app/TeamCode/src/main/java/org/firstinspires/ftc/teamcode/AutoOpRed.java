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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous Op Red", group = "Autonomous")
public class AutoOpRed extends LinearOpMode {
    private Robot bot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system variables.
        // The init() method of the hardware class does all the work here
        bot.init(hardwareMap);

        // Reset the encoders.
        bot.resetEncoders();

        // We will use encoders for driving distance.
        bot.setUseEncodersMode();

        // wait for the start button to be pressed.
        waitForStart();

        int forwardInches = 20;
        driveForwardDistance(forwardInches);
//        turnRightDistance(power, distance);
//        while (opModeIsActive()) {
//            jewelKnocker.setPosition(0.6);
              // Wait for servo to move.
//            sleep(1000);
//
//            telemetry.addData("Jewel Knocker", "Position " + jewelKnocker.getPosition());
//            telemetry.addData("Motor Left", "Position " + motorLeft.getCurrentPosition());
//            telemetry.addData("Motor Right", "Position " + motorRight.getCurrentPosition());
//            telemetry.update();
//
//            idle();
//        }
    }

    private void driveForwardDistance(int forwardInches) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = bot.motorLeft.getCurrentPosition() + (int)(forwardInches * bot.COUNTS_PER_INCH);
            newRightTarget = bot.motorLeft.getCurrentPosition() + (int)(forwardInches * bot.COUNTS_PER_INCH);

            bot.motorLeft.setTargetPosition(newLeftTarget);
            bot.motorRight.setTargetPosition(newRightTarget);

            bot.setRunToPositionMode();

            bot.driveForward(bot.AUTONOMOUS_DRIVE_SPEED);

            while (opModeIsActive() && bot.isBusy()) {
                // Do nothing.
                telemetry.addData("To", "%7d, %7d", newLeftTarget, newRightTarget);
                telemetry.addData("At", "%7d, %7d", bot.motorLeft.getCurrentPosition(), bot.motorRight.getCurrentPosition());
                telemetry.update();
            }

            bot.stopDriving();
            bot.setUseEncodersMode();
        }
    }
//
//    private void turnRightDistance(double power, int distance) {
//        bot.resetEncoders();
//
//        bot.motorLeft.setTargetPosition(-1 * distance);
//        bot.motorRight.setTargetPosition(distance);
//
//        bot.runToPosition();
//
//        turnRight(power);
//
//        while (bot.motorLeft.isBusy() && bot.motorRight.isBusy()) {
//            // Do nothing.
//            telemetry.addData("Right Motor: ", bot.motorRight.getCurrentPosition());
//            telemetry.addData("Left Motor: ", bot.motorLeft.getCurrentPosition());
//            telemetry.update();
//            idle();
//        }
//
//        // stop and change mode back to normal.
//        bot.stopDriving();
//        bot.useEncoders();
//    }
//
//    private void turnRight(double power) {
//        bot.motorLeft.setPower(power);
//        bot.motorRight.setPower(-1 * power);
//    }
}
