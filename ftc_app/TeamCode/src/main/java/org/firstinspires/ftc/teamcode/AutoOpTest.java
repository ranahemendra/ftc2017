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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Test", group = "Autonomous")
@Disabled
public class AutoOpTest extends AutoOpBase {

    float startAngle;
    float endAngle;
    int startPositionLeft;
    int startPositionRight;
    int endPositionLeft;
    int endPositionRight;

    @Override
    public void runOpMode() throws InterruptedException {
        initBot();

        // wait for the start button to be pressed.
        waitForStart();
        startAngle = bot.getCurrentAngle();
        endAngle = bot.getCurrentAngle();
        startPositionLeft = bot.motorLeft.getCurrentPosition();
        startPositionRight = bot.motorRight.getCurrentPosition();
        endPositionLeft = startPositionLeft;
        endPositionRight = startPositionRight;

        telemetry.addData("Angle: ", "Start: " + startAngle + " End: " + endAngle);
        telemetry.addData("Start Position: ", "Left: " + startPositionLeft + " Right: " + startPositionRight);
        telemetry.addData("End Position: ", "Left: " + endPositionLeft + " Right: " + endPositionRight);
        telemetry.update();

        driveForwardDistance(60, bot.AUTO_DRIVE_SPEED_NORMAL);

        endAngle = bot.getCurrentAngle();
        endPositionLeft = bot.motorLeft.getCurrentPosition();
        endPositionRight = bot.motorRight.getCurrentPosition();

        telemetry.addData("Angle: ", "Start: " + startAngle + " End: " + endAngle);
        telemetry.addData("Start Position: ", "Left: " + startPositionLeft + " Right: " + startPositionRight);
        telemetry.addData("End Position: ", "Left: " + endPositionLeft + " Right: " + endPositionRight);
        telemetry.update();

        turnRightToAngle(-84);

        while (opModeIsActive()) {
            // Keep this going.
            idle();
        }
    }

    void driveForwardDistance(int forwardInches, double driveSpeed) {
        float startAngle = bot.getCurrentAngle();

        if (opModeIsActive()) {
            int newRightTarget = bot.motorRight.getCurrentPosition() + (int) (forwardInches * bot.COUNTS_PER_INCH);
            int newLeftTarget = bot.motorLeft.getCurrentPosition() + (int) (forwardInches * bot.COUNTS_PER_INCH);

            int originalRightTarget = newRightTarget;
            int originalLeftTarget = newLeftTarget;

            bot.motorRight.setTargetPosition(newRightTarget);
            bot.motorLeft.setTargetPosition(newLeftTarget);

            bot.setRunToPositionMode();

            bot.driveForward(driveSpeed);

            while (opModeIsActive() && bot.isBusy()) {
                float newAngle = bot.getCurrentAngle();
                if (newAngle != startAngle && Math.abs(newAngle - startAngle) > 1) {
                    if (newAngle < startAngle) {
                        // bot veered right. give less power to the left motor.
                        bot.driveForward(driveSpeed, bot.LEAN_LEFT);
                    } else {
                        // bot veered left. give less power to the right motor.
                        bot.driveForward(driveSpeed, bot.LEAN_RIGHT);
                    }
                } else {
                    bot.driveForward(driveSpeed, bot.GO_STRAIGHT);
                }

                endAngle = bot.getCurrentAngle();
                endPositionLeft = bot.motorLeft.getCurrentPosition();
                endPositionRight = bot.motorRight.getCurrentPosition();

                telemetry.addData("Angle: ", "Start: " + startAngle + " End: " + endAngle);
                telemetry.addData("Start Position: ", "Left: " + startPositionLeft + " Right: " + startPositionRight);
                telemetry.addData("End Position: ", "Left: " + endPositionLeft + " Right: " + endPositionRight);
                telemetry.update();
            }

            bot.stopDriving();
            bot.setUseEncoderMode();
        }
    }

    void driveBackwardDistance(int backwardInches, double driveSpeed) {
        float startAngle = bot.getCurrentAngle();

        if (opModeIsActive()) {
            int newRightTarget = bot.motorRight.getCurrentPosition() + (int) (backwardInches * bot.COUNTS_PER_INCH);
            int newLeftTarget = bot.motorLeft.getCurrentPosition() + (int) (backwardInches * bot.COUNTS_PER_INCH);

            int originalRightTarget = newRightTarget;
            int originalLeftTarget = newLeftTarget;

            bot.motorRight.setTargetPosition(newRightTarget);
            bot.motorLeft.setTargetPosition(newLeftTarget);

            bot.setRunToPositionMode();

            bot.driveBackward(driveSpeed);

            while (opModeIsActive() && bot.isBusy()) {
                float newAngle = bot.getCurrentAngle();
                if (newAngle != startAngle && Math.abs(newAngle - startAngle) > 1) {
                    if (newAngle < startAngle) {
                        // bot veered right. give less power to the left motor.
                        bot.driveBackward(driveSpeed, bot.LEAN_LEFT);
                    } else {
                        // bot veered left. give less power to the right motor.
                        bot.driveBackward(driveSpeed, bot.LEAN_RIGHT);
                    }
                } else {
                    bot.driveBackward(driveSpeed, bot.GO_STRAIGHT);
                }

                endAngle = bot.getCurrentAngle();
                endPositionLeft = bot.motorLeft.getCurrentPosition();
                endPositionRight = bot.motorRight.getCurrentPosition();

                telemetry.addData("Angle: ", "Start: " + startAngle + " End: " + endAngle);
                telemetry.addData("Start Position: ", "Left: " + startPositionLeft + " Right: " + startPositionRight);
                telemetry.addData("End Position: ", "Left: " + endPositionLeft + " Right: " + endPositionRight);
                telemetry.update();
            }

            bot.stopDriving();
            bot.setUseEncoderMode();
        }
    }
}
