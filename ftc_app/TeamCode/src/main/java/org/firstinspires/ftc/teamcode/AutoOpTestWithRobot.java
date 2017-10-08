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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto Op Test", group = "Autonomous")
@Disabled
public class AutoOpTestWithRobot extends LinearOpMode {
    Robot bot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system variables.
        // The init() method of the hardware class does all the work here
        bot.init(hardwareMap);

        // Reset the encoders.
        bot.resetEncoders();

        // We will use encoders for driving distance.
        bot.setUseEncoderMode();

        // wait for the start button to be pressed.
        waitForStart();

        bot.jewelKnocker.setPosition(0.6);
        // wait for the servo to move.
        sleep(1000);

        int forwardInches = 10;
        driveForwardDistance(forwardInches);
    }

    void driveForwardDistance(int forwardInches) {
        int newRightTarget;
        int newLeftTarget;

        if (opModeIsActive()) {
            newRightTarget = bot.motorRight.getCurrentPosition() + (int)(forwardInches * bot.COUNTS_PER_INCH);
            newLeftTarget = bot.motorLeft.getCurrentPosition() + (int)(forwardInches * bot.COUNTS_PER_INCH);

            bot.motorRight.setTargetPosition(newRightTarget);
            bot.motorLeft.setTargetPosition(newLeftTarget);

            bot.setRunToPositionMode();

            bot.driveForward(bot.AUTONOMOUS_DRIVE_SPEED);

            while (opModeIsActive() && bot.isBusy()) {
                // Do nothing.
                telemetry.addData("To", "%7d, %7d", newRightTarget, newLeftTarget);
                telemetry.addData("At", "%7d, %7d", bot.motorRight.getCurrentPosition(), bot.motorLeft.getCurrentPosition());
                telemetry.update();
            }

            bot.stopDriving();
            bot.setUseEncoderMode();
        }
    }
}
