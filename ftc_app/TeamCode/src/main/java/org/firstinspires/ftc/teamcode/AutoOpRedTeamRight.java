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
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Red Team Right", group = "Autonomous")
public class AutoOpRedTeamRight extends AutoOpBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initBot();

        // wait for the start button to be pressed.
        waitForStart();

        bot.relicTrackables.activate();
        RelicRecoveryVuMark scannedVuMark = scanVumarks(1);

        bot.moveJewelKnockerDown();
        // Wait for the servo to move.
        sleep(600);

        boolean leftJewelRed = isLeftJewelRed();
        telemetry.addData("Color", "Left Red: " + leftJewelRed);
        telemetry.update();
        sleep(500);

        if (leftJewelRed) {
            // move forward
            driveForwardDistance(3, bot.AUTO_DRIVE_SPEED_SLOW);
            bot.resetJewelKnocker();
            sleep(500);
            driveForwardDistance(20, bot.AUTO_DRIVE_SPEED_SLOW);
        } else {
            driveBackwardDistance(3, bot.AUTO_DRIVE_SPEED_SLOW);
            bot.resetJewelKnocker();
            sleep(500);
            driveForwardDistance(26, bot.AUTO_DRIVE_SPEED_SLOW);
        }

        // Go back a few inches to align with the balancing stone
        driveBackwardDistance(8, bot.AUTO_DRIVE_SPEED_NORMAL);

        // Come back to the original position.
        driveForwardDistance(6, bot.AUTO_DRIVE_SPEED_NORMAL);

        // Turn right
        turnRightToAngle(-82);

        // Back up all the way to the red line.
        driveBackwardTill(RED, bot.AUTO_DRIVE_SPEED_SLOW);

        telemetry.addData("Vumark", scannedVuMark);

        int driveDistance = 40;
        if(scannedVuMark == scannedVuMark.LEFT) {
            driveDistance -= 16;
        } else if (scannedVuMark == scannedVuMark.CENTER) {
            driveDistance -= 8;
        } else if (scannedVuMark == scannedVuMark.RIGHT) {
            // Don't do anything.
        }

        // Drive forward to the cryptobox
        driveForwardDistance(driveDistance, bot.AUTO_DRIVE_SPEED_SLOW);

        // Turn right.
        turnLeftToAngle(-4);
        driveForwardDistance(8, bot.AUTO_DRIVE_SPEED_SLOW);

        bot.unclampGlyph();

        driveBackwardDistance(3, bot.AUTO_DRIVE_SPEED_NORMAL);
        sleep(500);
        driveForwardDistance(3, bot.AUTO_DRIVE_SPEED_NORMAL);

        telemetry.addData("Time taken", getRuntime());
        telemetry.update();

        while (opModeIsActive()) {
            // Keep this going.
            idle();
        }
    }
}
