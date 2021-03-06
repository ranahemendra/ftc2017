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

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Blue Team Left", group = "Autonomous")
public class AutoOpBlueTeamLeft extends AutoOpBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initBot();

        // wait for the start button to be pressed.
        waitForStart();

        startBot();

        //get off balancing stone
        if (leftJewelRed) {
            // move forward
            driveBackwardDistance(3, bot.AUTO_DRIVE_SPEED_SLOW);
            bot.resetJewelKnocker();
            sleep(500);
            driveBackwardDistance(20, bot.AUTO_DRIVE_SPEED_SLOW);
        } else {
            driveForwardDistance(1, bot.AUTO_DRIVE_SPEED_SLOW);
            bot.resetJewelKnocker();
            sleep(500);
            driveBackwardDistance(26, bot.AUTO_DRIVE_SPEED_SLOW);
        }

        bot.moveClawLifterDown(bot.CLAW_SPEED);
        sleep(800);
        bot.pauseClawLifter(bot.CLAW_PAUSE_SPEED);

        // Go back a few inches to align with the balancing stone
        driveForwardDistance(14, bot.AUTO_DRIVE_SPEED_NORMAL);

        // Come back to the original position.
        driveBackwardDistance(6, bot.AUTO_DRIVE_SPEED_NORMAL);

        // Turn Left.
        turnLeftToAngle(72);

       //continue going back to align with wall
        driveBackwardDistance(20, bot.AUTO_DRIVE_SPEED_NORMAL);

        telemetry.addData("Vumark", scannedVuMark);

        int driveDistance = 12;
        if(scannedVuMark == scannedVuMark.RIGHT) {
            driveDistance += 14;
        } else if (scannedVuMark == scannedVuMark.CENTER) {
            driveDistance += 7;
        } else if (scannedVuMark == scannedVuMark.LEFT) {
            // Don't do anything.
        }

        driveForwardDistance(driveDistance, bot.AUTO_DRIVE_SPEED_SLOW);

        // Turn Left.
        turnLeftToAngle(137);
        driveForwardDistance(13, bot.AUTO_DRIVE_SPEED_SLOW);
        // Drive forward to the cryptobox

        //shoots glyph out

        bot.shootGlyphOut();
        bot.unclampGlyph();
        sleep(100);
        bot.stopGlyphWheels();

        driveBackwardDistance(3, bot.AUTO_DRIVE_SPEED_NORMAL);

        // Drive forward to the cryptobox again
        driveForwardDistance(8, bot.AUTO_DRIVE_SPEED_SLOW);

        driveBackwardDistance(4, bot.AUTO_DRIVE_SPEED_NORMAL);

        telemetry.addData("Time taken", getRuntime());
        telemetry.update();

        while (opModeIsActive()) {
            // Keep this going.
            idle();
        }
    }
}
