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

@Autonomous(name = "New Blue Team Right with 2 more glyphs", group = "Autonomous")
public class NewAutoOpBlueTeamRightwith2moreglyphs extends AutoOpBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initBot();

        // wait for the start button to be pressed.
        waitForStart();

        startBot();

        //get off balancing stone
        if (leftJewelRed) {
            // move forward
            driveBackwardDistance(2, bot.AUTO_DRIVE_SPEED_SLOW);
            bot.resetJewelKnocker();
            sleep(2000);
            driveBackwardDistance(20, bot.AUTO_DRIVE_SPEED_SLOW);
        } else {
            driveForwardDistance(1, bot.AUTO_DRIVE_SPEED_SLOW);
            bot.resetJewelKnocker();
            sleep(2000);
            driveBackwardDistance(23, bot.AUTO_DRIVE_SPEED_NORMAL);
        }

        // Go back a few inches to align with the balancing stone
        driveForwardDistance(18, bot.AUTO_DRIVE_SPEED_NORMAL);

        // Come back to the original position.
        driveBackwardDistance(15, bot.AUTO_DRIVE_SPEED_NORMAL);


        telemetry.addData("Vumark", scannedVuMark);

        // go to distance slightly over corresponding section of cryptobox
        int driveDistance = 4;
        if(scannedVuMark == scannedVuMark.RIGHT) {
            driveDistance += 14;
        } else if (scannedVuMark == scannedVuMark.CENTER) {
            driveDistance += 7;
        } else if (scannedVuMark == scannedVuMark.LEFT) {
            // Don't do anything.
        }

        // Drive forward to the cryptobox
        driveBackwardDistance(driveDistance, bot.AUTO_DRIVE_SPEED_SLOW);

        // Turn right.
        turnRightToAngle(-47);
        driveForwardDistance(10, bot.AUTO_DRIVE_SPEED_SLOW);

        bot.shootGlyphOut();
        bot.unclampGlyph();
        sleep(100);
        bot.stopGlyphWheels();

        driveBackwardDistance(3, bot.AUTO_DRIVE_SPEED_NORMAL);

        driveForwardDistance(8, bot.AUTO_DRIVE_SPEED_NORMAL);

        driveBackwardDistance(7, bot.AUTO_DRIVE_SPEED_NORMAL);

        telemetry.addData("Time taken", getRuntime());
        telemetry.update();


        driveBackwardDistance(10, bot.AUTO_DRIVE_SPEED_FAST);
        turnLeftToAngle(90);
        bot.moveClawLifterDown(bot.CLAW_SPEED);
        sleep(1500);
//set power to 0; stops claw lifter
        bot.resetClawLifter();
        driveForwardDistance(30, bot.AUTO_DRIVE_SPEED_FAST);
        driveForwardDistance(4, bot.AUTO_DRIVE_SPEED_SLOW);
        bot.suckGlyphIn();
        bot.clampGlyph();
//wiggling glyph in
        for(int x = 0; x < 7; x++) {
            turnRightToAngle(80);
            driveForwardDistance(5, bot.AUTO_DRIVE_SPEED_SLOW);
            turnLeftToAngle(100);
        }
        bot.stopGlyphWheels();
//clear the glyph from friction with the ground as you back up
        bot.moveClawLifterUp(bot.CLAW_SPEED);
        sleep(1500);
//set power to 0; stops claw lifter
        bot.resetClawLifter();
        sleep(1000);
        driveBackwardDistance(5, bot.AUTO_DRIVE_SPEED_SLOW);
        turnRightToAngle(-90);

//go back
        driveBackwardDistance(10,bot.AUTO_DRIVE_SPEED_SLOW);
        turnLeftToAngle(120);
        driveForwardDistance(30, bot.AUTO_DRIVE_SPEED_NORMAL);

        while (opModeIsActive()) {
            // Keep this going.
            idle();
        }


    }
}
