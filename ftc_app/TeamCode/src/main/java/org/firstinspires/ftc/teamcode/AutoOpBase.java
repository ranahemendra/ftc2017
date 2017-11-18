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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

public abstract class AutoOpBase extends LinearOpMode {

    static final int RED = 0;
    static final int BLUE = 1;

    Robot bot = new Robot();
    RelicRecoveryVuMark scannedVuMark;
    boolean leftJewelRed;

    void initBot() throws InterruptedException {
        // Initialize the drive system variables.
        // The init() method of the hardware class does all the work here
        bot.init(hardwareMap, telemetry);

        bot.initVuforia();

        // Reset the encoders.
        bot.resetEncoders();

        // We will use encoders for driving distance.
        bot.setUseEncoderMode();
    }

    void startBot() throws InterruptedException {
        bot.moveJewelKnockerDown();
        bot.relicTrackables.activate();
        scannedVuMark = scanVumarks(1);

        leftJewelRed = isLeftJewelRed();

        telemetry.addData("Vumark", scannedVuMark);
        telemetry.addData("Color", "Left Red: " + leftJewelRed);
        telemetry.update();

        // Hold glyph.
        bot.clampGlyph();
        bot.suckGlyphIn();
        sleep(500);
        bot.stopGlyphWheels();
        bot.moveClawLifterUp(bot.CLAW_SPEED);
        sleep(2500);
        bot.resetClawLifter();
    }

    void driveForwardDistance(double maintainAngle, int forwardInches, double driveSpeed) {
        if (opModeIsActive()) {
            int newRightTarget = bot.motorRight.getCurrentPosition() + (int) (forwardInches * bot.COUNTS_PER_INCH);
            int newLeftTarget = bot.motorLeft.getCurrentPosition() + (int) (forwardInches * bot.COUNTS_PER_INCH);

            bot.motorRight.setTargetPosition(newRightTarget);
            bot.motorLeft.setTargetPosition(newLeftTarget);

            bot.setRunToPositionMode();

            bot.driveForward(driveSpeed);

            while (opModeIsActive() && bot.isBusy()) {
                float newAngle = bot.getCurrentAngle();
                if (newAngle != maintainAngle && Math.abs(newAngle - maintainAngle) > 1) {
                    if (newAngle < maintainAngle) {
                        // bot veered right. give less power to the left motor.
                        bot.driveForward(driveSpeed, bot.LEAN_LEFT);
                    } else {
                        // bot veered left. give less power to the right motor.
                        bot.driveForward(driveSpeed, bot.LEAN_RIGHT);
                    }
                } else {
                    bot.driveForward(driveSpeed, bot.GO_STRAIGHT);
                }

                double endAngle = bot.getCurrentAngle();

                telemetry.addData("Angle: ", "Start: " + maintainAngle + " End: " + endAngle);
                telemetry.update();
            }

            bot.stopDriving();
            bot.setUseEncoderMode();
        }
    }

    void driveForwardDistance(int forwardInches, double driveSpeed) {
        int newRightTarget;
        int newLeftTarget;

        if (opModeIsActive()) {
            newRightTarget = bot.motorRight.getCurrentPosition() + (int)(forwardInches * bot.COUNTS_PER_INCH);
            newLeftTarget = bot.motorLeft.getCurrentPosition() + (int)(forwardInches * bot.COUNTS_PER_INCH);

            bot.motorRight.setTargetPosition(newRightTarget);
            bot.motorLeft.setTargetPosition(newLeftTarget);

            bot.setRunToPositionMode();

            bot.driveForward(driveSpeed);

            while (opModeIsActive() && bot.isBusy()) {
                // Do nothing.
            }

            bot.stopDriving();
            bot.setUseEncoderMode();
        }
    }

    void driveBackwardDistance(double maintainAngle, int backwardInches, double driveSpeed) {
        backwardInches = -1 * backwardInches;

        if (opModeIsActive()) {
            int newRightTarget = bot.motorRight.getCurrentPosition() + (int) (backwardInches * bot.COUNTS_PER_INCH);
            int newLeftTarget = bot.motorLeft.getCurrentPosition() + (int) (backwardInches * bot.COUNTS_PER_INCH);

            bot.motorRight.setTargetPosition(newRightTarget);
            bot.motorLeft.setTargetPosition(newLeftTarget);

            bot.setRunToPositionMode();
            bot.driveBackward(driveSpeed);

            while (opModeIsActive() && bot.isBusy()) {
                float newAngle = bot.getCurrentAngle();
                if (newAngle != maintainAngle && Math.abs(newAngle - maintainAngle) > 1) {
                    if (newAngle < maintainAngle) {
                        // bot veered right. give less power to the left motor.
                        bot.driveBackward(driveSpeed, bot.LEAN_LEFT);
                    } else {
                        // bot veered left. give less power to the right motor.
                        bot.driveBackward(driveSpeed, bot.LEAN_RIGHT);
                    }
                } else {
                    bot.driveBackward(driveSpeed, bot.GO_STRAIGHT);
                }

                double endAngle = bot.getCurrentAngle();

                telemetry.addData("Angle: ", "Start: " + maintainAngle + " End: " + endAngle);
                telemetry.update();
            }

            bot.stopDriving();
            bot.setUseEncoderMode();
        }
    }

    void driveBackwardDistance(int backwardInches, double driveSpeed) {
        driveForwardDistance(-1 * backwardInches, driveSpeed);
    }

    /**
     * Turns the robot to an angle (relative to the position of calibration).
     * @param targetAngle
     */
    void turnLeftToAngle(double targetAngle) {
        float currentAngle = bot.getCurrentAngle();
        if (targetAngle == currentAngle) {
            // Nothing to do.
            return;
        }

        // Check if the target angle is to the left of current angle
        if (targetAngle > currentAngle) {
            turnLeftToAngleLocal(targetAngle);
        } else {
            // We will be crossing over the 180 mark.

            // Let's first go to 180.
            turnLeftToAngleLocal(180);

            // Now let's go to the actual target angle.
            turnLeftToAngleLocal(targetAngle);
        }

        bot.stopDriving();
        bot.setUseEncoderMode();
    }

    private void turnLeftToAngleLocal(double targetAngle) {
        float currentAngle = bot.getCurrentAngle();
        if (opModeIsActive() && currentAngle < targetAngle) {
            bot.autoOpTurnLeft(bot.AUTO_TURN_SPEED_NORMAL);
        }

        while (opModeIsActive() && targetAngle > currentAngle) {
            // Keep turning.
            currentAngle = bot.getCurrentAngle();
            telemetry.addData("Angle", currentAngle);
            telemetry.update();
        }

        bot.stopDriving();
    }

    /**
     * Turns the robot to an targetAngle (relative to the position of calibration).
     * @param targetAngle
     */
    void turnRightToAngle(double targetAngle) {
        float currentAngle = bot.getCurrentAngle();
        if (targetAngle == currentAngle) {
            // Nothing to do.
            return;
        }

        // Check if the target angle is to the left of current angle
        if (targetAngle < currentAngle) {
            turnRightToAngleLocal(targetAngle);
        } else {
            // We will be crossing over the 180 mark.

            // Let's first go to -179.99.
            turnRightToAngleLocal(-179.99);

            // Now let's go to the actual target angle.
            turnRightToAngleLocal(targetAngle);
        }

        bot.stopDriving();
        bot.setUseEncoderMode();
    }

    private void turnRightToAngleLocal(double targetAngle) {
        float currentAngle = bot.getCurrentAngle();
        if (opModeIsActive() && targetAngle < currentAngle) {
            bot.turnRight(bot.AUTO_TURN_SPEED_NORMAL);
        }

        while (opModeIsActive() && targetAngle < currentAngle) {
            // Keep turning.
            currentAngle = bot.getCurrentAngle();
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.update();
        }

        bot.stopDriving();
    }

    RelicRecoveryVuMark scanVumarks(int timeoutSeconds) {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(bot.relicTemplate);
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN && (System.currentTimeMillis() - startTime) < timeoutSeconds * 1000) {
            vuMark = RelicRecoveryVuMark.from(bot.relicTemplate);
            idle();
        }

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) bot.relicTemplate.getListener()).getPose();

            /* We further illustrate how to decompose the pose into useful rotational and
             * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;

            }
        } else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();
        return vuMark;
    }

    boolean isLeftJewelRed() {
        ColorSensor sensor = bot.getJewelColorSensorOutput();
        telemetry.addData("Red", sensor.red());
        telemetry.addData("Blue", sensor.blue());

        if (sensor.red() > sensor.blue()) {
            return true;
        } else {
            return false;
        }
    }

    void driveBackwardTill(int color, double power) {
        if (opModeIsActive()) {
            bot.driveBackward(power);
        }

        while (opModeIsActive() && !detectedLine(color)) {
            // Keep doing this.
        }

        bot.stopDriving();
        bot.setUseEncoderMode();
    }

    void driveBackwardTill(double maintainAngle, int color, double power) {
        if (opModeIsActive()) {
            bot.driveBackward(power);

            while (opModeIsActive() && !detectedLine(color)) {
                float newAngle = bot.getCurrentAngle();
                if (newAngle != maintainAngle && Math.abs(newAngle - maintainAngle) > 1) {
                    if (newAngle < maintainAngle) {
                        // bot veered right. give less power to the left motor.
                        bot.driveBackward(power, bot.LEAN_LEFT);
                    } else {
                        // bot veered left. give less power to the right motor.
                        bot.driveBackward(power, bot.LEAN_RIGHT);
                    }
                } else {
                    bot.driveBackward(power, bot.GO_STRAIGHT);
                }

                double endAngle = bot.getCurrentAngle();

                telemetry.addData("Angle: ", "Start: " + maintainAngle + " End: " + endAngle);
                telemetry.update();
            }

            bot.stopDriving();
            bot.setUseEncoderMode();
        }
    }

    boolean detectedLine(int color) {
        ColorSensor sensor = bot.getLineColorSensorOutput();

        if (color == RED && sensor.red() > 50) {
            return true;
        }

        if (color == BLUE && sensor.blue() > 50) {
            return true;
        }

        return false;
    }
}
