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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

public abstract class AutoOpBase extends LinearOpMode {
    Robot bot = new Robot();

    void initBot() throws InterruptedException {
        // Initialize the drive system variables.
        // The init() method of the hardware class does all the work here
        bot.init(hardwareMap, telemetry);

        bot.initVuforia();

        // Reset the encoders.
        bot.resetEncoders();

        // We will use encoders for driving distance.
        bot.setUseEncoderMode();

        // Hold glyph.
        bot.clampGlyphHolder();
        bot.moveClawLifterUp(bot.CLAW_SPEED);
        Thread.sleep((long)(time * 1000));
        bot.resetClawLifter();
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

            bot.driveForward(bot.DRIVE_SPEED -0.2);

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

    void turnLeftDistance(int forwardInches) {
        int newRightTarget;
        int newLeftTarget;

        if (opModeIsActive()) {
            newRightTarget = bot.motorRight.getCurrentPosition() + (int)(forwardInches * bot.COUNTS_PER_INCH);
            newLeftTarget = bot.motorLeft.getCurrentPosition() + (int)(forwardInches * bot.COUNTS_PER_INCH);

            bot.motorRight.setTargetPosition(newRightTarget);
            bot.motorLeft.setTargetPosition(-1 * newLeftTarget);

            bot.setRunToPositionMode();

            bot.turnLeft(bot.TURN_SPEED);

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

    void turnLeftByAngle(float angle) {
        if (angle == 0) {
            // Nothing to do.
            return;
        }

        // We will try to reduce the current angle by angle degrees. If we end up going below 0, we will use the target
        // angle as number of degrees below 0.
        float targetAngle = bot.angles.firstAngle - angle;

        if (targetAngle > 0) {
            // The target angle is > 0, reduce the first angle by turning left till we come to target angle.
            while (opModeIsActive() && bot.angles.firstAngle > targetAngle) {
                bot.turnLeft(bot.TURN_SPEED);
            }
        } else {
            // The target angle is < 0

            // First come to 0.
            while (opModeIsActive() && bot.angles.firstAngle > 0) {
                bot.turnLeft(bot.TURN_SPEED);
            }

            // Now turn further till we reach the targetAngle
            targetAngle = 360 + targetAngle;
            while (opModeIsActive() && bot.angles.firstAngle > targetAngle) {
                bot.turnLeft(bot.TURN_SPEED);
            }
        }

        bot.stopDriving();
        bot.setUseEncoderMode();
    }

    void turnRightByAngle(float angle) {
        if (angle == 0) {
            // Nothing to do.
            return;
        }

        // We will try to reduce the current angle by angle degrees. If we end up going below 0, we will use the target
        // angle as number of degrees below 0.
        float targetAngle = bot.angles.firstAngle + angle;

        if (targetAngle < 360) {
            // The target angle is < 360, increase the first angle by turning right till we come to target angle.
            while (opModeIsActive() && bot.angles.firstAngle < targetAngle) {
                bot.turnRight(bot.TURN_SPEED);
            }
        } else {
            // The target angle is > 360

            // First come to 360.
            while (opModeIsActive() && bot.angles.firstAngle < 360) {
                bot.turnRight(bot.TURN_SPEED);
            }

            // Now turn further till we reach the targetAngle
            targetAngle = targetAngle - 360;
            while (opModeIsActive() && bot.angles.firstAngle < targetAngle) {
                bot.turnRight(bot.TURN_SPEED);
            }
        }

        bot.stopDriving();
        bot.setUseEncoderMode();
    }

    /**
     * Turns the robot to an angle (relative to the position of calibration).
     * @param angle
     */
    void turnLeftToAngle(float angle) {
        if (angle == bot.angles.firstAngle) {
            // Nothing to do.
            return;
        }

        while (opModeIsActive() && bot.angles.firstAngle != angle) {
            bot.turnLeft(bot.TURN_SPEED);
        }

        bot.stopDriving();
        bot.setUseEncoderMode();
    }

    void turnRightToAngle(float angle) {
        if (angle == bot.angles.firstAngle) {
            // Nothing to do.
            return;
        }

        while (opModeIsActive() && bot.angles.firstAngle != angle) {
            bot.turnRight(bot.TURN_SPEED);
        }

        bot.stopDriving();
        bot.setUseEncoderMode();
    }

    RelicRecoveryVuMark scanVumarks() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(bot.relicTemplate);
        while (opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(bot.relicTemplate);
            idle();
        }

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) bot.relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

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

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
