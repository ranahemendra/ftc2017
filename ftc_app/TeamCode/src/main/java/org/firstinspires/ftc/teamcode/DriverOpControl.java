package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by shreyas on 9/16/2017.
 * Updated by Aahaan on 10/04/2017.
 * Upated by Howard on 10/09/2017
 */
@TeleOp (name = "Driver Op Control", group = "Driver Op")
public class DriverOpControl extends LinearOpMode {
    Robot bot   = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
         // Initialize the drive system variables.
         // The init() method of the hardware class does all the work here
        bot.init(hardwareMap, telemetry);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            manageChassis();
            manageClaw();
//            manageTelescopicArm();
//            manageRelicArm();
//            manageRelicHolder();
            idle();
            telemetry.update();
        }
    }

    void manageChassis() {
        String motion = "Stop";
        double leftStickX = gamepad1.left_stick_x;
        double rightTrigger = gamepad1.right_trigger;
        double leftTrigger = gamepad1.left_trigger;

        if(rightTrigger > 0 && leftTrigger > 0){
            // Both triggers are pressed, don't do anything.
            return;
        }

        if (rightTrigger == 0 && leftTrigger == 0 && leftStickX == 0) {
            // Stop the robot and return.
            bot.stopDriving();
            return;
        }

        if (rightTrigger > 0) {
            bot.driveForward(rightTrigger);
        }

        if (leftTrigger > 0) {
            bot.driveBackward(leftTrigger);
        }

        if (leftStickX > 0) {
            bot.turnRight(leftStickX);
        }

        if(leftStickX < 0){
            bot.turnLeft(leftStickX);
        }

        telemetry.addData("DT: ", "LT: %7f, RT: %7f, Turn: %7f", leftTrigger, rightTrigger, leftStickX);
    }

    void manageClaw() {
        // The code to manage the claw goes here.

        // Clamp/unclamp the glyphs
        if(gamepad1.x || gamepad2.x){
            bot.clampGlyph();
        } else if (gamepad1.b || gamepad2.b){
            bot.unclampGlyph();
        }

        // Suck in or push out the glyphs
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            bot.suckGlyphIn();
        } else if (gamepad1.right_bumper || gamepad2.right_bumper){
            bot.shootGlyphOut();
        } else {
            bot.stopGlyphWheels();
        }

        // Move the clasw lifter up or down (continuous motion).
        if(gamepad1.y || gamepad2.y) {
            if (!isClawLifterOnTop()) {
                bot.moveClawLifterUp(bot.CLAW_SPEED);
                sleep(5);
            }
        } else if(gamepad1.a || gamepad2.a) {
            bot.moveClawLifterDown(bot.CLAW_SPEED);
        } else {
            bot.resetClawLifter();
        }

        // Move the claw lifter up or down by 7 inches (height of a glyph).
        if (gamepad2.dpad_up) {
            moveClawLifterUp(7, bot.CLAW_SPEED);
        } else if (gamepad2.dpad_down) {
            moveClawLifterDown(7, bot.CLAW_SPEED);
        }

        telemetry.addData("claw lifter encoder: ", bot.clawLifter.getCurrentPosition());
        telemetry.update();
    }

    boolean isClawLifterOnTop() {
        int currentPosition = bot.clawLifter.getCurrentPosition();
        double currentPositionInInches = (currentPosition / bot.CLAW_COUNTS_PER_INCH);
        return (currentPositionInInches >= 28);
    }

    void moveClawLifterUp(int upInches, double clawSpeed) {
        int newTarget;

        if (opModeIsActive()) {
            newTarget = bot.clawLifter.getCurrentPosition() + (int)(upInches * bot.CLAW_COUNTS_PER_INCH);

            bot.clawLifter.setTargetPosition(newTarget);
            bot.clawLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            bot.moveClawLifterUp(clawSpeed);

            // Keep moving the claw lifter till either the stop button is pressed, claw lifter reaches its position
            // or any of the gamepad2 or gamepad1's dpad is pressed.
            while (opModeIsActive() && bot.clawLifter.isBusy() &&
                    !(gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right ||
                            gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right ||
                            gamepad1.a || gamepad2.a || gamepad1.y || gamepad2.y)) {
                // Do nothing.
            }

            bot.resetClawLifter();
        }
    }

    void moveClawLifterDown(int downInches, double clawSpeed) {
        moveClawLifterUp(downInches, -1 * clawSpeed);
    }

//    void manageTelescopicArm() {
//        if(gamepad2.y) {
//            bot.extendTelescopicArm(bot.TELESCOPIC_ARM_SPEED);
//        } else if(gamepad2.a){
//            bot.retractTelescopicArm(bot.TELESCOPIC_ARM_SPEED);
//        } else {
//            bot.extendTelescopicArm(0);
//        }
//    }
//
//        //relic arm movement
//    void manageRelicArm() throws InterruptedException {
//        if(gamepad2.right_trigger > 0){
//            bot.relicArmUp(gamepad2.right_trigger);
//        } else if(gamepad2.left_trigger > 0){
//            bot.relicArmDown(gamepad2.left_trigger);
//        }  else {
//            bot.stopRelicArm();
//        }
//    }
//
//    void manageRelicHolder() throws InterruptedException {
//        if(gamepad2.x){
//            bot.grabRelic();
//        } else if(gamepad2.b){
//            bot.releaseRelic();
//        }
//    }
//

}