package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
            manageTelescopicArm();
            manageRelicArm();
            manageRelicHolder();
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
            bot.turnRight(leftStickX/2);
        }

        if(leftStickX < 0){
            bot.turnLeft(leftStickX/2);
        }

        telemetry.addData("DT: ", "LT: %7f, RT: %7f, Turn: %7f", leftTrigger, rightTrigger, leftStickX);
    }

    void manageClaw() {
        // The code to manage the claw goes here.
        //later, make it open a little less, so when drop the glyph in cryptobox, doesn't knock others off
        if(gamepad1.x){
            bot.clampGlyph();
        } else if (gamepad1.b){
            bot.unclampGlyph();
        }

        if (gamepad1.left_bumper) {
            bot.suckGlyphIn();
        } else if (gamepad1.right_bumper){
            bot.shootGlyphOut();
        } else {
            bot.stopGlyphWheels();
        }

        if(gamepad1.y) {
            bot.moveClawLifterUp(bot.CLAW_SPEED);
        } else if(gamepad1.a){
            bot.moveClawLifterDown(bot.CLAW_SPEED);
        } else {
            bot.resetClawLifter();
        }
    }

    void manageTelescopicArm() {
        if(gamepad2.y) {
            bot.extendTelescopicArm(bot.TELESCOPIC_ARM_SPEED);
        } else {
            bot.extendTelescopicArm(0);
        }
    }

        //relic arm movement
    void manageRelicArm() throws InterruptedException {
        if(gamepad2.right_trigger > 0){
            bot.relicArmUp(gamepad2.right_trigger);
        } else if(gamepad2.left_trigger > 0){
            bot.relicArmDown(gamepad2.left_trigger);
        }  else {
            bot.stopRelicArm();
        }
    }

    void manageRelicHolder() throws InterruptedException {
        if(gamepad2.x){
            bot.grabRelic();
        } else if(gamepad2.b){
            bot.releaseRelic();
        }
    }


}