package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by shreyas on 9/16/2017.
 * Updated by Aahaan on 10/04/2017.
 */
@TeleOp (name = "Driver Op Control", group = "Driver Op")
public class DriverOpControl extends LinearOpMode {
    Robot bot   = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
         // Initialize the drive system variables.
         // The init() method of the hardware class does all the work here
        bot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            manageChassis();
            manageClaw();
            manageRelicHolder();
            idle();
        }
    }

    private void manageChassis() {
        String motion = "Stop";
        double leftStickX = gamepad1.left_stick_x;
        double rightTrigger = gamepad1.right_trigger;
        double leftTrigger = gamepad1.left_trigger;

        double leftPower = 0;
        double rightPower = 0;

        if (leftTrigger == 0 && rightTrigger != 0   ) {
            // We are going forward.
            leftPower = rightTrigger - (leftStickX > 0 ? -1 * leftStickX : 0);
            rightPower = rightTrigger + (leftStickX < 0 ? -1 * leftStickX : 0);
            motion = "Forward";
        } else if (leftTrigger != 0 && rightTrigger == 0) {
            // We are going backwards.
            leftPower = -1 * leftTrigger + (leftStickX < 0 ? leftStickX : 0);
            rightPower = -1 * leftTrigger - (leftStickX > 0 ? leftStickX : 0);
            motion = "Backwards";
        }

        bot.motorLeft.setPower(leftPower);
        bot.motorRight.setPower(rightPower);

        telemetry.addData("Motion", motion);
        telemetry.addData("Power", "Left: " + leftPower + " Right: " + rightPower);
        telemetry.addData("StickX", "Stick X Power " + leftStickX);
        telemetry.update();
    }

    private void manageClaw() {
        // The code to manage the claw goes here.
        //later, make it open a little less, so when drop the glyph in cryptobox, doesn't knock others off
        if(gamepad1.x){
            bot.leftClaw.setPosition(0.5);
            bot.rightClaw.setPosition(0);
        } else if (gamepad1.b){
            bot.leftClaw.setPosition(0.2);
            bot.rightClaw.setPosition(0.8);
        }

        if(gamepad1.y){
            bot.clawLifter.setPower(-0.5);
        } else if(gamepad1.a){
            bot.clawLifter.setPower(0.5);
        } else {
            bot.clawLifter.setPower(0);
        }
    }

    private void manageRelicHolder() {
        if(gamepad2.x){
            bot.relicHolder.setPosition(0.8);
        } else if(gamepad2.b){
            bot.relicHolder.setPosition(0.35);
        }
    }
}