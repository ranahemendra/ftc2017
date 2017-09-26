package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by shreyas, howard on 9/16/2017.
 */

@TeleOp (name = "Driver Op Control", group = "Driver Op")
public class DriverOpControl extends LinearOpMode {
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor clawLifter;

    private Servo leftClaw;
    private Servo rightClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        motorRight = hardwareMap.dcMotor.get("right_motor");
        clawLifter = hardwareMap.dcMotor.get("clawLifter");
        leftClaw = hardwareMap.servo.get("clawLeft");
        rightClaw = hardwareMap.servo.get("clawRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            manageChassis();
            manageClaw();
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
        if (leftTrigger == 0 && rightTrigger != 0) {
            // We are going forward.
            leftPower = -1 * rightTrigger - (leftStickX > 0 ? -1 * leftStickX : 0);
            rightPower = -1 * rightTrigger + (leftStickX < 0 ? -1 * leftStickX : 0);
            motion = "Forward";
        } else if (leftTrigger != 0 && rightTrigger == 0) {
            // We are going backwards.
            leftPower = leftTrigger + (leftStickX < 0 ? leftStickX : 0);
            rightPower = leftTrigger - (leftStickX > 0 ? leftStickX : 0);
            motion = "Backwards";
        }

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);

        telemetry.addData("Motion", motion);
        telemetry.addData("Power", "Left: " + leftPower + " Right: " + rightPower);
        telemetry.addData("StickX", "Stick X Power " + leftStickX);
        telemetry.update();
    }

    private void manageClaw() {
        // The code to manage the claw goes here.
        if(gamepad1.x){
            leftClaw.setPosition(0.5);
            rightClaw.setPosition(0);
        } else if (gamepad1.b){
            leftClaw.setPosition(0);
            rightClaw.setPosition(0.5);
        }

        if(gamepad1.y){
            clawLifter.setPower(-0.5);
        } else if(gamepad1.a){
            clawLifter.setPower(0.5);
        } else {
            clawLifter.setPower(0);
        }
    }
}