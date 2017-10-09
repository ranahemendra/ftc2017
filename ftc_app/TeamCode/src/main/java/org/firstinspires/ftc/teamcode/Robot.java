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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the device names have been configured on the robot:
 */
public class Robot {
    static final double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: NeveRest Motor Encoder
    static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double AUTONOMOUS_DRIVE_SPEED  = 0.5;
    static final double DRIVE_SPEED             = 0.1;
    static final double TURN_SPEED              = 0.5;

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    DcMotor motorLeft;
    DcMotor motorRight;

    DcMotor clawLifter;
    Servo leftClaw;
    Servo rightClaw;

    DcMotor telescopicArmMotor;
    CRServo relicArm;
    Servo relicHolder;

    Servo jewelKnocker;


    /* local OpMode members. */
    HardwareMap hardwareMap =  null;

    /* Constructor */
    public Robot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) throws InterruptedException {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        // Define and initialize all motors and servos
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        motorRight = hardwareMap.dcMotor.get("right_motor");
        clawLifter = hardwareMap.dcMotor.get("clawLifter");
        leftClaw = hardwareMap.servo.get("clawLeft");
        rightClaw = hardwareMap.servo.get("clawRight");
//        relicHolder = hardwareMap.servo.get("relicHolder");
        telescopicArmMotor = hardwareMap.dcMotor.get("telescopic_arm_motor");
        relicArm = hardwareMap.crservo.get("relic_arm");
        jewelKnocker = hardwareMap.servo.get("jewelKnocker");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        stopDriving();
        resetClawLifter();

        // Set the default position of all the servos.
        resetTelescopicArmMotor();
        resetRelicArm();
//        relicHolder.setPosition(0.35);
        resetJewelKnocker();

        initVuforia();
    }

    void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ARr7YvL/////AAAAGb/1qh0tGkDOrCNudt1/GAZ1QzTs3c5/pYfIEYWRNYiuJLfY1B4Hw8TCkpl/H7ehQE1duIuZ/50IqBecdogHIkW9nVApAHtZeMw52WQ/YDbc6jOtK995dYnU78p/QvZxk4Be+arr/ljflXssN42Cb8ppo2uRAo3TNPGuNOIO/lYyuxMadQoagcF+yv8TLTpJaeTNYcv4U2HZYmYO0hayv1ECBV2NuyHu8oqqLlctpAXRvVfy5SUY0ysLqNrgcqD64YBi5Yh5At296HhKtRT1KSgLu8sJMUGUtvMq+bVEwXf7MA9oLg+5nqRILvajzw/pCmNYId4IeOpHk9LwrHnN0FPcVDtWK8Gd/8FB4M0I9ozj";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    void resetEncoders() {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void setUseEncoderMode() {
        // Set wheel motors to run with encoders.
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void setRunToPositionMode() {
        // Set wheel motors to run to position.
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void driveForward(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    void turnLeft(double power) {
        motorLeft.setPower( -1 * power);
        motorRight.setPower(power);
    }

    void stopDriving() {
        driveForward(0);
    }

    boolean isBusy() {
        return motorLeft.isBusy() && motorRight.isBusy();
    }

    void resetJewelKnocker() {
        jewelKnocker.setPosition(0);
    }

    void moveJewelKnockerDown() {
        jewelKnocker.setPosition(0.6);
    }

    void clampGlyphHolder() {
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0);
    }

    void moveClawLifterUp(double time) throws InterruptedException {
        clawLifter.setPower(AUTONOMOUS_DRIVE_SPEED);
        Thread.sleep((long)(time * 1000));
        resetClawLifter();
    }

    void resetClawLifter() {
        clawLifter.setPower(0);
    }

    void resetTelescopicArmMotor() {
        telescopicArmMotor.setPower(0);
    }


        //initializing continuous rotation servo
    void resetRelicArm() throws InterruptedException {
        relicArm.setPower(-1);
        Thread.sleep(500);
        relicArmStop();
    }

        //Sets Relic Arm Power
    void relicArmUp() {
        relicArm.setPower(1);
    }

    void relicArmDown() {
        relicArm.setPower(-1);
    }

    void relicArmStop() {
        relicArm.setPower(0);
    }
}

