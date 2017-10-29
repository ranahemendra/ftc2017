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

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

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

    static final double AUTO_DRIVE_SPEED_SLOW   = 0.25;
    static final double AUTO_DRIVE_SPEED_NORMAL = 0.5;
    static final double AUTO_DRIVE_SPEED_FAST   = 1.0;
    static final double CLAW_SPEED              = 0.7;
    static final double TELESCOPIC_ARM_SPEED    = 1;
    static final double AUTO_TURN_SPEED_SLOW    = 0.15;
    static final double AUTO_TURN_SPEED_NORMAL  = 0.3;

    static final int LEAN_LEFT                  = -1;
    static final int GO_STRAIGHT                = 0;
    static final int LEAN_RIGHT                 = 1;
    static final double LEAN_CORRECTION         = 0.20;

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    DcMotor motorLeft;
    DcMotor motorRight;

    DcMotor clawLifter;
    Servo leftClaw;
    Servo rightClaw;

    DcMotor telescopicArmMotor;
    DcMotor relicArm;
    Servo relicHolder;

    Servo jewelKnocker;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float jewelColorSensorHsvValues[] = {0F, 0F, 0F};
    ColorSensor jewelColorSensor;

    float lineColorSensorHsvValues[] = {0F, 0F, 0F};
    ColorSensor lineColorSensor;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    Telemetry telemetry;


    /* local OpMode members. */
    HardwareMap hardwareMap =  null;

    /* Constructor */
    public Robot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Define and initialize all motors and servos
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        motorRight = hardwareMap.dcMotor.get("right_motor");
        clawLifter = hardwareMap.dcMotor.get("clawLifter");
        leftClaw = hardwareMap.servo.get("clawLeft");
        rightClaw = hardwareMap.servo.get("clawRight");
        telescopicArmMotor = hardwareMap.dcMotor.get("telescopic_arm_motor");

        relicArm = hardwareMap.dcMotor.get("relic_arm");
        relicHolder = hardwareMap.servo.get("relic_holder");

        jewelKnocker = hardwareMap.servo.get("jewelKnocker");

        // get a reference to the jewel color sensor.
        jewelColorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // get a reference to line color sensor.
        lineColorSensor = hardwareMap.get(ColorSensor.class, "line_color_sensor");

        // Set all motors to zero power
        stopDriving();
        resetClawLifter();
        resetGlyphHolder();

        // Set the default position of all the servos.
        resetTelescopicArm();
        stopRelicArm();
        resetRelicHolder();

        resetJewelKnocker();

        initGyro();
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

    void initGyro() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
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
        driveForward(power, power);
    }

    void driveForward(double power, int lean) {
        if (lean == GO_STRAIGHT) {
            driveForward(power, power);
        } else if (lean == LEAN_LEFT) {
            driveForward(power - power * LEAN_CORRECTION, power);
        } else if (lean == LEAN_RIGHT) {
            driveForward(power, power - power * LEAN_CORRECTION);
        }
    }

    void driveForward(double leftPower, double rightPower) {
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    void driveBackward(double power, int lean) {
        if (lean == GO_STRAIGHT) {
            driveBackward(power, power);
        } else if (lean == LEAN_LEFT) {
            driveBackward(power - power * LEAN_CORRECTION, power);
        } else if (lean == LEAN_RIGHT) {
            driveBackward(power, power - power * LEAN_CORRECTION);
        }
    }

    void driveBackward(double leftPower, double rightPower) {
        driveForward(-leftPower, -rightPower);
    }

    void driveBackward(double power) {
        driveForward(-power);
//        motorLeft.setDirection(DcMotor.Direction.FORWARD);
//        motorRight.setDirection(DcMotor.Direction.REVERSE);
//        motorLeft.setPower(power);
//        motorRight.setPower(power);
    }

    void turnLeft(double power) {
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setPower(-power);
        motorRight.setPower(-power);
    }

    void autoOpTurnLeft(double power) {
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    void turnRight(double power) {
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setPower(power);
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
        jewelKnocker.setPosition(0.8);
    }

    void moveClawLifterUp(double power) {
        clawLifter.setPower(power);
    }

    void moveClawLifterDown(double power) {
        clawLifter.setPower(-1 * power);
    }

    void resetClawLifter() {
        clawLifter.setPower(0);
    }

    void resetGlyphHolder() {
        leftClaw.setPosition(0);
        rightClaw.setPosition(1);
    }

    void clampGlyph() {
        leftClaw.setPosition(1);
        rightClaw.setPosition(0);
    }

    void unclampGlyph() {
//        resetGlyphHolder();
        leftClaw.setPosition(0.1);
        rightClaw.setPosition(0.9);
    }

    void resetTelescopicArm() {
        telescopicArmMotor.setPower(0);
    }

    void extendTelescopicArm(double power) {
        telescopicArmMotor.setPower(power);
    }

    // Initializing continuous rotation servo
    void stopRelicArm() {
        relicArm.setPower(0);
    }

    // Sets Relic Arm Power
    void relicArmUp(double power) throws InterruptedException {
        relicArm.setDirection(DcMotor.Direction.FORWARD);
        relicArm.setPower(power);
    }

    void relicArmDown(double power) {
        relicArm.setDirection(DcMotor.Direction.REVERSE);
        relicArm.setPower(power);
    }

    void resetRelicHolder() {
        relicHolder.setPosition(0.5);
    }

    void grabRelic() throws InterruptedException{
        double position = relicHolder.getPosition();
        if(position < 1){
            relicHolder.setPosition(position + 0.05);
            Thread.sleep(50);
        }
    }

    void releaseRelic() throws InterruptedException{
        double position = relicHolder.getPosition();
        if(position > 0){
            relicHolder.setPosition(position - 0.05);
            Thread.sleep(50);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    ColorSensor getJewelColorSensorOutput() {
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (jewelColorSensor.red() * SCALE_FACTOR),
                (int) (jewelColorSensor.green() * SCALE_FACTOR),
                (int) (jewelColorSensor.blue() * SCALE_FACTOR),
                jewelColorSensorHsvValues);

        return jewelColorSensor;
    }

    ColorSensor getLineColorSensorOutput() {
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (lineColorSensor.red() * SCALE_FACTOR),
                (int) (lineColorSensor.green() * SCALE_FACTOR),
                (int) (lineColorSensor.blue() * SCALE_FACTOR),
                lineColorSensorHsvValues);

        return lineColorSensor;
    }

    public float getCurrentAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public Acceleration getNewGravity() {
        return imu.getGravity();
    }
}

