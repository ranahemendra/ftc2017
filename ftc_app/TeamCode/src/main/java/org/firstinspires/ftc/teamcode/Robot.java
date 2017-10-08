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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the device names have been configured on the robot:
 */
public class Robot {
    DcMotor motorLeft;
    DcMotor motorRight;

    DcMotor clawLifter;
    Servo leftClaw;
    Servo rightClaw;

    Servo relicHolder;

    Servo jewelKnocker;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public Robot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {
        // Save reference to Hardware map
        hwMap = hardwareMap;

        // Define and initialize all motors and servos
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        motorRight = hardwareMap.dcMotor.get("right_motor");
        clawLifter = hardwareMap.dcMotor.get("clawLifter");
        leftClaw = hardwareMap.servo.get("clawLeft");
        rightClaw = hardwareMap.servo.get("clawRight");
        relicHolder = hardwareMap.servo.get("relicHolder");
        jewelKnocker = hardwareMap.servo.get("jewelKnocker");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);
        clawLifter.setPower(0);

        // Set wheel motors to run with encoders.
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set claw lifter to run without encoders.
        clawLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the default position of all the servos.
        relicHolder.setPosition(0.35);

        jewelKnocker.setPosition(0);
    }
 }

