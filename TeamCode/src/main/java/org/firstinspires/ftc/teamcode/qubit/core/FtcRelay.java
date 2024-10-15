/* Copyright (c) 2023 The Qubit Bot. All rights reserved.
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

package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the robot intake.
 */
public class FtcRelay extends FtcSubSystem {
    private static final String TAG = "FtcRelay";
    public static final String ARM_MOTOR_NAME = "armMotor";
    public static final int ARM_FORWARD_POSITION = 0;
    public static final int ARM_BACKWARD_POSITION = 2800;
    public static final double ARM_BACKWARD_POWER = FtcMotor.MIN_POWER;
    public static final double ARM_FORWARD_POWER = FtcMotor.MAX_POWER;
    public static final String RACK_N_PINION_SERVO_NAME = "rackAndPinionServo";
    public static final double RACK_N_PINION_PUSH_POWER = 1.0;
    public static final double RACK_N_PINION_PULL_POWER = 0.0;
    public static final double RACK_N_PINION_STOP_POWER = FtcServo.MID_POSITION;
    public static final String SPIN_SERVO_NAME = "spinServo";
    public static final double SPIN_IN_POWER = 0.75;
    public static final double SPIN_OUT_POWER = 0.25;
    public static final double SPIN_HOLD_POWER = 0.55;
    public static final double SPIN_STOP_POWER = FtcServo.MID_POSITION;
    private final boolean relayEnabled = true;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    public FtcMotor armMotor = null;
    public FtcServo rackNPinionServo = null;
    public FtcServo spinServo = null;

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (relayEnabled) {
            armMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, ARM_MOTOR_NAME));
            armMotor.setDirection(DcMotor.Direction.REVERSE);
            armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            rackNPinionServo = new FtcServo(hardwareMap.get(Servo.class, RACK_N_PINION_SERVO_NAME));
            rackNPinionServo.setDirection(Servo.Direction.FORWARD);

            spinServo = new FtcServo(hardwareMap.get(Servo.class, SPIN_SERVO_NAME));
            spinServo.setDirection(Servo.Direction.REVERSE);

            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Operates the intake using the gamePads.
     * Left bumper -> rotate out
     * Left trigger -> rotate in
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2) {
        FtcLogger.enter();
        if (gamePad1.right_trigger >= 0.5 || gamePad2.right_trigger >= 0.5) {
            spinIn();
        } else if (gamePad1.right_bumper || gamePad2.right_bumper) {
            spinOut();
        } else {
            spinHold();
        }

        if (gamePad1.dpad_up || gamePad2.dpad_up) {
            rnpPushOut();
        } else if (gamePad1.dpad_down || gamePad2.dpad_down) {
            rnpPullIn();
        } else {
            rnpStop();
        }

        if (gamePad1.left_trigger >= 0.5 || gamePad2.left_trigger >= 0.5) {
            armForward();
        } else if (gamePad1.left_bumper || gamePad2.left_bumper) {
            armBackward();
        }

        FtcLogger.exit();
    }

    private void rnpPullIn() {
        FtcLogger.enter();
        if (relayEnabled) {
            rackNPinionServo.setPosition(RACK_N_PINION_PULL_POWER);
        }

        FtcLogger.exit();
    }

    private void rnpPushOut() {
        FtcLogger.enter();
        if (relayEnabled) {
            rackNPinionServo.setPosition(RACK_N_PINION_PUSH_POWER);
        }

        FtcLogger.exit();
    }

    private void rnpStop() {
        FtcLogger.enter();
        if (relayEnabled) {
            rackNPinionServo.setPosition(RACK_N_PINION_STOP_POWER);
        }

        FtcLogger.exit();
    }

    private void armForward() {
        FtcLogger.enter();
        if (relayEnabled) {
            armMotor.setTargetPosition(ARM_FORWARD_POSITION);
            armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            armMotor.setPower(ARM_FORWARD_POWER);
        }

        FtcLogger.exit();
    }

    private void armBackward() {
        FtcLogger.enter();
        if (relayEnabled) {
            armMotor.setTargetPosition(ARM_BACKWARD_POSITION);
            armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            armMotor.setPower(ARM_BACKWARD_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Rotate inwards to intake the object.
     */
    private void spinIn() {
        FtcLogger.enter();
        if (relayEnabled) {
            spinServo.setPosition(SPIN_IN_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Rotate outwards to outtake the object.
     */
    private void spinOut() {
        FtcLogger.enter();
        if (relayEnabled) {
            spinServo.setPosition(SPIN_OUT_POWER);
        }

        FtcLogger.exit();
    }

    private void spinHold() {
        FtcLogger.enter();
        if (relayEnabled) {
            spinServo.setPosition(SPIN_HOLD_POWER);
        }

        FtcLogger.exit();
    }

    private void spinStop() {
        FtcLogger.enter();
        if (relayEnabled) {
            spinServo.setPosition(SPIN_STOP_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Displays intake telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (relayEnabled && telemetryEnabled) {
            if (spinServo != null && rackNPinionServo != null && armMotor != null) {
                telemetry.addData(TAG, String.format(Locale.US, "spin: %.4f, rnp: %.4f, arm: %d",
                        spinServo.getPosition(), rackNPinionServo.getPosition(), armMotor.getCurrentPosition()));
            }
        }

        FtcLogger.exit();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (relayEnabled) {
            armForward();
            spinServo.getController().pwmEnable();
            rackNPinionServo.getController().pwmEnable();
            rnpPullIn();
            spinStop();
        }

        FtcLogger.exit();
    }

    /**
     * Stops the intake.
     */
    public void stop() {
        FtcLogger.enter();
        if (relayEnabled) {
            if (spinServo != null) {
                spinServo.getController().pwmDisable();
            }

            if (rackNPinionServo != null) {
                rackNPinionServo.getController().pwmDisable();
            }

            if (armMotor != null) {
                armMotor.setPower(FtcMotor.ZERO_POWER);
            }
        }

        FtcLogger.exit();
    }
}
