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
    public static final String LEFT_LIFT_SERVO_NAME = "leftLiftServo";
    public static final String RIGHT_LIFT_SERVO_NAME = "rightLiftServo";
    public static final double LEFT_LIFT_SERVO_FORWARD_POSITION = 0.5;
    public static final double LEFT_LIFT_SERVO_BACKWARD_POSITION = 0.5;
    public static final double RIGHT_LIFT_SERVO_FORWARD_POSITION = 0.5;
    public static final double RIGHT_LIFT_SERVO_BACKWARD_POSITION = 0.5;
    public static final String RACK_N_PINION_SERVO_NAME = "rackAndPinionServo";
    public static final double RACK_N_PINION_SERVO_FORWARD_POSITION = 0.5;
    public static final double RACK_N_PINION_SERVO_REAR_POSITION = 0.5;
    public static final String SPINNER_SERVO_NAME = "spinnerServo";
    public static final double SPINNER_IN_POWER = 1.0;
    public static final double SPINNER_HOLD_POWER = 0.60;
    public static final double SPINNER_STOP_POWER = 0.50;
    public static final double SPINNER_OUT_POWER = 0.00;
    private final boolean relayEnabled = false;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    public FtcServo leftLiftServo = null;
    public FtcServo rightLiftServo = null;
    public FtcServo rackNPinionServo = null;
    public FtcServo spinnerServo = null;

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
            leftLiftServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_LIFT_SERVO_NAME));
            spinnerServo.setDirection(Servo.Direction.FORWARD);

            rightLiftServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_LIFT_SERVO_NAME));
            spinnerServo.setDirection(Servo.Direction.FORWARD);

            rackNPinionServo = new FtcServo(hardwareMap.get(Servo.class, RACK_N_PINION_SERVO_NAME));
            rackNPinionServo.setDirection(Servo.Direction.FORWARD);

            spinnerServo = new FtcServo(hardwareMap.get(Servo.class, SPINNER_SERVO_NAME));
            spinnerServo.setDirection(Servo.Direction.FORWARD);
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
            pushOut();
        } else if (gamePad1.dpad_down || gamePad2.dpad_down) {
            pullIn();
        }

        if (gamePad1.left_trigger >= 0.5 || gamePad2.left_trigger >= 0.5) {
            rotateForward();
        } else if (gamePad1.right_bumper || gamePad2.right_bumper) {
            rotateBackward();
        }

        FtcLogger.exit();
    }

    private void pullIn() {
        FtcLogger.enter();
        if (relayEnabled) {
            rackNPinionServo.setPosition(RACK_N_PINION_SERVO_REAR_POSITION);
        }

        FtcLogger.exit();
    }

    private void pushOut() {
        FtcLogger.enter();
        if (relayEnabled) {
            rackNPinionServo.setPosition(RACK_N_PINION_SERVO_FORWARD_POSITION);
        }

        FtcLogger.exit();
    }

    private void rotateForward() {
        FtcLogger.enter();
        if (relayEnabled) {
            leftLiftServo.setPosition(LEFT_LIFT_SERVO_FORWARD_POSITION);
            rightLiftServo.setPosition(RIGHT_LIFT_SERVO_FORWARD_POSITION);
        }

        FtcLogger.exit();
    }

    private void rotateBackward() {
        FtcLogger.enter();
        if (relayEnabled) {
            leftLiftServo.setPosition(LEFT_LIFT_SERVO_BACKWARD_POSITION);
            rightLiftServo.setPosition(RIGHT_LIFT_SERVO_BACKWARD_POSITION);
        }

        FtcLogger.exit();
    }

    /**
     * Rotate inwards to intake the object.
     */
    private void spinIn() {
        FtcLogger.enter();
        if (relayEnabled) {
            spinnerServo.setPosition(SPINNER_IN_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Rotate outwards to outtake the object.
     */
    private void spinOut() {
        FtcLogger.enter();
        if (relayEnabled) {
            spinnerServo.setPosition(SPINNER_OUT_POWER);
        }

        FtcLogger.exit();
    }

    private void spinHold() {
        FtcLogger.enter();
        if (relayEnabled) {
            spinnerServo.setPosition(SPINNER_HOLD_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Displays intake telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (relayEnabled && telemetryEnabled && spinnerServo != null) {
            telemetry.addData(TAG, String.format(Locale.US, "Position: %.4f",
                    spinnerServo.getPosition()));
        }

        FtcLogger.exit();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (relayEnabled) {
            leftLiftServo.getController().pwmEnable();
            rightLiftServo.getController().pwmEnable();
            leftLiftServo.setPosition(LEFT_LIFT_SERVO_FORWARD_POSITION);
            rightLiftServo.setPosition(RIGHT_LIFT_SERVO_FORWARD_POSITION);

            rackNPinionServo.getController().pwmEnable();
            rackNPinionServo.setPosition(RACK_N_PINION_SERVO_REAR_POSITION);

            spinnerServo.getController().pwmEnable();
            spinnerServo.setPosition(SPINNER_STOP_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Stops the intake.
     */
    public void stop() {
        FtcLogger.enter();
        if (relayEnabled) {
            leftLiftServo.setPosition(LEFT_LIFT_SERVO_FORWARD_POSITION);
            rightLiftServo.setPosition(RIGHT_LIFT_SERVO_FORWARD_POSITION);
            leftLiftServo.getController().pwmDisable();
            rightLiftServo.getController().pwmDisable();

            rackNPinionServo.setPosition(RACK_N_PINION_SERVO_REAR_POSITION);
            rackNPinionServo.getController().pwmDisable();

            spinnerServo.setPosition(SPINNER_STOP_POWER);
            spinnerServo.getController().pwmDisable();
        }

        FtcLogger.exit();
    }
}
