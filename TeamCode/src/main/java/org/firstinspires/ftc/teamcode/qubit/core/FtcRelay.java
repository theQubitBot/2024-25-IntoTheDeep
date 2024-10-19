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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * A class to manage the robot sample/specimen delivery.
 */
public class FtcRelay extends FtcSubSystem {
    private static final String TAG = "FtcRelay";
    public static final String ARM_MOTOR_NAME = "armMotor";
    public static final int ARM_FORWARD_POSITION = 0;
    public static final int ARM_BACKWARD_POSITION = 2800;
    public static final int ARM_LOW_RUNG_POSITION = 1400;
    public static final int ARM_HANG_POSITION = ARM_LOW_RUNG_POSITION;
    public static final double ARM_BACKWARD_POWER = FtcMotor.MIN_POWER;
    public static final double ARM_FORWARD_POWER = FtcMotor.MAX_POWER;
    public static final String RACK_N_PINION_SERVO_NAME = "rackAndPinionServo";
    public static final double RACK_N_PINION_EXTEND_POWER = 1.0;
    private static final double RACK_N_PINION_RETRACT_POWER = 0.0;
    public static final double RACK_N_PINION_STOP_POWER = FtcServo.MID_POSITION;
    public static final int RACK_N_PINION_TRAVEL_TIME = 2000; // milliseconds
    public static final String SPIN_SERVO_NAME = "spinServo";
    private static final double SPIN_IN_POWER = 0.75;
    private static final double SPIN_OUT_POWER = 0.25;
    private static final double SPIN_HOLD_POWER = 0.55;
    public static final double SPIN_STOP_POWER = FtcServo.MID_POSITION;
    private final boolean relayEnabled = true;
    public boolean telemetryEnabled = true;
    Deadline rnpTravelDeadline = null;
    public static int endAutoOpArmPosition = ARM_FORWARD_POSITION;
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

            rnpTravelDeadline = new Deadline(RACK_N_PINION_TRAVEL_TIME, TimeUnit.MILLISECONDS);
            rackNPinionServo = new FtcServo(hardwareMap.get(Servo.class, RACK_N_PINION_SERVO_NAME));
            rackNPinionServo.setDirection(Servo.Direction.REVERSE);

            spinServo = new FtcServo(hardwareMap.get(Servo.class, SPIN_SERVO_NAME));
            spinServo.setDirection(Servo.Direction.FORWARD);
            rnpTravelDeadline = new Deadline(2, TimeUnit.SECONDS);

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
     * @param runtime The tele op runtime.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
        FtcLogger.enter();
        if (FtcUtils.gameOver(runtime) || FtcUtils.hangInitiated(gamePad1, gamePad2, runtime)) {
            spinStop();
        } else if (gamePad1.right_trigger >= 0.5 || gamePad2.right_trigger >= 0.5) {
            spinIn();
        } else if (gamePad1.right_bumper || gamePad2.right_bumper) {
            spinOut();
        } else {
            spinHold();
        }

        if (FtcUtils.gameOver(runtime)) {
            rnpStop();
        } else if (gamePad1.dpad_up || gamePad2.dpad_up) {
            rnpExtend(false);
        } else if (gamePad1.dpad_down || gamePad2.dpad_down ||
                FtcUtils.hangInitiated(gamePad1, gamePad2, runtime)) {
            rnpRetract(false);
        } else {
            rnpStop();
        }

        if (FtcUtils.hangInitiated(gamePad1, gamePad2, runtime)) {
            moveArm(ARM_HANG_POSITION - endAutoOpArmPosition);
        } else if (gamePad1.left_trigger >= 0.5 || gamePad2.left_trigger >= 0.5) {
            moveArm(ARM_FORWARD_POSITION - endAutoOpArmPosition);
        } else if (gamePad1.left_bumper || gamePad2.left_bumper) {
            moveArm(ARM_BACKWARD_POSITION - endAutoOpArmPosition);
        }

        FtcLogger.exit();
    }

    /**
     * Extend the intake.
     * @param waitTillCompletion When ture, waits for intake to be fully extended.
     *                           Stops the extension servo.
     */
    public void rnpExtend(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (relayEnabled) {
            rackNPinionServo.setPosition(RACK_N_PINION_EXTEND_POWER);
            if (waitTillCompletion) {
                rnpTravelDeadline.reset();
                FtcUtils.sleep(rnpTravelDeadline);
                rackNPinionServo.setPosition(RACK_N_PINION_STOP_POWER);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Retract the intake.
     * @param waitTillCompletion When ture, waits for intake to be fully retracted.
     *                           Stops the extension servo.
     */
    public void rnpRetract(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (relayEnabled) {
            rackNPinionServo.setPosition(RACK_N_PINION_RETRACT_POWER);
            if (waitTillCompletion) {
                rnpTravelDeadline.reset();
                FtcUtils.sleep(rnpTravelDeadline);
                rackNPinionServo.setPosition(RACK_N_PINION_STOP_POWER);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Stops the intake.
     */
    public void rnpStop() {
        FtcLogger.enter();
        if (relayEnabled) {
            rackNPinionServo.setPosition(RACK_N_PINION_STOP_POWER);
        }

        FtcLogger.exit();
    }

    public int getArmPosition() {
        FtcLogger.enter();
        int armPosition = ARM_FORWARD_POSITION - endAutoOpArmPosition;
        if (relayEnabled) {
            armPosition = armMotor.getCurrentPosition();
        }

        FtcLogger.exit();
        return armPosition;
    }

    public void moveArm(int targetPosition) {
        FtcLogger.enter();
        if (relayEnabled) {
            int currentPosition = armMotor.getCurrentPosition();
            double power = currentPosition < targetPosition ? ARM_BACKWARD_POWER : ARM_FORWARD_POWER;
            armMotor.setTargetPosition(targetPosition);
            armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            armMotor.setPower(power);
        }

        FtcLogger.exit();
    }

    /**
     * Spin inwards to intake the object.
     */
    public void spinIn() {
        FtcLogger.enter();
        if (relayEnabled) {
            spinServo.setPosition(SPIN_IN_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Spin outwards to outtake the object.
     */
    public void spinOut() {
        FtcLogger.enter();
        if (relayEnabled) {
            spinServo.setPosition(SPIN_OUT_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Spin slowly inwards to hold the object.
     */
    public void spinHold() {
        FtcLogger.enter();
        if (relayEnabled) {
            spinServo.setPosition(SPIN_HOLD_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Stops all motion.
     */
    public void spinStop() {
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

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (relayEnabled) {
            moveArm(FtcRelay.ARM_FORWARD_POSITION - endAutoOpArmPosition);
            spinServo.getController().pwmEnable();
            rackNPinionServo.getController().pwmEnable();
            rnpRetract(false);
            spinStop();
        }

        FtcLogger.exit();
    }

    /**
     * Stops the Relay.
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
