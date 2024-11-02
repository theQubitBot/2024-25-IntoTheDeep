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
    public static final int ARM_MINIMUM_POSITION = 0;
    public static final int ARM_FORWARD_POSITION = 105;
    public static final int ARM_BACKWARD_POSITION = 2570;
    public static final int ARM_POSITION_CHAMBER_LOW = 813;
    public static final int ARM_POSITION_CHAMBER_HIGH = 1260;
    public static final int ARM_LOW_RUNG_POSITION = 1200;
    public static final int ARM_HANG_POSITION = ARM_LOW_RUNG_POSITION;
    public static final double ARM_FORWARD_POWER = 0.80;
    public static final double ARM_BACKWARD_POWER = -ARM_FORWARD_POWER;
    public static final String RACK_N_PINION_SERVO_NAME = "rackAndPinionServo";
    public static final String LEFT_FLIP_SERVO_NAME = "leftFlipServo";
    public static final String RIGHT_FLIP_SERVO_NAME = "rightFlipServo";
    public static final double RACK_N_PINION_EXTEND_POWER = 1.0;
    public static final double RACK_N_PINION_RETRACT_POWER = 0.0;
    public static final double RACK_N_PINION_STOP_POWER = FtcServo.MID_POSITION;
    public static final int RACK_N_PINION_TRAVEL_TIME = 2000; // milliseconds
    public static final String LEFT_SPIN_SERVO_NAME = "leftSpinServo";
    public static final String RIGHT_SPIN_SERVO_NAME = "rightSpinServo";
    public static final double SPIN_IN_POWER = 0.5700;
    public static final double SPIN_OUT_POWER = 0.0000;
    public static final double SPIN_HOLD_POWER = 0.5280;
    public static final double SPIN_STOP_POWER = FtcServo.MID_POSITION;
    public static final String LIFT_SERVO_NAME = "liftServo";
    private final boolean relayEnabled = true;
    public boolean telemetryEnabled = true;
    Deadline rnpTravelDeadline = null;
    public static int endAutoOpArmPosition = ARM_MINIMUM_POSITION;
    private Telemetry telemetry = null;
    public FtcMotor armMotor = null;
    public FtcServo rackNPinionServo = null;
    public FtcServo leftSpinServo = null;
    public FtcServo rightSpinServo = null;
    public FtcServo leftFlipServo = null;
    public FtcServo rightFlipServo = null;
    public FtcServo liftServo = null;

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

            leftSpinServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_SPIN_SERVO_NAME));
            leftSpinServo.setDirection(Servo.Direction.FORWARD);
            rightSpinServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_SPIN_SERVO_NAME));
            rightSpinServo.setDirection(Servo.Direction.FORWARD);

            leftFlipServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_FLIP_SERVO_NAME));
            leftFlipServo.setDirection(Servo.Direction.FORWARD);
            rightFlipServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_FLIP_SERVO_NAME));
            rightFlipServo.setDirection(Servo.Direction.FORWARD);

            liftServo = new FtcServo(hardwareMap.get(Servo.class, LIFT_SERVO_NAME));
            liftServo.setDirection(Servo.Direction.FORWARD);

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
     * @param runtime  The tele op runtime.
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
            rnpExtend();
        } else if (gamePad1.dpad_down || gamePad2.dpad_down ||
                FtcUtils.hangInitiated(gamePad1, gamePad2, runtime)) {
            rnpRetract();
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
     */
    public void rnpExtend() {
        FtcLogger.enter();
        if (relayEnabled) {
            rackNPinionServo.setPosition(RACK_N_PINION_EXTEND_POWER);
        }

        FtcLogger.exit();
    }

    public void rnpWaitAndStop() {
        FtcUtils.sleep(RACK_N_PINION_TRAVEL_TIME);
        rackNPinionServo.setPosition(RACK_N_PINION_STOP_POWER);
    }

    /**
     * Retract the intake.
     */
    public void rnpRetract() {
        FtcLogger.enter();
        if (relayEnabled) {
            rackNPinionServo.setPosition(RACK_N_PINION_RETRACT_POWER);
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
            leftSpinServo.setPosition(SPIN_IN_POWER);
            rightSpinServo.setPosition(SPIN_IN_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Spin outwards to outtake the object.
     */
    public void spinOut() {
        FtcLogger.enter();
        if (relayEnabled) {
            leftSpinServo.setPosition(SPIN_OUT_POWER);
            rightSpinServo.setPosition(SPIN_OUT_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Spin slowly inwards to hold the object.
     */
    public void spinHold() {
        FtcLogger.enter();
        if (relayEnabled) {
            leftSpinServo.setPosition(SPIN_HOLD_POWER);
            rightSpinServo.setPosition(SPIN_HOLD_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Stops all motion.
     */
    public void spinStop() {
        FtcLogger.enter();
        if (relayEnabled) {
            leftSpinServo.setPosition(SPIN_STOP_POWER);
            rightSpinServo.setPosition(SPIN_STOP_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Displays intake telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (relayEnabled && telemetryEnabled) {
            if (leftSpinServo != null && rightSpinServo != null && leftFlipServo != null && rightFlipServo != null
                    && liftServo != null && rackNPinionServo != null && armMotor != null) {
                telemetry.addData(TAG, String.format(Locale.US, "spin: %.4f, rnp: %.4f, arm: %d",
                        leftSpinServo.getPosition(), rightSpinServo.getPosition(), leftFlipServo.getPosition(),
                        rightFlipServo.getPosition(), liftServo.getPosition(), rackNPinionServo.getPosition(),
                        armMotor.getCurrentPosition()));
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
            leftSpinServo.getController().pwmEnable();
            rightSpinServo.getController().pwmEnable();
            leftFlipServo.getController().pwmEnable();
            rightFlipServo.getController().pwmEnable();
            liftServo.getController().pwmEnable();
            rackNPinionServo.getController().pwmEnable();
            rnpStop();
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
            if (leftSpinServo != null) {
                leftSpinServo.getController().pwmDisable();
            }
            if (rightSpinServo != null) {
                rightSpinServo.getController().pwmDisable();
            }
            if (leftFlipServo != null) {
                leftFlipServo.getController().pwmDisable();
            }
            if (rightFlipServo != null) {
                rightFlipServo.getController().pwmDisable();
            }
            if (liftServo != null) {
                liftServo.getController().pwmDisable();
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
