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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * A class to manage the robot sample/specimen delivery.
 */
public class FtcIntake extends FtcSubSystem {
    private static final String TAG = "FtcIntake";
    public static final String LEFT_SPIN_SERVO_NAME = "leftSpinServo";
    public static final String RIGHT_SPIN_SERVO_NAME = "rightSpinServo";
    public static final double SPIN_IN_POWER = 1.0000;
    public static final double SPIN_OUT_POWER = 0.0000;
    public static final double SPIN_HOLD_POWER = 0.6000;
    public static final double SPIN_STOP_POWER = FtcServo.MID_POSITION;
    public static final String LEFT_FLIP_SERVO_NAME = "leftFlipServo";
    public static final String RIGHT_FLIP_SERVO_NAME = "rightFlipServo";
    public static final double FLIP_DOWN_LEFT_POSITION = 0.4000;
    public static final double FLIP_DOWN_RIGHT_POSITION = FLIP_DOWN_LEFT_POSITION - 0.0085;

    public static final double FLIP_HORIZONTAL_LEFT_POSITION = 0.4300;
    public static final double FLIP_HORIZONTAL_RIGHT_POSITION = FLIP_HORIZONTAL_LEFT_POSITION - 0.085;
    public static final double FLIP_DELIVER_LEFT_POSITION = 0.4800;
    public static final double FLIP_DELIVER_RIGHT_POSITION = FLIP_DELIVER_LEFT_POSITION - 0.085;
    public static final int FLIP_TRAVEL_TIME = 2000; // milliseconds
    private final boolean intakeEnabled = true;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    private final FtcBot parent;
    public FtcServo leftSpinServo = null;
    public FtcServo rightSpinServo = null;
    public FtcServo leftFlipServo = null;
    public FtcServo rightFlipServo = null;
    private Deadline travelDeadline = null;

    public FtcIntake(FtcBot robot) {
        parent = robot;
    }

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (intakeEnabled) {
            leftSpinServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_SPIN_SERVO_NAME));
            leftSpinServo.setDirection(Servo.Direction.FORWARD);
            rightSpinServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_SPIN_SERVO_NAME));
            rightSpinServo.setDirection(Servo.Direction.FORWARD);

            leftFlipServo = new FtcServo(hardwareMap.get(Servo.class, LEFT_FLIP_SERVO_NAME));
            leftFlipServo.setDirection(Servo.Direction.FORWARD);
            rightFlipServo = new FtcServo(hardwareMap.get(Servo.class, RIGHT_FLIP_SERVO_NAME));
            rightFlipServo.setDirection(Servo.Direction.REVERSE);

            travelDeadline = new Deadline(FLIP_TRAVEL_TIME, TimeUnit.MILLISECONDS);
            travelDeadline.expire();

            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Operates the intake using the gamePads.
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     * @param runtime  The tele op runtime.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
        FtcLogger.enter();

        if (!travelDeadline.hasExpired()) {
            // Once travel deadline has been set, all operations are suspended.
            // This ensures that flip completes its operation. E.g.: Hang initiated.
            return;
        }

        if (FtcUtils.gameOver(runtime)) {
            spinStop();
        } else if (FtcUtils.hangInitiated(gamePad1, gamePad2, runtime)) {
            spinStop();
            flipDelivery(false);
            travelDeadline.reset();
        } else if (gamePad1.right_trigger >= 0.5 || gamePad2.right_trigger >= 0.5) {
            spinIn();
            flipDown(false);
        } else if (gamepad1.dpad_down || gamepad1.a || gamepad2.a) {
            // you cannot flip to delivery unless rnp is retracted, bucket is down, and lift is down
            // Use magnetic sensors
            spinIn();
            flipDelivery(true);
        } else if (gamePad2.right_stick_y <= -0.5) {
            flipDown(false);
            spinOut();
        } else {
            spinHold();
            flipHorizontal(false);
        }

        FtcLogger.exit();
    }

    public void flipDelivery(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (intakeEnabled) {
            leftFlipServo.setPosition(FLIP_DELIVER_LEFT_POSITION);
            rightFlipServo.setPosition(FLIP_DELIVER_RIGHT_POSITION);
            if (waitTillCompletion) {
                FtcUtils.sleep(FLIP_TRAVEL_TIME);
            }
        }

        FtcLogger.exit();
    }

    public void flipHorizontal(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (intakeEnabled) {
            leftFlipServo.setPosition(FLIP_HORIZONTAL_LEFT_POSITION);
            rightFlipServo.setPosition(FLIP_HORIZONTAL_RIGHT_POSITION);
            if (waitTillCompletion) {
                FtcUtils.sleep(FLIP_TRAVEL_TIME);
            }
        }

        FtcLogger.exit();
    }

    public void flipDown(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (intakeEnabled) {
            leftFlipServo.setPosition(FLIP_DOWN_LEFT_POSITION);
            rightFlipServo.setPosition(FLIP_DOWN_RIGHT_POSITION);
            if (waitTillCompletion) {
                FtcUtils.sleep(FLIP_TRAVEL_TIME);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Spin slowly inwards to hold the sample.
     */
    public void spinHold() {
        FtcLogger.enter();
        if (intakeEnabled) {
            leftSpinServo.setPosition(SPIN_HOLD_POWER);
            rightSpinServo.setPosition(SPIN_HOLD_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Spin inwards to intake the sample.
     */
    public void spinIn() {
        FtcLogger.enter();
        if (intakeEnabled) {
            leftSpinServo.setPosition(SPIN_IN_POWER);
            rightSpinServo.setPosition(SPIN_IN_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Spin outwards to outtake the sample.
     */
    public void spinOut() {
        FtcLogger.enter();
        if (intakeEnabled) {
            leftSpinServo.setPosition(SPIN_OUT_POWER);
            rightSpinServo.setPosition(SPIN_OUT_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Stops all motion.
     */
    public void spinStop() {
        FtcLogger.enter();
        if (intakeEnabled) {
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
        if (intakeEnabled && telemetryEnabled) {
            if (leftSpinServo != null && rightSpinServo != null && leftFlipServo != null && rightFlipServo != null) {
                telemetry.addData(TAG, String.format(Locale.US,
                        "spin: %5.4f, %5.4f, flip: %5.4f, %5.4f",
                        leftSpinServo.getPosition(), rightSpinServo.getPosition(),
                        leftFlipServo.getPosition(), rightFlipServo.getPosition()));
            }
        }

        FtcLogger.exit();
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (intakeEnabled) {
            leftSpinServo.getController().pwmEnable();
            rightSpinServo.getController().pwmEnable();
            leftFlipServo.getController().pwmEnable();
            rightFlipServo.getController().pwmEnable();

            spinStop();
            flipHorizontal(false);
        }

        FtcLogger.exit();
    }

    /**
     * Stops the Relay.
     */
    public void stop() {
        FtcLogger.enter();
        if (intakeEnabled) {
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
        }

        FtcLogger.exit();
    }
}
