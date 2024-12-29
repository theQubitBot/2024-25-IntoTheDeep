/* Copyright (c) 2024 The Qubit Bot. All rights reserved.
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

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * A class to manage the robot lift.
 */
public class FtcLift extends FtcSubSystem {
    private static final String TAG = "FtcLift";
    public static final String LEFT_MOTOR_NAME = "leftLiftMotor";
    public static final String RIGHT_MOTOR_NAME = "rightLiftMotor";
    public static final int TARGET_POSITION_TOLERANCE = 5;
    public static final int POSITION_HIGH_BASKET = 2025;
    public static final int POSITION_LOW_BASKET = 650;
    public static final int POSITION_HANG = 1000;
    public static final int POSITION_HIGH_RUNG = POSITION_HIGH_BASKET;
    public static final int POSITION_LOW_RUNG = POSITION_HANG;
    public static final int POSITION_HIGH_CHAMBER = 1550;
    public static final int POSITION_HIGH_CHAMBER_DELIVERY = 850;
    public static final int POSITION_LOW_CHAMBER = 450;
    public static final int POSITION_SPECIMEN_INTAKE = 50;
    public static final int POSITION_FLOOR = 5;
    public static final int POSITION_MINIMUM = 0;
    public static final int POSITION_INVALID = Integer.MIN_VALUE;

    public static final double UP_POWER = FtcMotor.MAX_POWER;
    public static final double DOWN_POWER = -UP_POWER;
    public static final int TRAVEL_TIME_MAX_MS = 2000;
    private final boolean liftEnabled = true;
    public boolean telemetryEnabled = true;
    public static int endAutoOpLeftLiftPosition = POSITION_MINIMUM;
    public static int endAutoOpRightLiftPosition = POSITION_MINIMUM;
    private Telemetry telemetry = null;
    private final FtcBot parent;
    private FtcMotor leftLiftMotor = null;
    private FtcMotor rightLiftMotor = null;
    public static final String LEFT_TOUCH_SENSOR_NAME = "leftLiftTouch";
    public static final String RIGHT_TOUCH_SENSOR_NAME = "rightLiftTouch";
    private TouchSensor leftLiftTouch = null;
    private TouchSensor rightLiftTouch = null;
    private final boolean enableLiftResetOnTouch = true;
    private boolean hangSequenceInitiated = false;

    public FtcLift(FtcBot robot) {
        parent = robot;
    }

    public boolean atPosition(int position) {
        FtcLogger.enter();
        boolean atPosition = false;
        if (liftEnabled) {
            atPosition = FtcUtils.areEqual(getLeftPosition(), position, TARGET_POSITION_TOLERANCE);
        }

        FtcLogger.exit();
        return atPosition;
    }

    private boolean botIsHanging() {
        return parent != null && parent.imu != null && parent.imu.getPitch() <= 10.0;
    }

    /**
     * Estimate approximate time (in milliseconds) the lift will take
     * to travel from currentPosition to targetPosition.
     *
     * @param currentPosition Current position of the lift encoder.
     * @param targetPosition  Target position of the lift encoder.
     * @return Estimated lift travel time in milliseconds.
     */
    private long estimateTravelTime(int currentPosition, int targetPosition) {
        int estimate;
        int distance = Math.abs(Math.abs(targetPosition) - Math.abs(currentPosition));

        estimate = distance / 2;
        return Range.clip(estimate, 0, TRAVEL_TIME_MAX_MS);
    }

    /**
     * Get the current left lift position.
     *
     * @return The current left lift position.
     */
    public int getLeftPosition() {
        int position = POSITION_INVALID;
        FtcLogger.enter();
        if (liftEnabled) {
            position = leftLiftMotor.getCurrentPosition();
        }

        FtcLogger.exit();
        return position;
    }

    /**
     * Get the current right lift position.
     *
     * @return The right left lift position.
     */
    public int getRightPosition() {
        int position = POSITION_INVALID;
        FtcLogger.enter();
        if (liftEnabled) {
            position = rightLiftMotor.getCurrentPosition();
        }

        FtcLogger.exit();
        return position;
    }

    /**
     * Initialize standard Hardware interfaces
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (liftEnabled) {
            if (enableLiftResetOnTouch) {
                leftLiftTouch = hardwareMap.get(TouchSensor.class, LEFT_TOUCH_SENSOR_NAME);
                rightLiftTouch = hardwareMap.get(TouchSensor.class, RIGHT_TOUCH_SENSOR_NAME);
            }

            rightLiftMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, RIGHT_MOTOR_NAME));
            rightLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);
            rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightLiftMotor.setTargetPositionTolerance(TARGET_POSITION_TOLERANCE);

            leftLiftMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, LEFT_MOTOR_NAME));
            leftLiftMotor.setDirection(DcMotorEx.Direction.FORWARD);
            leftLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftLiftMotor.setTargetPositionTolerance(TARGET_POSITION_TOLERANCE);

            // Initialize lift
            rightLiftMotor.setTargetPosition(POSITION_FLOOR);
            rightLiftMotor.setPower(FtcMotor.ZERO_POWER);

            leftLiftMotor.setTargetPosition(POSITION_FLOOR);
            leftLiftMotor.setPower(FtcMotor.ZERO_POWER);

            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Determines if the lift is near its target position.
     *
     * @param currentPosition Lift's current position.
     * @param targetPosition  Lift's target position.
     * @return True if lift is close to its target position, false otherwise.
     */
    private Boolean liftNearTarget(int currentPosition, int targetPosition) {
        return FtcUtils.areEqual(currentPosition, targetPosition, TARGET_POSITION_TOLERANCE);
    }

    /**
     * Operates lift based on gamePad inputs.
     *
     * @param gamePad1 The gamePad1 to control the lift operation.
     * @param gamePad2 The gamePad2 to control the lift operation.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
        if (liftEnabled) {
            if (FtcUtils.gameOver(runtime)) {
                if (botIsHanging()) {
                    lowerBotSlowly();
                } else {
                    stop();
                }

                if (!FtcUtils.DEBUG) {
                    return;
                }
            }

            resetLiftIfTouchPressed();

            int leftCurrentPosition = leftLiftMotor.getCurrentPosition();
            int rightCurrentPosition = rightLiftMotor.getCurrentPosition();

            int leftTargetPosition = leftCurrentPosition;
            int rightTargetPosition = rightCurrentPosition;

            if (gamePad1.a || gamePad2.a) {
                leftTargetPosition = FtcLift.POSITION_FLOOR - endAutoOpLeftLiftPosition;
                rightTargetPosition = FtcLift.POSITION_FLOOR - endAutoOpRightLiftPosition;
            } else if (gamePad1.b || gamePad2.b) {
                leftTargetPosition = FtcLift.POSITION_HIGH_CHAMBER - endAutoOpLeftLiftPosition;
                rightTargetPosition = FtcLift.POSITION_HIGH_CHAMBER - endAutoOpRightLiftPosition;
            } else if (gamePad1.x || gamePad2.x) {
                leftTargetPosition = FtcLift.POSITION_SPECIMEN_INTAKE - endAutoOpLeftLiftPosition;
                rightTargetPosition = FtcLift.POSITION_SPECIMEN_INTAKE - endAutoOpRightLiftPosition;
            } else if (gamePad1.y || gamePad2.y) {
                leftTargetPosition = FtcLift.POSITION_HIGH_BASKET - endAutoOpLeftLiftPosition;

                // Right lift may be at chamber level, lower it.
                rightTargetPosition = FtcLift.POSITION_FLOOR - endAutoOpRightLiftPosition;
            } else if (FtcUtils.hangInitiated(gamePad1, gamePad2, runtime)) {
                hangSequenceInitiated = true;
                leftTargetPosition = FtcLift.POSITION_HANG - endAutoOpLeftLiftPosition;
                rightTargetPosition = FtcLift.POSITION_HANG - endAutoOpRightLiftPosition;
            }

            if (!liftNearTarget(leftCurrentPosition, leftTargetPosition) ||
                    !liftNearTarget(rightCurrentPosition, rightTargetPosition)) {
                move(leftTargetPosition, rightTargetPosition, false);
            } else {
                // If lift is at LOW position, set motor power to be zero
                if (liftNearTarget(leftCurrentPosition, POSITION_MINIMUM - endAutoOpLeftLiftPosition)) {
                    leftLiftMotor.setPower(FtcMotor.ZERO_POWER);
                }

                if (liftNearTarget(rightCurrentPosition, POSITION_MINIMUM - endAutoOpRightLiftPosition)) {
                    rightLiftMotor.setPower(FtcMotor.ZERO_POWER);
                }
            }
        }
    }

    public void lowerBotSlowly() {
        if (liftEnabled) {
            int leftCurrentPosition = leftLiftMotor.getCurrentPosition();
            int leftTargetPosition = leftCurrentPosition;

            if (leftCurrentPosition < POSITION_LOW_RUNG) {
                leftTargetPosition = leftTargetPosition + TARGET_POSITION_TOLERANCE;
            }

            int rightcurrentPosition = rightLiftMotor.getCurrentPosition();
            int rightTargetPosition = rightcurrentPosition;
            if (rightcurrentPosition < POSITION_LOW_RUNG) {
                rightTargetPosition = rightTargetPosition + TARGET_POSITION_TOLERANCE;
            }

            move(leftTargetPosition, rightTargetPosition, false);

            // 5 ticks in 25 ms
            // 1000 ticks in 25 * 1000 / 5 =5000ms = 5 seconds
            FtcUtils.sleep(25);
        }
    }

    /**
     * Moves lift to the target motor encoder position.
     *
     * @param leftTargetPosition The target motor encoder position.
     * @param waitTillCompletion When true, waits for lift to reach target position.
     */
    public void move(int leftTargetPosition, int rightTargetPosition, boolean waitTillCompletion) {
        if (liftEnabled) {
            double liftPower;
            leftTargetPosition = Range.clip(leftTargetPosition,
                    POSITION_MINIMUM - endAutoOpLeftLiftPosition, POSITION_HIGH_BASKET - endAutoOpLeftLiftPosition);
            int leftCurrentPosition = leftLiftMotor.getCurrentPosition();
            if (!liftNearTarget(leftCurrentPosition, leftTargetPosition)) {
                // Must set motor position before setting motor mode.
                leftLiftMotor.setTargetPosition(leftTargetPosition);
                leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                liftPower = leftTargetPosition > leftCurrentPosition ? UP_POWER : DOWN_POWER;
                leftLiftMotor.setPower(liftPower);
            }

            rightTargetPosition = Range.clip(rightTargetPosition,
                    POSITION_MINIMUM - endAutoOpRightLiftPosition, POSITION_HIGH_BASKET - endAutoOpRightLiftPosition);
            int rightCurrentPosition = rightLiftMotor.getCurrentPosition();
            if (!liftNearTarget(rightCurrentPosition, rightTargetPosition)) {
                // Must set motor position before setting motor mode.
                rightLiftMotor.setTargetPosition(rightTargetPosition);
                rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                liftPower = rightTargetPosition > rightCurrentPosition ? UP_POWER : DOWN_POWER;
                rightLiftMotor.setPower(liftPower);
            }

            if (waitTillCompletion) {
                long timeout = estimateTravelTime(leftCurrentPosition, leftTargetPosition);
                Deadline d = new Deadline(timeout, TimeUnit.MILLISECONDS);
                d.reset();
                while (!d.hasExpired() && !liftNearTarget(leftCurrentPosition, leftTargetPosition)) {
                    FtcUtils.sleep(FtcUtils.CYCLE_MS);
                    leftCurrentPosition = leftLiftMotor.getCurrentPosition();
                }
            }
        }
    }

    public void resetLiftIfTouchPressed() {
        FtcLogger.enter();

        if (enableLiftResetOnTouch) {
            // Must get current position before evaluating lift encoder reset.
            int leftCurrentPosition = leftLiftMotor.getCurrentPosition();
            int rightCurrentPosition = rightLiftMotor.getCurrentPosition();

            // Reset lift encoder if lift is not at low position (belt is slipping)
            if (leftLiftTouch.isPressed() &&
                    // If hang operation has been initiated, then don't set zero lift power
                    !hangSequenceInitiated &&
                    (!liftNearTarget(leftCurrentPosition, POSITION_FLOOR) &&
                            !liftNearTarget(leftCurrentPosition, POSITION_MINIMUM))) {
                leftLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                leftLiftMotor.setTargetPosition(POSITION_FLOOR);
                leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(FtcMotor.ZERO_POWER);
                endAutoOpLeftLiftPosition = 0;
                endAutoOpRightLiftPosition = 0;
            }

            if (rightLiftTouch.isPressed() &&
                    // If hang operation has been initiated, then don't set zero lift power
                    !hangSequenceInitiated &&
                    (!liftNearTarget(rightCurrentPosition, POSITION_FLOOR) &&
                            !liftNearTarget(rightCurrentPosition, POSITION_MINIMUM))) {
                rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                rightLiftMotor.setTargetPosition(POSITION_FLOOR);
                rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setPower(FtcMotor.ZERO_POWER);
                endAutoOpLeftLiftPosition = 0;
                endAutoOpRightLiftPosition = 0;
            }
        }

        FtcLogger.exit();
    }

    /**
     * Displays lift motor telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (liftEnabled && telemetryEnabled) {
            String message = String.format(Locale.US, "LLPower %.2f, LLPosition %d, RLPower %.2f, RLPosition %d",
                    leftLiftMotor.getPower(), leftLiftMotor.getCurrentPosition(),
                    rightLiftMotor.getPower(), rightLiftMotor.getCurrentPosition());
            if (enableLiftResetOnTouch) {
                message += String.format(Locale.US, "LeftTouch: %b, RightTouch: %b",
                        leftLiftTouch.isPressed(), rightLiftTouch.isPressed());
            }

            telemetry.addData(TAG, message);
        }

        FtcLogger.exit();
    }

    /**
     * Stops the lift by setting lift motor power to zero.
     */
    public void stop() {
        FtcLogger.enter();
        if (liftEnabled) {
            if (leftLiftMotor != null) {
                leftLiftMotor.setPower(FtcMotor.ZERO_POWER);
            }

            if (rightLiftMotor != null) {
                rightLiftMotor.setPower(FtcMotor.ZERO_POWER);
            }
        }

        FtcLogger.exit();
    }
}
