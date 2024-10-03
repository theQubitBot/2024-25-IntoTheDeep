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

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the robot lift.
 */
public class FtcLift extends FtcSubSystem {
    private static final String TAG = "FtcLift";
    public static final String RIGHT_LIFT_MOTOR_NAME = "leftLiftMotor";
    public static final String LEFT_LIFT_MOTOR_NAME = "rightLiftMotor";
    public static final int LIFT_POSITION_HIGH = 1340;
    public static final int LIFT_POSITION_LOW = 5;
    public static final int LIFT_POSITION_MEDIUM = 420;
    public static final int LIFT_POSITION_MINIMUM = 0;
    public static final int LIFT_POSITION_ERROR_MARGIN = 20;
    public static final int LEFT_LIFT_POSITION_INVALID = Integer.MIN_VALUE;
    public static final int RIGHT_LIFT_POSITION_INVALID = Integer.MIN_VALUE;
    public static final double LIFT_UP_POWER = 1.0;
    public static final double LIFT_DOWN_POWER = -LIFT_UP_POWER;
    public static final double LIFT_ZERO_POWER = 0.0;
    public static final double LIFT_TIME_MAX_MS = 3000;
    private final boolean liftEnabled = false;
    public boolean telemetryEnabled = true;
    private FtcBot parent;
    private Telemetry telemetry = null;
    public FtcMotor leftLiftMotor = null;
    public FtcMotor rightLiftMotor = null;

    /**
     * Estimate approximate time (in milliseconds) the lift will take
     * to travel from currentPosition to targetPosition.
     *
     * @param currentPosition Current position of the lift encoder.
     * @param targetPosition  Target position of the lift encoder.
     * @return Estimated lift travel time in milliseconds.
     */
    public double estimateTravelTime(int currentPosition, int targetPosition) {
        // Factor is (LIFT_TIME_HIGH_JUNCTION_MS - LIFT_TIME_LOW_JUNCTION_MS) /
        // (LIFT_TIME_HIGH_JUNCTION_MS - LIFT_TIME_LOW_JUNCTION_MS)
        double estimate;
        int distance = Math.abs(Math.abs(targetPosition) - Math.abs(currentPosition));
        estimate = 750 + distance * 5.0 / 12.0;
        return Range.clip(estimate, 0, LIFT_TIME_MAX_MS);
    }

    /**
     * Get the current lift position.
     *
     * @return The current lift position.
     */
    public int getPosition() {
        int leftPosition = LEFT_LIFT_POSITION_INVALID;
        int rightPosition = RIGHT_LIFT_POSITION_INVALID;
        FtcLogger.enter();
        if (liftEnabled) {
            leftPosition = leftLiftMotor.getCurrentPosition();
            rightPosition = rightLiftMotor.getCurrentPosition();
        }

        FtcLogger.exit();
        return leftPosition;
    }

    /**
     * Initialize standard Hardware interfaces
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, FtcBot parent) {
        FtcLogger.enter();
        this.parent = parent;
        this.telemetry = telemetry;
        if (liftEnabled) {
            rightLiftMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, RIGHT_LIFT_MOTOR_NAME));
            rightLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);
            rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            leftLiftMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, RIGHT_LIFT_MOTOR_NAME));
            leftLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);
            leftLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            // Initialize lift
            rightLiftMotor.setTargetPosition(LIFT_POSITION_LOW);
            rightLiftMotor.setPower(LIFT_ZERO_POWER);

            leftLiftMotor.setTargetPosition(LIFT_POSITION_LOW);
            leftLiftMotor.setPower(LIFT_ZERO_POWER);

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
    public Boolean liftNearTarget(int currentPosition, int targetPosition) {
        return FtcUtils.areEqual(currentPosition, targetPosition, LIFT_POSITION_ERROR_MARGIN);
    }

    /**
     * Operates lift based on gamePad inputs.
     *
     * @param gamePad1 The gamePad1 to control the lift operation.
     * @param gamePad2 The gamePad2 to control the lift operation.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2) {
        if (liftEnabled) {
            // Stop the lift if it hits the magnetic stops.
            double leftLiftPower = leftLiftMotor.getPower();
            double rightLiftPower = rightLiftMotor.getPower();
            int leftTargetPosition = LEFT_LIFT_POSITION_INVALID;
            int rightTargetPosition = RIGHT_LIFT_POSITION_INVALID;

            // If lift zero is being reset, we want lower the lift physically as well.
            if (gamePad1.a || gamePad2.a) {
                leftTargetPosition = FtcLift.LIFT_POSITION_LOW;
                rightTargetPosition = FtcLift.LIFT_POSITION_LOW;
            } else if (gamePad1.x || gamePad2.x || gamePad1.b || gamePad2.b) {
                leftTargetPosition = FtcLift.LIFT_POSITION_MEDIUM;
                rightTargetPosition = FtcLift.LIFT_POSITION_MEDIUM;
            } else if (gamePad1.y || gamePad2.y) {
                leftTargetPosition = FtcLift.LIFT_POSITION_HIGH;
                rightTargetPosition = FtcLift.LIFT_POSITION_HIGH;
            }

            if (leftTargetPosition != LEFT_LIFT_POSITION_INVALID ||
            rightTargetPosition != RIGHT_LIFT_POSITION_INVALID) {
                move(leftTargetPosition, rightTargetPosition, false);
            }

        }
    }

    /**
     * Moves lift to the target motor encoder position.
     *
     * @param leftTargetPosition The target motor encoder position.
     * @param waitTillEnd    When true, waits for lift to reach target position.
     */
    public void move(int leftTargetPosition, int rightTargetPosition, boolean waitTillEnd) {
        if (liftEnabled) {
            leftTargetPosition = Range.clip(leftTargetPosition,
                    LIFT_POSITION_MINIMUM, LIFT_POSITION_HIGH);
            rightTargetPosition = Range.clip(rightTargetPosition,
                    LIFT_POSITION_MINIMUM, LIFT_POSITION_HIGH);
            int leftCurrentPosition = leftLiftMotor.getCurrentPosition();
            int rightCurrentPosition = rightLiftMotor.getCurrentPosition();
            if (leftTargetPosition != leftCurrentPosition || rightTargetPosition != rightCurrentPosition) {
                // Must set motor position before setting motor mode.
                leftLiftMotor.setTargetPosition(leftTargetPosition);
                leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                rightLiftMotor.setTargetPosition(rightTargetPosition);
                rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }

            double liftPower = LIFT_ZERO_POWER;
            if (leftTargetPosition > leftCurrentPosition || rightTargetPosition > rightCurrentPosition) {
                liftPower = LIFT_UP_POWER;
            } else if (leftTargetPosition < leftCurrentPosition || rightTargetPosition < rightCurrentPosition) {
                liftPower = LIFT_DOWN_POWER;
            }

            leftLiftMotor.setPower(liftPower);
            rightLiftMotor.setPower(liftPower);
            if (waitTillEnd) {
                double waitTime = estimateTravelTime(leftCurrentPosition, leftTargetPosition);
                ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                while (!liftNearTarget(leftLiftMotor.getCurrentPosition(), leftTargetPosition) &&
                        !liftNearTarget(rightLiftMotor.getCurrentPosition(), rightTargetPosition) &&
                        runtime.milliseconds() < waitTime) {
                    FtcUtils.sleep(10);
                }
            }
        }
    }

    /**
     * Displays lift motor telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (liftEnabled && telemetryEnabled) {
            telemetry.addData(TAG, String.format(Locale.US, "Power %.2f, position %d",
                    leftLiftMotor.getPower(), leftLiftMotor.getCurrentPosition(),
                    rightLiftMotor.getPower(), rightLiftMotor.getCurrentPosition()));
        }

        FtcLogger.exit();
    }

    /**
     * Stops the lift by setting lift motor power to zero.
     */
    public void stop() {
        FtcLogger.enter();
        if (liftEnabled) {
            leftLiftMotor.setPower(LIFT_ZERO_POWER);
            rightLiftMotor.setPower(LIFT_ZERO_POWER);
        }

        FtcLogger.exit();
    }
}
