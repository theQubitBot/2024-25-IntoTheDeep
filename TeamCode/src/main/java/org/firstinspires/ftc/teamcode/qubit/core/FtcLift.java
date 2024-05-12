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
    public static final String LIFT_MOTOR_NAME = "liftMotor";
    public static final int LIFT_POSITION_HIGH = 1340;
    public static final int LIFT_POSITION_LOW = 5;
    public static final int LIFT_POSITION_MEDIUM = 420;
    public static final int LIFT_POSITION_MINIMUM = 0;
    public static final int LIFT_POSITION_ERROR_MARGIN = 20;
    public static final int LIFT_POSITION_INVALID = Integer.MIN_VALUE;
    public static final double LIFT_UP_POWER = 1.0;
    public static final double LIFT_DOWN_POWER = -LIFT_UP_POWER;
    public static final double LIFT_ZERO_POWER = 0.0;
    public static final double LIFT_TIME_MAX_MS = 3000;
    private final boolean liftEnabled = false;
    public boolean telemetryEnabled = true;
    private FtcBot parent;
    private Telemetry telemetry = null;
    public FtcMotor liftMotor = null;

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
        int position = LIFT_POSITION_INVALID;
        FtcLogger.enter();
        if (liftEnabled) {
            position = liftMotor.getCurrentPosition();
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
    public void init(HardwareMap hardwareMap, Telemetry telemetry, FtcBot parent) {
        FtcLogger.enter();
        this.parent = parent;
        this.telemetry = telemetry;
        if (liftEnabled) {
            liftMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, LIFT_MOTOR_NAME));
            liftMotor.setDirection(DcMotorEx.Direction.REVERSE);
            liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            // Initialize lift
            liftMotor.setTargetPosition(LIFT_POSITION_LOW);
            liftMotor.setPower(LIFT_ZERO_POWER);

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
            double liftPower = liftMotor.getPower();
            int targetPosition = LIFT_POSITION_INVALID;

            // If lift zero is being reset, we want lower the lift physically as well.
            if (gamePad1.a || gamePad2.a) {
                targetPosition = FtcLift.LIFT_POSITION_LOW;
            } else if (gamePad1.x || gamePad2.x || gamePad1.b || gamePad2.b) {
                targetPosition = FtcLift.LIFT_POSITION_MEDIUM;
            } else if (gamePad1.y || gamePad2.y) {
                targetPosition = FtcLift.LIFT_POSITION_HIGH;
            }

            if (targetPosition != LIFT_POSITION_INVALID) {
                move(targetPosition, false);
            }
        }
    }

    /**
     * Moves lift to the target motor encoder position.
     *
     * @param targetPosition The target motor encoder position.
     * @param waitTillEnd    When true, waits for lift to reach target position.
     */
    public void move(int targetPosition, boolean waitTillEnd) {
        if (liftEnabled) {
            targetPosition = Range.clip(targetPosition,
                    LIFT_POSITION_MINIMUM, LIFT_POSITION_HIGH);
            int currentPosition = liftMotor.getCurrentPosition();
            if (targetPosition != currentPosition) {
                // Must set motor position before setting motor mode.
                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }

            double liftPower = LIFT_ZERO_POWER;
            if (targetPosition > currentPosition) {
                liftPower = LIFT_UP_POWER;
            } else if (targetPosition < currentPosition) {
                liftPower = LIFT_DOWN_POWER;
            }

            liftMotor.setPower(liftPower);
            if (waitTillEnd) {
                double waitTime = estimateTravelTime(currentPosition, targetPosition);
                ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                while (!liftNearTarget(liftMotor.getCurrentPosition(), targetPosition) &&
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
                    liftMotor.getPower(), liftMotor.getCurrentPosition()));
        }

        FtcLogger.exit();
    }

    /**
     * Stops the lift by setting lift motor power to zero.
     */
    public void stop() {
        FtcLogger.enter();
        if (liftEnabled) {
            liftMotor.setPower(LIFT_ZERO_POWER);
        }

        FtcLogger.exit();
    }
}
