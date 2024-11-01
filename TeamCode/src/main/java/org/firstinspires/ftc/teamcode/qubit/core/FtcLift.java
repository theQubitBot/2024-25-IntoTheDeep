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
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public static final int POSITION_HIGH = 2000;
    public static final int POSITION_MEDIUM = 450;
    public static final int POSITION_HANG = 1221;

    public static final int POSITION_LOW = 5;
    public static final int POSITION_MINIMUM = 0;
    public static final int POSITION_ERROR_MARGIN = 20;
    public static final int POSITION_INVALID = Integer.MIN_VALUE;
    public static final double UP_POWER = 1.0;
    public static final double DOWN_POWER = -UP_POWER;
    public static final double TRAVEL_TIME_MAX_MS = 3000;
    public static final long TRAVEL_TIME_LOW2MID_MS = 1000;
    public static final long TRAVEL_TIME_LOW2HIGH_MS = 2000;
    public static final long TRAVEL_TIME_MID2HIGH_MS = 1000;
    Deadline low2MidTravelDeadline, low2HighTravelDeadline, mid2HighTravelDeadline;
    private final boolean liftEnabled = true;
    public boolean telemetryEnabled = true;
    public static int endAutoOpLiftPosition = POSITION_MINIMUM;
    private Telemetry telemetry = null;
    public FtcMotor leftLiftMotor = null;
    public FtcMotor rightLiftMotor = null;
    DigitalChannel touch1;
    DigitalChannel touch2;

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
        return Range.clip(estimate, 0, TRAVEL_TIME_MAX_MS);
    }

    /**
     * Get the current lift position.
     *
     * @return The current lift position.
     */
    public int getPosition() {
        int leftPosition = POSITION_INVALID;
        FtcLogger.enter();
        if (liftEnabled) {
            leftPosition = leftLiftMotor.getCurrentPosition();
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
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (liftEnabled) {
            touch1 = hardwareMap.get(DigitalChannel.class, "touch_sensor1");
            touch1.setMode(DigitalChannel.Mode.INPUT);
            touch2 = hardwareMap.get(DigitalChannel.class, "touch_sensor2");
            touch2.setMode(DigitalChannel.Mode.INPUT);

            rightLiftMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, RIGHT_MOTOR_NAME));
            rightLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);
            rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            leftLiftMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, LEFT_MOTOR_NAME));
            leftLiftMotor.setDirection(DcMotorEx.Direction.FORWARD);
            leftLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            // Initialize lift
            rightLiftMotor.setTargetPosition(POSITION_LOW);
            rightLiftMotor.setPower(FtcMotor.ZERO_POWER);

            leftLiftMotor.setTargetPosition(POSITION_LOW);
            leftLiftMotor.setPower(FtcMotor.ZERO_POWER);

            low2MidTravelDeadline = new Deadline(TRAVEL_TIME_LOW2MID_MS, TimeUnit.MILLISECONDS);
            low2HighTravelDeadline = new Deadline(TRAVEL_TIME_LOW2HIGH_MS, TimeUnit.MILLISECONDS);
            mid2HighTravelDeadline = new Deadline(TRAVEL_TIME_MID2HIGH_MS, TimeUnit.MILLISECONDS);

            low2MidTravelDeadline.reset();
            mid2HighTravelDeadline.reset();
            mid2HighTravelDeadline.reset();

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
        return FtcUtils.areEqual(currentPosition, targetPosition, POSITION_ERROR_MARGIN);
    }

    /**
     * Operates lift based on gamePad inputs.
     *
     * @param gamePad1 The gamePad1 to control the lift operation.
     * @param gamePad2 The gamePad2 to control the lift operation.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
        if (liftEnabled) {
            int leftCurrentPosition = leftLiftMotor.getCurrentPosition();
            int rightcurrentPosition = rightLiftMotor.getCurrentPosition();

            int leftTargetPosition = leftCurrentPosition;
            int rightTargetPosition = rightcurrentPosition;

            if (!touch1.getState() && !touch2.getState()) {
                // both buttons are being pressed
                leftCurrentPosition = POSITION_LOW;
                rightcurrentPosition = POSITION_LOW;
            }

            // If lift zero is being reset, we want lower the lift physically as well.
            if (gamePad1.a || gamePad2.a) {
                leftTargetPosition = FtcLift.POSITION_LOW - endAutoOpLiftPosition;
                rightTargetPosition = FtcLift.POSITION_LOW - endAutoOpLiftPosition;
            } else if (gamePad1.y || gamePad2.y) {
                leftTargetPosition = FtcLift.POSITION_HIGH - endAutoOpLiftPosition;
                rightTargetPosition = FtcLift.POSITION_HIGH - endAutoOpLiftPosition;
            } else if (FtcUtils.hangInitiated(gamePad1, gamePad2, runtime)) {
                leftTargetPosition = FtcLift.POSITION_HANG - endAutoOpLiftPosition;
                rightTargetPosition = FtcLift.POSITION_HANG - endAutoOpLiftPosition;
            }

            if (leftTargetPosition != leftCurrentPosition ||
                    rightTargetPosition != rightcurrentPosition) {
                move(leftTargetPosition, rightTargetPosition, false);
            }
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
                    POSITION_MINIMUM - endAutoOpLiftPosition, POSITION_HIGH - endAutoOpLiftPosition);
            int leftCurrentPosition = leftLiftMotor.getCurrentPosition();
            if (leftTargetPosition != leftCurrentPosition) {
                // Must set motor position before setting motor mode.
                leftLiftMotor.setTargetPosition(leftTargetPosition);
                leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                liftPower = leftTargetPosition > leftCurrentPosition ? UP_POWER : DOWN_POWER;
                leftLiftMotor.setPower(liftPower);
            }

            rightTargetPosition = Range.clip(rightTargetPosition,
                    POSITION_MINIMUM - endAutoOpLiftPosition, POSITION_HIGH - endAutoOpLiftPosition);
            int rightCurrentPosition = rightLiftMotor.getCurrentPosition();
            if (rightTargetPosition != rightCurrentPosition) {
                // Must set motor position before setting motor mode.
                rightLiftMotor.setTargetPosition(rightTargetPosition);
                rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                liftPower = rightTargetPosition > rightCurrentPosition ? UP_POWER : DOWN_POWER;
                rightLiftMotor.setPower(liftPower);
            }

            if (waitTillCompletion) {
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
            telemetry.addData(TAG, String.format(Locale.US, "LLPower %.2f, LLPosition %d, RLPower %.2f, RLPosition %d",
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
            leftLiftMotor.setPower(FtcMotor.ZERO_POWER);
            rightLiftMotor.setPower(FtcMotor.ZERO_POWER);
        }

        FtcLogger.exit();
    }
}
