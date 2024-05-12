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
 * A class to manage the robot Arm.
 */
public class FtcArm extends FtcSubSystem {
    private static final String TAG = "FtcArm";
    public static final String ARM_SERVO_NAME = "tetrisServo";
    public static final double ARM_POSITION_DELIVERY_FLUSH = 0.8800;
    public static final double ARM_POSITION_RECEIVE = 0.4950;
    public static final double ARM_POSITION_ERROR_MARGIN = 0.0010;
    public static final long ARM_MOVE_TIME_MS = 400;
    private final boolean armEnabled = false;
    public boolean telemetryEnabled = true;
    private FtcBot parent;
    private Telemetry telemetry = null;
    public FtcServo armServo = null;
    public FtcArmAsyncExecutor armAsyncExecutor = null;

    public boolean armInPosition(double position) {
        return FtcUtils.areEqual(armServo.getPosition(), position, ARM_POSITION_ERROR_MARGIN);
    }

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, FtcBot parent) {
        FtcLogger.enter();
        this.parent = parent;
        this.telemetry = telemetry;
        if (armEnabled) {
            armServo = new FtcServo(hardwareMap.get(Servo.class, ARM_SERVO_NAME));

            // Stop any previous, lingering executors.
            if (armAsyncExecutor != null) {
                FtcLogger.error(TAG, "Found a previous instance of armAsyncExecutor");
                armAsyncExecutor.setOperation(FtcArmAsyncExecutor.ArmOperation.Stop, false);
            }

            // Start a new executor
            armAsyncExecutor = new FtcArmAsyncExecutor(this);
            Thread armAsyncExecutorThread = new Thread(armAsyncExecutor);
            armAsyncExecutorThread.start();

            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Operates arm based on gamePad inputs.
     *
     * @param gamePad1 The gamePad1 to control the arm operation.
     * @param gamePad2 The gamePad2 to control the arm operation.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2) {
        FtcLogger.enter();
        if (armEnabled) {
            if (gamePad1.right_trigger >= 0.5 || gamePad2.right_trigger >= 0.5) {
                if (parent == null) {
                    // We are likely testing arm
                    armAsyncExecutor.setOperation(FtcArmAsyncExecutor.ArmOperation.DeliverAndReceive, false);
                } else {
                /*
                - arm delivers if
                    - lift is in high position or
                    - lift is in medium position or
                    - lift is in low position and hand is in intake position
                 */
                    armAsyncExecutor.setOperation(FtcArmAsyncExecutor.ArmOperation.DeliverAndReceive, false);
                }
            } else if (gamePad1.right_bumper || gamePad2.right_bumper) {
                if (parent == null) {
                    // We are likely testing arm
                    armAsyncExecutor.setOperation(FtcArmAsyncExecutor.ArmOperation.ReceiveOnly, false);
                } else {
                    armAsyncExecutor.setOperation(FtcArmAsyncExecutor.ArmOperation.ReceiveOnly, false);
                }
            }
        }

        FtcLogger.exit();
    }

    /**
     * Moves arm to the given target position.
     *
     * @param position             The position to move the arm to.
     * @param waitTillMoveComplete True, if method should wait for arm to move to the given position.
     */
    public void move(double position, boolean waitTillMoveComplete) {
        FtcLogger.enter();
        if (armEnabled) {
            armServo.setPosition(position);
            if (waitTillMoveComplete) {
                FtcUtils.sleep(ARM_MOVE_TIME_MS);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Displays arm servo telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (armEnabled && telemetryEnabled) {
            telemetry.addData(TAG, String.format(Locale.US, "Position %.4f",
                    armServo.getPosition()));
        }

        FtcLogger.exit();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (armEnabled) {
            armServo.getController().pwmEnable();
            move(ARM_POSITION_RECEIVE, false);
        }

        FtcLogger.exit();
    }

    /**
     * Stops the arm.
     */
    public void stop() {
        FtcLogger.enter();
        if (armEnabled) {
            armAsyncExecutor.setOperation(FtcArmAsyncExecutor.ArmOperation.Stop, false);
            armAsyncExecutor = null;
            armServo.getController().pwmDisable();
        }

        // There is no need to explicitly move the servo upon stop as
        // the power is cut off to servo when Auto/Tele Op ends.
        FtcLogger.exit();
    }
}
