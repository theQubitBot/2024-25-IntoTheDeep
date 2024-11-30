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

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the sample delivery.
 */
public class FtcArm extends FtcSubSystem {
    private static final String TAG = "FtcArm";
    public static final String ARM_SERVO_NAME = "armServo";
    public static final double ARM_FORWARD_POSITION = 0.5050;
    public static final double ARM_BACKWARD_POSITION = 0.6100;
    public static final int ARM_FORWARD_TRAVEL_TIME = 750; // milliseconds
    public static final int ARM_BACKWARD_TRAVEL_TIME = 1250; // milliseconds
    private final boolean armEnabled = true;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    private final FtcBot parent;
    private FtcServo armServo = null;

    public FtcArm(FtcBot robot) {
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
        if (armEnabled) {
            armServo = new FtcServo(hardwareMap.get(Servo.class, ARM_SERVO_NAME));
            armServo.setDirection(Servo.Direction.REVERSE);

            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    public boolean isBackward() {
        FtcLogger.enter();
        boolean backward = false;
        if (armEnabled) {
            backward = FtcUtils.areEqual(armServo.getPosition(), ARM_BACKWARD_POSITION, FtcUtils.EPSILON3);
        }

        FtcLogger.exit();
        return backward;
    }

    public boolean isForward() {
        FtcLogger.enter();
        boolean forward = false;
        if (armEnabled) {
            forward = FtcUtils.areEqual(armServo.getPosition(), ARM_FORWARD_POSITION, FtcUtils.EPSILON3);
        }

        FtcLogger.exit();
        return forward;
    }

    /**
     * Operates the arm using the gamePads.
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     * @param runtime  The tele op runtime.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
        FtcLogger.enter();

        if (armEnabled) {
            if (!FtcUtils.DEBUG && FtcUtils.gameOver(runtime)) {
                return;
            }

            if (gamePad1.left_bumper || gamePad2.left_bumper) {
                if (
                    // Both drivers force manual override
                        (gamePad1.left_bumper && gamePad2.left_bumper) ||
                                // GamePad1 manual override
                                (gamePad1.share && gamePad1.left_bumper) ||
                                // GamePad2 manual override
                                (gamePad2.share && gamePad2.left_bumper) ||
                                // lift is at medium or high position
                                (parent != null && (parent.lift.atPosition(FtcLift.POSITION_LOW_BASKET) ||
                                        parent.lift.atPosition(FtcLift.POSITION_HIGH_BASKET) ||
                                        parent.lift.atPosition(FtcLift.POSITION_HANG) ||
                                        // intake is down or horizontal
                                        parent.intake.isHorizontal() || parent.intake.isDown()))) {
                    moveBackward(false);
                } else {
                    moveForward(false);
                }
            } else {
                // Automatically move arm forward when bumper is not pressed.
                moveForward(false);
            }
        }

        FtcLogger.exit();
    }

    public void moveBackward(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (armEnabled) {
            armServo.setPosition(ARM_BACKWARD_POSITION);
            if (waitTillCompletion) {
                FtcUtils.sleep(ARM_BACKWARD_TRAVEL_TIME);
            }
        }

        FtcLogger.exit();
    }

    public void moveForward(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (armEnabled) {
            armServo.setPosition(ARM_FORWARD_POSITION);
            if (waitTillCompletion) {
                FtcUtils.sleep(ARM_FORWARD_TRAVEL_TIME);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Displays telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (armEnabled && telemetryEnabled && armServo != null) {
            telemetry.addData(TAG, String.format(Locale.US, "%5.4f",
                    armServo.getPosition()));
        }

        FtcLogger.exit();
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (armEnabled) {
            if (armServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
                armServo.getController().pwmEnable();
            }

            moveForward(false);
        }

        FtcLogger.exit();
    }

    /**
     * Stops the Arm.
     */
    public void stop() {
        FtcLogger.enter();
        if (armEnabled) {
            if (armServo != null &&
                    armServo.getController().getPwmStatus() != ServoController.PwmStatus.DISABLED) {
                armServo.getController().pwmDisable();
            }
        }

        FtcLogger.exit();
    }
}
