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

import java.util.Locale;

/**
 * A class to manage the robot catapult.
 */
public class FtcCatapult extends FtcSubSystem {
    private static final String TAG = "FtcCatapult";
    public static final String CATAPULT_SERVO_NAME = "droneServo";
    public static final double SET_POSITION = 0.5900;
    public static final double RELEASE_POSITION = 0.7000;
    public static final long CATAPULT_RELEASE_TIME_MS = 700;
    public static final long CATAPULT_SET_TIME_MS = 200;
    private final boolean catapultEnabled = false;
    public boolean telemetryEnabled = true;
    private FtcBot parent = null;
    private Telemetry telemetry = null;
    public FtcServo catapultServo = null;

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
        if (catapultEnabled) {
            catapultServo = new FtcServo(hardwareMap.get(Servo.class, CATAPULT_SERVO_NAME));
            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Operates the catapult using the gamePads.
     * gamePad options -> release
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
        FtcLogger.enter();
        if (catapultEnabled &&
                // Debug mode OR end game or both controllers are used
                (FtcUtils.DEBUG || runtime.seconds() > 90 || (gamePad1.start && gamePad2.start))) {
            if (gamePad1.start || gamePad2.start) {
                release(true);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Rotate to release the catapult.
     *
     * @param waitTillCompletion When true, waits till operation completes.
     */
    private void release(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (catapultEnabled) {
            if (parent == null) {
                // We are likely testing catapult
                catapultServo.setPosition(RELEASE_POSITION);
                if (waitTillCompletion) {
                    FtcUtils.sleep(CATAPULT_RELEASE_TIME_MS);
                }
            } else {
                // Release only if catapult is not already released.
                if (!FtcUtils.areEqual(catapultServo.getPosition(), RELEASE_POSITION, FtcUtils.EPSILON4)) {
                    catapultServo.setPosition(RELEASE_POSITION);
                    if (waitTillCompletion) {
                        FtcUtils.sleep(CATAPULT_RELEASE_TIME_MS);
                    }
                }
            }

            // reset catapult for next match
            set(false);
        }

        FtcLogger.exit();
    }

    /**
     * Rotate to set the catapult.
     *
     * @param waitTillCompletion When true, waits till operation completes.
     */
    public void set(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (catapultEnabled) {
            catapultServo.setPosition(SET_POSITION);
            if (waitTillCompletion) {
                FtcUtils.sleep(CATAPULT_SET_TIME_MS);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Displays catapult telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (catapultEnabled && telemetryEnabled) {
            telemetry.addData(TAG, String.format(Locale.US, "Position: %.4f",
                    catapultServo.getPosition()));
        }

        FtcLogger.exit();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (catapultEnabled) {
            catapultServo.getController().pwmEnable();
            set(false);
        }

        FtcLogger.exit();
    }

    /**
     * Stops the catapult.
     */
    public void stop() {
        FtcLogger.enter();
        if (catapultEnabled) {
            catapultServo.getController().pwmDisable();
        }

        FtcLogger.exit();
    }
}
