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
 * A class to manage the flag.
 */
public class FtcFlag extends FtcSubSystem {
    private static final String TAG = "FtcFlag";
    public static final String FLAG_SERVO_NAME = "flagServo";
    public static final double FLAG_UP_POSITION = 0.6100;
    public static final double FLAG_DOWN_POSITION = 0.4800;
    public static final int FLAG_TRAVEL_TIME = 1200; // milliseconds
    private final boolean flagEnabled = true;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    public FtcServo flagServo = null;

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (flagEnabled) {
            flagServo = new FtcServo(hardwareMap.get(Servo.class, FLAG_SERVO_NAME));
            flagServo.setDirection(Servo.Direction.REVERSE);
            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Operates the flag using the gamePads.
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2) {
        FtcLogger.enter();
        FtcLogger.exit();
    }

    /**
     * Raise the flag.
     *
     * @param waitTillCompletion When true, waits till the flag is raised.
     */
    public void raise(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (flagEnabled) {
            flagServo.setPosition(FLAG_UP_POSITION);
            if (waitTillCompletion) {
                FtcUtils.sleep(FLAG_TRAVEL_TIME);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Lower the flag.
     *
     * @param waitTillCompletion When true, waits till the flag is lowered.
     */
    public void lower(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (flagEnabled) {
            flagServo.setPosition(FLAG_DOWN_POSITION);
            if (waitTillCompletion) {
                FtcUtils.sleep(FLAG_TRAVEL_TIME);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Displays intake telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (flagEnabled && telemetryEnabled && flagServo != null) {
            telemetry.addData(TAG, String.format(Locale.US, "flag: %.4f", flagServo.getPosition()));
        }

        FtcLogger.exit();
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (flagEnabled) {
            flagServo.getController().pwmEnable();
            lower(false);
        }

        FtcLogger.exit();
    }

    /**
     * Stops the flag.
     */
    public void stop() {
        FtcLogger.enter();
        if (flagEnabled) {
            if (flagServo != null) {
                flagServo.getController().pwmDisable();
            }
        }

        FtcLogger.exit();
    }
}
