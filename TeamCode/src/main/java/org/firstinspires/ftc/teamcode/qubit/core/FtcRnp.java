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
 * A class to manage the robot sample/specimen delivery.
 */
public class FtcRnp extends FtcSubSystem {
    private static final String TAG = "FtcRnp";
    public static final String RNP_SERVO_NAME = "rnpServo";
    public static final double RNP_EXTEND_POWER = 1.0;
    public static final double RNP_RETRACT_POWER = 0.0;
    public static final double RNP_STOP_POWER = FtcServo.MID_POSITION;
    public static final int RNP_TRAVEL_TIME = 2000; // milliseconds
    private final boolean rnpEnabled = true;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    public FtcServo rackNPinionServo = null;

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (rnpEnabled) {
            rackNPinionServo = new FtcServo(hardwareMap.get(Servo.class, RNP_SERVO_NAME));
            rackNPinionServo.setDirection(Servo.Direction.REVERSE);

            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Operates the relay using the gamePads.
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     * @param runtime  The tele op runtime.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
        FtcLogger.enter();

        if (FtcUtils.gameOver(runtime)) {
            rnpStop(false);
        } else if (gamePad1.dpad_up || gamePad2.dpad_up) {
            rnpExtend(false);
        } else if (gamePad1.dpad_down || gamePad2.dpad_down ||
                FtcUtils.hangInitiated(gamePad1, gamePad2, runtime)) {
            rnpRetract(false);
        } else {
            rnpStop(false);
        }

        FtcLogger.exit();
    }

    /**
     * Extend the intake.
     */
    public void rnpExtend(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (rnpEnabled) {
            rackNPinionServo.setPosition(RNP_EXTEND_POWER);
            if (waitTillCompletion) {
                rnpStop(true);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Retract the intake.
     */
    public void rnpRetract(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (rnpEnabled) {
            rackNPinionServo.setPosition(RNP_RETRACT_POWER);
            if (waitTillCompletion) {
                rnpStop(true);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Stops the rack and pinion motion.
     *
     * @param waitTillCompletion When true, waits for the extension/retraction operation to complete
     */
    public void rnpStop(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (rnpEnabled) {
            // Wait for previous extension/retraction operation to complete.
            if (waitTillCompletion) {
                FtcUtils.sleep(RNP_TRAVEL_TIME);
            }

            rackNPinionServo.setPosition(RNP_STOP_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Displays intake telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (rnpEnabled && telemetryEnabled && rackNPinionServo != null) {
            telemetry.addData(TAG, String.format(Locale.US, "rnp: %5.4f",
                    rackNPinionServo.getPosition()));
        }

        FtcLogger.exit();
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (rnpEnabled) {
            rackNPinionServo.getController().pwmEnable();
            rnpStop(false);
        }

        FtcLogger.exit();
    }

    /**
     * Stops the Relay.
     */
    public void stop() {
        FtcLogger.enter();
        if (rnpEnabled) {
            if (rackNPinionServo != null) {
                rackNPinionServo.getController().pwmDisable();
            }
        }

        FtcLogger.exit();
    }
}
