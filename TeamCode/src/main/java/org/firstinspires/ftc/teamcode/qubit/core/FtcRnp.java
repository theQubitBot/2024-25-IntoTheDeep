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
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * A class to manage the input extension/retraction.
 */
public class FtcRnp extends FtcSubSystem {
    private static final String TAG = "FtcRnp";
    public static final String RNP_SERVO_NAME = "rnpServo";
    public static final double RNP_EXTEND_POWER = 1.0;
    public static final double RNP_RETRACT_POWER = 0.0;
    public static final double RNP_STOP_POWER = FtcServo.MID_POSITION;
    public static final int RNP_EXTEND_TRAVEL_TIME = 750; // milliseconds
    public static final int RNP_RETRACT_TRAVEL_TIME = 2000; // milliseconds
    private final boolean rnpEnabled = true;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    private FtcServo rnpServo = null;
    public static final String EXTEND_TOUCH_SENSOR = "extendTouchSensor";
    public static final String RETRACT_TOUCH_SENSOR = "retractTouchSensor";
    private TouchSensor extendTouchSensor = null;
    private TouchSensor retractTouchSensor = null;
    private final boolean useTouchSensors = true;

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
            rnpServo = new FtcServo(hardwareMap.get(Servo.class, RNP_SERVO_NAME));
            rnpServo.setDirection(Servo.Direction.REVERSE);

            if (useTouchSensors) {
                extendTouchSensor = hardwareMap.get(TouchSensor.class, EXTEND_TOUCH_SENSOR);
                retractTouchSensor = hardwareMap.get(TouchSensor.class, RETRACT_TOUCH_SENSOR);
            }

            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Operates the rack and pinion using the gamePads.
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     * @param runtime  The tele op runtime.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
        FtcLogger.enter();

        if (!FtcUtils.DEBUG && FtcUtils.gameOver(runtime)) {
            stop(false);
        } else if (FtcUtils.hangInitiated(gamePad1, gamePad2, runtime)) {
            retract(false);
        } else if (gamePad1.dpad_up || gamePad2.dpad_up) {
            if (
                // Both drivers force manual override
                    (gamePad1.dpad_up && gamePad2.dpad_up) ||
                            // GamePad1 manual override
                            (gamePad1.share && gamePad1.dpad_up) ||
                            // GamePad2 manual override
                            (gamePad2.share && gamePad2.dpad_up)) {
                extend(false);
            } else if (useTouchSensors && extendTouchSensor.isPressed()) {
                // Already extended, stop operation
                stop(false);
            } else {
                extend(false);
            }
        } else if (gamePad1.dpad_down || gamePad2.dpad_down) {
            if (
                // Both drivers force manual override
                    (gamePad1.dpad_down && gamePad2.dpad_down) ||
                            // GamePad1 manual override
                            (gamePad1.share && gamePad1.dpad_down) ||
                            // GamePad2 manual override
                            (gamePad2.share && gamePad2.dpad_down)) {
                retract(false);
            } else if (useTouchSensors && retractTouchSensor.isPressed()) {
                // Already retracted, stop operation
                stop(false);
            } else {
                retract(false);
            }
        } else if (FtcUtils.lastNSeconds(runtime, 10)) {
            retract(false);
        } else {
            stop(false);
        }

        FtcLogger.exit();
    }

    /**
     * Extend the intake.
     */
    public void extend(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (rnpEnabled) {
            if (useTouchSensors) {
                if (extendTouchSensor.isPressed()) {
                    rnpServo.setPosition(RNP_STOP_POWER);
                } else {
                    rnpServo.setPosition(RNP_EXTEND_POWER);
                }
            } else {
                rnpServo.setPosition(RNP_EXTEND_POWER);
            }

            if (waitTillCompletion) {
                Deadline travelDeadline = new Deadline(RNP_EXTEND_TRAVEL_TIME, TimeUnit.MILLISECONDS);
                while (!travelDeadline.hasExpired()) {
                    if (useTouchSensors) {
                        if (extendTouchSensor.isPressed()) {
                            break;
                        }
                    }

                    FtcUtils.sleep(FtcUtils.CYCLE_MS);
                }

                rnpServo.setPosition(RNP_STOP_POWER);
            }
        }

        FtcLogger.exit();
    }

    public boolean isExtended() {
        FtcLogger.enter();
        boolean extended = false;
        if (rnpEnabled) {
            if (useTouchSensors) {
                extended = extendTouchSensor.isPressed();
            }
        }

        FtcLogger.exit();
        return extended;
    }

    public boolean isRetracted() {
        FtcLogger.enter();
        boolean retracted = false;
        if (rnpEnabled) {
            if (useTouchSensors) {
                retracted = retractTouchSensor.isPressed();
            }
        }

        FtcLogger.exit();
        return retracted;
    }

    /**
     * Retract the intake.
     */
    public void retract(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (rnpEnabled) {
            if (useTouchSensors) {
                if (retractTouchSensor.isPressed()) {
                    rnpServo.setPosition(RNP_STOP_POWER);
                } else {
                    rnpServo.setPosition(RNP_RETRACT_POWER);
                }
            } else {
                rnpServo.setPosition(RNP_RETRACT_POWER);
            }

            if (waitTillCompletion) {
                Deadline travelDeadline = new Deadline(RNP_RETRACT_TRAVEL_TIME, TimeUnit.MILLISECONDS);
                while (!travelDeadline.hasExpired()) {
                    if (useTouchSensors) {
                        if (retractTouchSensor.isPressed()) {
                            break;
                        }
                    }

                    FtcUtils.sleep(FtcUtils.CYCLE_MS);
                }

                rnpServo.setPosition(RNP_STOP_POWER);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Displays intake telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (rnpEnabled && telemetryEnabled && rnpServo != null) {
            String message = String.format(Locale.US, "%5.4f",
                    rnpServo.getPosition());
            if (useTouchSensors) {
                message += String.format(Locale.US, " extended: %b, retracted: %b",
                        extendTouchSensor.isPressed(), retractTouchSensor.isPressed());
            }

            telemetry.addData(TAG, message);
        }

        FtcLogger.exit();
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (rnpEnabled) {
            if (rnpServo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
                rnpServo.getController().pwmEnable();
            }

            rnpServo.setPosition(RNP_STOP_POWER);
        }

        FtcLogger.exit();
    }

    /**
     * Stops the rack and pinion motion.
     *
     * @param waitTillCompletion When true, waits for the extension/retraction operation to complete
     */
    public void stop(boolean waitTillCompletion) {
        FtcLogger.enter();
        if (rnpEnabled) {
            if (waitTillCompletion) {
                Deadline travelDeadline =
                        new Deadline(Math.max(RNP_EXTEND_TRAVEL_TIME, RNP_RETRACT_TRAVEL_TIME), TimeUnit.MILLISECONDS);
                while (!travelDeadline.hasExpired()) {
                    // Sleep before check to allow extension/retraction operation to commence
                    FtcUtils.sleep(FtcUtils.CYCLE_MS);
                    if (useTouchSensors) {
                        if (extendTouchSensor.isPressed() || retractTouchSensor.isPressed()) {
                            break;
                        }
                    }
                }
            }

            rnpServo.setPosition(RNP_STOP_POWER);
        }

        FtcLogger.exit();
    }
}
