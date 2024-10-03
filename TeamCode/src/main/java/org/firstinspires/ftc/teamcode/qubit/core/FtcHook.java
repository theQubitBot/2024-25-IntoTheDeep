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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A class to manage the robot hook.
 */
public class FtcHook extends FtcSubSystem {
    private static final String TAG = "FtcHook";
    public static final String HOOK_MOTOR_NAME = "hookMotor";
    public static final int HOOK_MOTOR_LOW_POSITION = 5;
    public static final int HOOK_MOTOR_HIGH_POSITION = 7700;
    public static final double HOOK_UP_POWER = 1.0;
    public static final double HOOK_DOWN_POWER = -HOOK_UP_POWER;
    public static final double HOOK_ZERO_POWER = 0.0;
    private final boolean hookEnabled = false;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry = null;
    public FtcMotor hookMotor = null;

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (hookEnabled) {

            hookMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, HOOK_MOTOR_NAME));
            hookMotor.setDirection(DcMotorEx.Direction.FORWARD);
            hookMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            hookMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            hookMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hookMotor.setTargetPosition(HOOK_MOTOR_LOW_POSITION);
            hookMotor.setPower(HOOK_ZERO_POWER);

            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Operates the hook using the gamePads.
     * DPad up -> raise
     * DPad down -> lower
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
        FtcLogger.enter();
        if (hookEnabled &&
                // Debug mode or end game or both controllers are used
                (FtcUtils.DEBUG || runtime.seconds() > 90 ||
                        (gamePad1.dpad_up && gamePad2.dpad_up) ||
                        (gamePad1.dpad_down && gamePad2.dpad_down))) {

            if (gamePad1.dpad_up || gamePad2.dpad_up) {
                raise();
            } else if (gamePad1.dpad_down || gamePad2.dpad_down) {
                lower();
            }
        }

        FtcLogger.exit();
    }

    /**
     * Raises the hook by operating the hook motor.
     */
    public void raise() {
        FtcLogger.enter();
        if (hookEnabled) {
            int currentPosition = hookMotor.getCurrentPosition();
            double hookPower = HOOK_ZERO_POWER;
            if (currentPosition < HOOK_MOTOR_HIGH_POSITION) {
                // Must set motor position before setting motor mode.
                hookMotor.setTargetPosition(HOOK_MOTOR_HIGH_POSITION);
                hookMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                hookPower = HOOK_UP_POWER;
            }

            hookMotor.setPower(hookPower);
        }

        FtcLogger.exit();
    }

    /**
     * Lowers the hook by operating the hook motor.
     */
    private void lower() {
        FtcLogger.enter();
        if (hookEnabled && hookMotor != null) {
            int currentPosition = hookMotor.getCurrentPosition();
            double hookPower = HOOK_ZERO_POWER;
            if (currentPosition > HOOK_MOTOR_LOW_POSITION) {
                // Must set motor position before setting motor mode.
                hookMotor.setTargetPosition(HOOK_MOTOR_LOW_POSITION);
                hookMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                hookPower = HOOK_DOWN_POWER;
            }

            hookMotor.setPower(hookPower);
        }

        FtcLogger.exit();
    }




    /**
     * Displays hook telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (hookEnabled && telemetryEnabled && hookMotor != null) {
            telemetry.addData(TAG, String.format(Locale.US, "Motor: %d", hookMotor.getCurrentPosition()));
        }

        FtcLogger.exit();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();

        FtcLogger.exit();
    }

    /**
     * Stops the catapult.
     */
    public void stop() {
        FtcLogger.enter();
        if (hookEnabled) {
            if (hookMotor != null) {
                hookMotor.setPower(HOOK_ZERO_POWER);
            }
        }

        FtcLogger.exit();
    }
}
