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
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;

import java.io.File;
import java.io.IOException;

/**
 * A configuration class to hold match configuration data.
 * Configuration is saved to a file and persists across
 * auto ops, tele ops and reboots.
 */
public class MatchConfig {
    private static final String TAG = "MatchConfig";
    public static final String MATCH_CONFIG_FILENAME = "matchConfig.txt";
    private static final long MAX_START_DELAY_SECONDS = 10;
    private boolean gamePad1Connected, gamePad2Connected;
    public boolean configIsComplete;
    private final boolean configFeatureEnabled = true;
    private boolean lastDPadUpPressed = false, lastDPadDownPressed = false;
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;

    public AllianceColorEnum allianceColor;
    public RobotPositionEnum robotPosition;
    public long delayInSeconds;

    /**
     * Constructor
     */
    public MatchConfig() {
        if (configFeatureEnabled) {
            reset();
        }
    }

    /**
     * Initialize standard Hardware interfaces.
     * Reads match configuration from the configuration file.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        readFromFile();

        FtcLogger.exit();
    }

    /**
     * Determines if the gamePad is connected to the driver station.
     *
     * @param gamePad The gamePad to evaluate.
     * @return true, if gamePad is connected, false otherwise.
     */
    private boolean isGamePadConnected(Gamepad gamePad) {
        // GamePad is assumed to be connected if any of the inputs
        // is not a default input.
        return gamePad.getGamepadId() != Gamepad.ID_UNASSOCIATED || gamePad.getUser() != null ||
                gamePad.a || gamePad.b || gamePad.x || gamePad.y ||
                gamePad.left_bumper || gamePad.left_trigger > 0.1 ||
                gamePad.right_bumper || gamePad.right_trigger > 0.1 ||
                gamePad.dpad_left || gamePad.dpad_right || gamePad.dpad_up || gamePad.dpad_down ||
                Math.abs(gamePad.left_stick_x) > 0.1 || Math.abs(gamePad.left_stick_y) > 0.1 ||
                Math.abs(gamePad.right_stick_x) > 0.1 || Math.abs(gamePad.right_stick_y) > 0.1 ||
                gamePad.left_stick_button || gamePad.right_stick_button;
    }

    /**
     * Reads match configuration from the saved configuration file.
     */
    private void readFromFile() {
        FtcLogger.enter();

        if (configFeatureEnabled) {
            try {
                File file = AppUtil.getInstance().getSettingsFile(MATCH_CONFIG_FILENAME);
                String data = ReadWriteFile.readFileOrThrow(file);
                MatchConfig savedMatchConfig = FtcUtils.deserialize(data, MatchConfig.class);
                allianceColor = savedMatchConfig.allianceColor;
                robotPosition = savedMatchConfig.robotPosition;
                delayInSeconds = savedMatchConfig.delayInSeconds;
            } catch (IOException e) {
                reset();
            } catch (Exception e) {
                // First time around, config file is absent. Do nothing
                reset();
            }
        }

        FtcLogger.exit();
    }

    /**
     * Resets the match configuration to default values.
     */
    public void reset() {
        FtcLogger.enter();
        allianceColor = AllianceColorEnum.RED;
        robotPosition = RobotPositionEnum.RIGHT;
        delayInSeconds = 0;
        gamePad1Connected = gamePad2Connected = false;
        configIsComplete = !configFeatureEnabled;
        FtcLogger.exit();
    }

    /**
     * Updates match configuration based on gamPad inputs.
     *
     * @param gamePad1 The first gamPad to use.
     * @param gamePad2 The second gamPad to use.
     */
    public void update(Gamepad gamePad1, Gamepad gamePad2) {
        if (configFeatureEnabled) {
            // Configure alliance color
            if (gamePad1.right_bumper || gamePad1.left_bumper ||
                    gamePad2.right_bumper || gamePad2.left_bumper) {
                allianceColor = AllianceColorEnum.BLUE;
            } else if (gamePad1.right_trigger > 0.5 || gamePad1.left_trigger > 0.5 ||
                    gamePad2.right_trigger > 0.5 || gamePad2.left_trigger > 0.5) {
                allianceColor = AllianceColorEnum.RED;
            }

            // Configure robot position on the field
            if (gamePad1.dpad_left || gamePad2.dpad_left) {
                robotPosition = RobotPositionEnum.LEFT;
            } else if (gamePad1.dpad_right || gamePad2.dpad_right) {
                robotPosition = RobotPositionEnum.RIGHT;
            }

            // Configure initial delay
            if (gamePad1.dpad_up || gamePad2.dpad_up) {
                if (!lastDPadUpPressed) {
                    lastDPadUpPressed = true;
                    delayInSeconds = Math.min(delayInSeconds + 1, MAX_START_DELAY_SECONDS);
                }
            } else if (gamePad1.dpad_down || gamePad2.dpad_down) {
                if (!lastDPadDownPressed) {
                    lastDPadDownPressed = true;
                    delayInSeconds = Math.max(delayInSeconds - 1, 0);
                }
            } else {
                lastDPadUpPressed = false;
                lastDPadDownPressed = false;
            }

            // Evaluate gamePad connections, if not connected.
            if (!gamePad1Connected) {
                gamePad1Connected = isGamePadConnected(gamePad1);
            }

            if (!gamePad2Connected) {
                gamePad2Connected = isGamePadConnected(gamePad2);
            }
        }
    }

    /**
     * Saves the configuration to the configuration file.
     */
    public void saveToFile() {
        FtcLogger.enter();
        if (configFeatureEnabled) {
            File file = AppUtil.getInstance().getSettingsFile(MATCH_CONFIG_FILENAME);
            ReadWriteFile.writeFile(file, FtcUtils.serialize(this));
        }

        FtcLogger.exit();
    }

    /**
     * Displays Match Configuration on Driver Station screen.
     */
    public void showConfiguration() {
        if (configFeatureEnabled) {
            telemetry.addData("Match configuration", "");
            telemetry.addData(FtcUtils.TAG, "%s alliance, %s robot position",
                    allianceColor, robotPosition);
            telemetry.addData(FtcUtils.TAG, "start delay %d seconds", delayInSeconds);
        }
    }

    /**
     * Displays Match Configuration Command Menu on Driver Station screen.
     */
    public void showMenu() {
        if (configFeatureEnabled) {
            telemetry.addData("Match Configuration Command Menu", "");
            telemetry.addData("Bumper", "alliance color BLUE");
            telemetry.addData("Trigger", "alliance color RED");
            telemetry.addData("dPad", "left: LEFT position, right: RIGHT position");
            telemetry.addData("Start delay", "dPad up: increase, down: decrease");
        }
    }
}
