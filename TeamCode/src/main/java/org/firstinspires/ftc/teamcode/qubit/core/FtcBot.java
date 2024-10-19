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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;

/**
 * A class to manage the robot. This is essentially a composite design pattern.
 * Robot level operations are simply invoked on all subsystems.
 */
public class FtcBot extends FtcSubSystem {
    private static final String TAG = "FtcBot";
    private boolean telemetryEnabled = true;
    public FtcBulkRead bulkRead = null;
    public FtcDriveTrain driveTrain = null;

    // robot sub systems
    public FtcImu imu = null;
    public FtcLift lift = null;
    public FtcRelay relay = null;
    public MatchConfig config = null;
    private Telemetry telemetry = null;

    /* Constructor */
    public FtcBot() {
    }

    public void disableTelemetry() {
        FtcLogger.enter();
        telemetryEnabled = false;
        driveTrain.telemetryEnabled = false;
        imu.telemetryEnabled = false;
        lift.telemetryEnabled = false;
        relay.telemetryEnabled = false;
        FtcLogger.exit();
    }

    public void enableTelemetry() {
        FtcLogger.enter();
        telemetryEnabled = true;
        driveTrain.telemetryEnabled = true;
        imu.telemetryEnabled = true;
        lift.telemetryEnabled = true;
        relay.telemetryEnabled = true;
        FtcLogger.exit();
    }

    /**
     * Initialize standard Hardware interfaces
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     * @param autoOp      If true, initializes the webcams. Otherwise initializes only IMU.
     *                    Typically you would save on processing by not initializing the webcams or IMU
     *                    unnecessarily
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean autoOp) {
        FtcLogger.enter();
        this.telemetry = telemetry;

        bulkRead = new FtcBulkRead();
        bulkRead.init(hardwareMap, telemetry);
        config = new MatchConfig();
        config.init(hardwareMap, telemetry);

        driveTrain = new FtcDriveTrain(this);
        driveTrain.setDriveTypeAndMode(DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.POINT_OF_VIEW_DRIVE);
        driveTrain.init(hardwareMap, telemetry);

        imu = new FtcImu();
        imu.init(hardwareMap, telemetry);

        lift = new FtcLift();
        lift.init(hardwareMap, telemetry);

        relay = new FtcRelay();
        relay.init(hardwareMap, telemetry);

        telemetry.addData(TAG, "initialized");
        FtcLogger.exit();
    }

    /**
     * Operate the robot in tele operation.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
        FtcLogger.enter();

        bulkRead.clearBulkCache();

        // Drive operation
        driveTrain.operate(gamePad1, gamePad2, loopTime);

        // Relay ideally operates before lift during hang (endgame).
        relay.operate(gamePad1, gamePad2, runtime);
        lift.operate(gamePad1, gamePad2, runtime);
        if (telemetryEnabled) {
            imu.showTelemetry();
            showGamePadTelemetry(gamePad1);
            driveTrain.showTelemetry();
            lift.showTelemetry();
            relay.showTelemetry();
        }

        FtcLogger.exit();
    }

    /**
     * Display game pad telemetry.
     *
     * @param gamePad The gamePad.
     */
    public void showGamePadTelemetry(Gamepad gamePad) {
        FtcLogger.enter();
        if (telemetryEnabled) {
            telemetry.addData("LeftStick", "%.2f %.2f",
                    gamePad.left_stick_x, gamePad.left_stick_y);
            telemetry.addData("RightStick", "%.2f, %.2f",
                    gamePad.right_stick_x, gamePad.right_stick_y);
        }

        FtcLogger.exit();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        relay.start();
        FtcLogger.exit();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop() {
        FtcLogger.enter();
        if (imu != null) {
            imu.stop();
        }

        if (driveTrain != null) {
            driveTrain.stop();
        }

        if (lift != null) {
            lift.stop();
        }

        if (relay != null) {
            relay.stop();
        }

        FtcLogger.exit();
    }
}
