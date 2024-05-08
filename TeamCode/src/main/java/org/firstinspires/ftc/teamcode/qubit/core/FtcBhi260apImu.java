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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * A class to manage the built-in IMU.
 */
public class FtcBhi260apImu extends FtcSubSystem {
    private static final String TAG = "FtcBhi260apImu";
    public boolean telemetryEnabled = true;
    Telemetry telemetry = null;
    private IMU imu = null;
    private boolean imuIsGood = true;
    private static final Object directionLock = new Object();
    private double heading = 0.0; // Increases when robot turns left
    private double pitch = 0.0;
    private double roll = 0.0;

    /* Constructor */
    public FtcBhi260apImu() {
    }

    /**
     * A thread safe method to get IMU heading.
     *
     * @return The IMU heading.
     */
    public double getHeading() {
        double currentHeading;
        synchronized (directionLock) {
            currentHeading = heading;
        }

        return currentHeading;
    }

    /**
     * A thread safe method to get IMU pitch.
     *
     * @return The IMU pitch.
     */
    public double getPitch() {
        double currentPitch;
        synchronized (directionLock) {
            currentPitch = pitch;
        }

        return currentPitch;
    }

    /**
     * A thread safe method to get IMU roll.
     *
     * @return The IMU roll.
     */
    public double getRoll() {
        double currentRoll;
        synchronized (directionLock) {
            currentRoll = roll;
        }

        return currentRoll;
    }

    public boolean imuIsGood(){
        boolean isGood;
        synchronized (directionLock) {
            isGood = imuIsGood;
        }

        return isGood;
    }

    /**
     * Initialize IMU. Robot must be still during IMU initialization (calibration).
     * See <a href="https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html">...</a>
     * for initialization times.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry object to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        imu = hardwareMap.get(IMU.class, "imu");
        if (imu != null) {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();

            // Get a reading before the async reader is started.
            read();
            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not initialized");
        }

        FtcLogger.exit();
    }

    /**
     * Read the IMU and store information for later use.
     */
    public void read() {
        FtcLogger.enter();
        if (imu != null) {
            YawPitchRollAngles yawPitchRollAngles = imu.getRobotYawPitchRollAngles();
            synchronized (directionLock) {
                heading = yawPitchRollAngles.getYaw(AngleUnit.DEGREES);
                pitch = yawPitchRollAngles.getPitch(AngleUnit.DEGREES);
                roll = yawPitchRollAngles.getRoll(AngleUnit.DEGREES);
                imuIsGood = yawPitchRollAngles.getAcquisitionTime() != 0;
            }
        }

        FtcLogger.exit();
    }

    /**
     * Display IMU (gyro) telemetry.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (telemetryEnabled) {
            telemetry.addData("Heading/Yaw", "%.2f", getHeading());
            telemetry.addData("Pitch", "%.2f", getPitch());
            telemetry.addData("Roll", "%.2f", getRoll());
        }

        FtcLogger.exit();
    }
}
