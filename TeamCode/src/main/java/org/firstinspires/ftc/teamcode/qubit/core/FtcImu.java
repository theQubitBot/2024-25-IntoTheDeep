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

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A class to manage the built-in IMU.
 */
public class FtcImu extends FtcSubSystem {
    public static final String TAG = "FtcImu";
    static final double HEADING_THRESHOLD = 0.20;
    public boolean telemetryEnabled = true;
    Telemetry telemetry = null;

    private FtcBhi260apImu bhi260apImu = null;
    private FtcGoBoDriver ftcGoBoDriver = null;
    private final boolean useBhi260apImu = true;
    private final boolean useGoBoDriver = true;
    public static double endAutoOpHeading = 0;
    private double initialTeleOpHeading = 0.0;
    private double initialGyroHeadingToDetermineDrift = 0.0;
    private double initialTeleOpPitch = 0.0;
    private double initialTeleOpRoll = 0.0;

    // PERFORMANCE
    // Set to true to enable asynchronous background update of IMU values.
    public final boolean asyncUpdaterEnabled = false;
    private static final Object directionLock = new Object();
    private FtcImuAsyncUpdater imuAsyncUpdater = null;

    private static double heading = 0.0; // Increases when robot turns left
    private static double pitch = 0.0; // Increases when front rises
    private static double roll = 0.0; // Increases when left rises

    // PERFORMANCE
    // Simple mechanism to ensure a single IMU read per loop, and only when needed.
    private boolean gyroAlreadyRead = false;

    /* Constructor */
    public FtcImu() {
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

    /**
     * Determine the difference between the target heading and the robot's current heading
     *
     * @param targetHeading Desired heading (relative to global reference established at last gyro Reset).
     * @return headingOffset Degrees in the range +/- 180. Centered on the robot's frame of reference
     * Positive headingOffset means the robot should spin Counter Clockwise (CCW) to reduce error.
     */
    public double getHeadingOffset(double targetHeading) {
        FtcLogger.enter();

        // Z increases when robot turns left!
        // Yaw is never more than 180 or less than -180
        targetHeading = normalize(targetHeading, AngleUnit.DEGREES);
        double currentHeading = getHeading();
        double headingOffset;
        if (targetHeading * currentHeading >= 0) {
            // Target and current are either both positive or both negative
            headingOffset = currentHeading - targetHeading;
        } else {
            // They are of opposite signs
            headingOffset = targetHeading + currentHeading;
        }

        FtcLogger.exit();
        return headingOffset;
    }

    /**
     * Returns desired steering correction force within [-1,1] range.
     * Positive correction force implies robot should steer right
     *
     * @param headingOffset Robot heading offset in degrees
     * @param pCoEff        Proportional Gain Coefficient
     * @return The steering correction after applying the proportional gain coefficient
     */
    public double getSteeringCorrection(double headingOffset, double pCoEff) {
        FtcLogger.enter();
        double steeringCorrection = headingOffset * pCoEff;
        FtcLogger.exit();
        return steeringCorrection;
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
        // Save reference to Hardware map
        this.telemetry = telemetry;
        if (useBhi260apImu) {
            bhi260apImu = new FtcBhi260apImu();
            bhi260apImu.init(hardwareMap, telemetry);
        }

        if (useGoBoDriver) {
            ftcGoBoDriver = new FtcGoBoDriver();
            ftcGoBoDriver.init(hardwareMap, telemetry);
        }

        if (useBhi260apImu || useGoBoDriver) {
            // Get a reading before the async reader is started.
            readAsync();
            if (asyncUpdaterEnabled) {
                if (imuAsyncUpdater != null) {
                    // Stop any previously executing updater
                    imuAsyncUpdater.stop();
                }

                imuAsyncUpdater = new FtcImuAsyncUpdater(this);
                Thread imuUpdaterThread = new Thread(imuAsyncUpdater);
                imuUpdaterThread.start();
            }

            if (endAutoOpHeading == 0) {
                initialTeleOpHeading = getHeading();
            } else {
                initialTeleOpHeading = endAutoOpHeading;
            }

            initialTeleOpRoll = getRoll();
            initialTeleOpPitch = getPitch();

            initialGyroHeadingToDetermineDrift = getHeading();
            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not initialized");
        }

        FtcLogger.exit();
    }

    /**
     * A simple logic to determine if IMU is drifting.
     *
     * @return True, if IMU is drifting.
     */
    public boolean isGyroDrifting() {
        read();
        return !FtcUtils.areEqual(
                initialGyroHeadingToDetermineDrift, getHeading(), FtcUtils.EPSILON1);
    }

    /**
     * Normalize angle to be within [-180, 179.9999] degrees
     * or to be within [-Pi, Pi] radians.
     *
     * @param angle     The angle to normalize.
     * @param angleUnit The angle units
     * @return Normalized angle.
     */
    public static double normalize(double angle, AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.DEGREES) {
            while (angle >= 180) angle -= 180.0;
            while (angle <= -180) angle += 180.0;
        } else {
            while (angle >= Math.PI) angle -= Math.PI;
            while (angle <= -Math.PI) angle += Math.PI;
        }

        return angle;
    }

    /**
     * Read the IMU and store information for later use.
     */
    public void read() {
        if (!asyncUpdaterEnabled) {
            readAsync();
        }
    }

    /**
     * Read the IMU and store information for later use.
     * Invoked by async updater directly.
     * This may be invoked from other classes.
     */
    public void readAsync() {
        if (useBhi260apImu && bhi260apImu != null && bhi260apImu.imuIsGood()) {
            bhi260apImu.read();
            synchronized (directionLock) {
                heading = normalize(bhi260apImu.getHeading(), AngleUnit.DEGREES);
                roll = normalize(bhi260apImu.getRoll(), AngleUnit.DEGREES);
                pitch = normalize(bhi260apImu.getPitch(), AngleUnit.DEGREES);
            }
        } else if (useGoBoDriver && ftcGoBoDriver != null) {
            synchronized (directionLock) {
                heading = normalize(ftcGoBoDriver.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES);
            }
        } else {
            // All IMUs are bad or not enabled
            // Fallback to the next IMU?!!
            if (telemetryEnabled) {
                telemetry.addData(TAG, "All IMUs are dead.");
            }
        }
    }

    /**
     * Read the IMU if not already read.
     */
    public void readOnce() {
        if (!gyroAlreadyRead) {
            read();
            gyroAlreadyRead = true;
        }
    }

    /**
     * If the IMU has already been read, then reset so that another read may occur.
     */
    public void resetReadOnce() {
        gyroAlreadyRead = false;
    }

    /**
     * Display IMU (gyro) telemetry.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (telemetryEnabled) {
            if (useBhi260apImu && bhi260apImu != null) {
                bhi260apImu.showTelemetry();
            }

            if (useGoBoDriver && ftcGoBoDriver != null) {
                ftcGoBoDriver.showTelemetry();
            }
        }

        FtcLogger.exit();
    }

    /**
     * Stops the background IMU updater, if enabled.
     */
    public void stop() {
        FtcLogger.enter();
        if (asyncUpdaterEnabled && imuAsyncUpdater != null) {
            imuAsyncUpdater.stop();
            imuAsyncUpdater = null;
        }

        FtcLogger.exit();
    }
}
