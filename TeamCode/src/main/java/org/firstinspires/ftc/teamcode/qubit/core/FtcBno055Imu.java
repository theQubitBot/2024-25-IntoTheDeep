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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadRunner.util.AxisDirection;
import org.firstinspires.ftc.teamcode.roadRunner.util.BNO055IMUUtil;

/**
 * A class to manage the external Adafruit IMU.
 */
public class FtcBno055Imu extends FtcSubSystem {
    private static final String TAG = "FtcBno055Imu";
    public boolean telemetryEnabled = true;
    Telemetry telemetry = null;
    private BNO055IMU imu = null;
    private boolean imuIsGood = true;
    private static final Object directionLock = new Object();

    // External IMU may not be exactly orthogonal to axes.
    // Store away initial IMU inclinations to offset the actual angles.
    private double initialHeading = 0.0;
    private double initialPitch = 0.0;
    private double initialRoll = 0.0;
    private double heading = 0.0;
    private double pitch = 0.0;
    private double roll = 0.0;

    /* Constructor */
    public FtcBno055Imu() {
    }

    /**
     * A thread safe method to get IMU heading.
     *
     * @return The IMU heading.
     */
    public double getHeading() {
        double currentHeading;
        synchronized (directionLock) {
            currentHeading = heading - initialHeading;
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
            currentPitch = pitch - initialPitch;
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
            currentRoll = roll - initialRoll;
        }

        return currentRoll;
    }

    public boolean imuIsGood(){
        boolean isGood;
        synchronized (directionLock) {
            isGood = imuIsGood;
        }

        if(telemetryEnabled && !isGood){
            telemetry.addData(TAG, "IMU is in a bad state.");
        }

        return isGood;
    }

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry object to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        BNO055IMU.Parameters gyroParameters;
        imu = hardwareMap.get(BNO055IMU.class, "bno055Imu");
        if (imu != null) {
            // Create imu parameters object.
            gyroParameters = new BNO055IMU.Parameters();

            // Use degrees as angle unit.
            gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

            // Express acceleration as m/s^2.
            gyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

            // Disable logging to improve performance.
            gyroParameters.loggingEnabled = FtcUtils.DEBUG;
            gyroParameters.loggingTag = TAG;

            // Gyro must be initialized
            imu.initialize(gyroParameters);
            while (!imu.isGyroCalibrated()) {
                // Wait till imu is fully calibrated.
                FtcUtils.sleep(100);
            }

            remapZAxis(AxisDirection.NEG_X);

            // Wait for calibration and remap to kick in
            FtcUtils.sleep(500);
            read();
            initialHeading = heading;
            initialRoll = roll;
            initialPitch = pitch;
            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not initialized");
        }

        FtcLogger.exit();
    }

    /**
     * Read the imu and store information for later use.
     */
    public void read() {
        FtcLogger.enter();
        if (imu != null) {
            Orientation angles = imu.getAngularOrientation();
            synchronized (directionLock) {
                heading = angles.firstAngle;
                roll = angles.secondAngle;
                pitch = angles.thirdAngle;
                imuIsGood = angles.acquisitionTime != 0;
            }
        }

        FtcLogger.exit();
    }

    public void remapZAxis(AxisDirection axisDirection) {
        FtcLogger.enter();
        if (imu != null) {
            BNO055IMUUtil.remapZAxis(imu, axisDirection);
        }

        FtcLogger.exit();
    }

    /**
     * Display imu telemetry.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (telemetryEnabled) {
            telemetry.addData("Heading", "%.2f", getHeading());
            telemetry.addData("Pitch", "%.2f", getPitch());
            telemetry.addData("Roll", "%.2f", getRoll());
        }

        FtcLogger.exit();
    }
}
