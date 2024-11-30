/* Copyright (c) 2024 The Qubit Bot. All rights reserved.
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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * A class to manage the bulk read operations.
 */
public class FtcBulkRead extends FtcSubSystem {
    private static final String TAG = "FtcBulkRead";
    private List<LynxModule> allLynxModules = null;
    private LynxModule.BulkCachingMode cachingMode;
    private Telemetry telemetry = null;

    /**
     * Constructor.
     */
    public FtcBulkRead() {
        // Start off with the default OFF mode.
        cachingMode = LynxModule.BulkCachingMode.OFF;
    }

    /**
     * Initialize standard Hardware interfaces
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;

        // Get a list of Control/Expansion Hub modules to manage caching support.
        if (hardwareMap != null) {
            allLynxModules = hardwareMap.getAll(LynxModule.class);
            if (allLynxModules != null && !allLynxModules.isEmpty()) {
                // MANUAL mode is recommended since AUTO suffers
                // from performance issues in certain cases.
                setCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Gets the current bulk caching mode.
     *
     * @return The current bulk caching mode.
     */
    public LynxModule.BulkCachingMode getCachingMode() {
        return cachingMode;
    }

    /**
     * Sets the bulk caching mode. This operation is a no-op if
     * the new caching mode is same as the current caching mode.
     */
    public void setCachingMode(LynxModule.BulkCachingMode newCachingMode) {
        FtcLogger.enter();
        if (allLynxModules != null) {
            for (LynxModule lynxModule : allLynxModules) {
                lynxModule.setBulkCachingMode(newCachingMode);
            }

            cachingMode = newCachingMode;
        }

        FtcLogger.exit();
    }

    /**
     * Display bulk caching mode.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        telemetry.addData(TAG, "BulkCachingMode %s", cachingMode);
        FtcLogger.exit();
    }

    /**
     * Clears the bulk cache.
     */
    public void clearBulkCache() {
        FtcLogger.enter();
        if (cachingMode == LynxModule.BulkCachingMode.MANUAL) {
            for (LynxModule lynxModule : allLynxModules) {
                lynxModule.clearBulkCache();
            }
        }

        FtcLogger.exit();
    }
}
