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

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.Assert;

import java.util.HashMap;
import java.util.Locale;

/**
 * A global utility for managing robot execution logs.
 */
public final class FtcLogger {
    private static final String TAG = "FtcLogger";
    private static ElapsedTime runtime = null;

    // PERFORMANCE
    // Set to false for official runs.
    private static final boolean performanceMetricsEnabled = false;
    private static HashMap<String, Double> performanceMetricsMap = null;

    /* Constructor */
    static {
        if (performanceMetricsEnabled) {
            runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            performanceMetricsMap = new HashMap<>(64);
        }
    }

    public FtcLogger() {
    }

    public static void debug(String format, Object... args) {
        if (FtcUtils.DEBUG) {
            RobotLog.dd(TAG, format, args);
        }
    }

    public static void enter() {
        if (performanceMetricsEnabled || FtcUtils.DEBUG) {
            StackTraceElement[] steArray = Thread.currentThread().getStackTrace();
            StackTraceElement ste = steArray[3];
            String className = getClassNameOnly(ste.getClassName());
            String methodName = ste.getMethodName();
            String key = String.format("%s.%s", className, methodName);
            if (performanceMetricsEnabled) {
                performanceMetricsMap.put(key, runtime.milliseconds());
            }

            String message = String.format("%s - enter", key);
            RobotLog.dd(TAG, message);
        }
    }

    public static void error(String tag, String format, Object... args) {
        RobotLog.ee(tag, format, args);
    }

    public static void exit() {
        if (performanceMetricsEnabled || FtcUtils.DEBUG) {
            StackTraceElement[] steArray = Thread.currentThread().getStackTrace();
            StackTraceElement ste = steArray[3];
            String className = getClassNameOnly(ste.getClassName());
            String methodName = ste.getMethodName();
            String key = String.format("%s.%s", className, methodName);
            String message = String.format("%s - exit", key);
            if (performanceMetricsEnabled && performanceMetricsMap.containsKey(key)) {
                double methodExecutionTime = runtime.milliseconds() - performanceMetricsMap.get(key);
                message += String.format(Locale.US, " - %.2f ms", methodExecutionTime);
            }

            RobotLog.dd(TAG, message);
        }
    }

    private static String getClassNameOnly(String fullClassName) {
        Assert.assertNotNull(fullClassName, "getClassNameOnly>fullClassName");
        String classNameOnly;
        String[] dataArray = fullClassName.split("\\.");
        if (dataArray.length > 0) {
            classNameOnly = dataArray[dataArray.length - 1];
        } else {
            classNameOnly = fullClassName;
        }

        return classNameOnly;
    }

    public static void info(String tag, String format, Object... args) {
        if (performanceMetricsEnabled || FtcUtils.DEBUG) {
            RobotLog.ii(tag, format, args);
        }
    }
}
