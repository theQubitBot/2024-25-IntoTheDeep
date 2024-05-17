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

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagDetectionPipeline extends OpenCvPipeline {
    private static final String TAG = "AtdPipeline";

    // Volatile because accessed by OpMode without syncObject
    public volatile boolean error = false;
    public volatile Exception lastException = null;

    public static final int TAG_ID_ONE = 1;
    public static final int TAG_ID_TWO = 2;
    public static final int TAG_ID_THREE = 3;
    private long nativeAprilTagPtr;
    private final Mat greyScaleMat = new Mat();
    private ArrayList<AprilTagDetection> latestDetections = null;

    private final Object detectionsLock = new Object();

    private final double fx = 578.272;
    private final double fy = 578.272;
    private final double cx = 402.145;
    private final double cy = 221.506;

    // UNITS ARE METERS
    private final double tagSize = 0.166;

    public AprilTagDetectionPipeline() {
        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(
                AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    protected void finalize() {
        // Might be null if createAprilTagDetector() threw an exception
        if (nativeAprilTagPtr != 0) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeAprilTagPtr);
            nativeAprilTagPtr = 0;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections() {
        ArrayList<AprilTagDetection> tempDetections;
        synchronized (detectionsLock) {
            tempDetections = latestDetections;
        }

        return tempDetections;
    }

    @Override
    public Mat processFrame(Mat input) {
        try {
            error = false;
            lastException = null;

            // Convert to greyscale
            Imgproc.cvtColor(input, greyScaleMat, Imgproc.COLOR_RGBA2GRAY);

            // Run AprilTag
            ArrayList<AprilTagDetection> tempDetections = AprilTagDetectorJNI
                    .runAprilTagDetectorSimple(nativeAprilTagPtr, greyScaleMat, tagSize, fx, fy, cx, cy);
            synchronized (detectionsLock) {
                latestDetections = tempDetections;
            }
        } catch (Exception exception) {
            lastException = exception;
            error = true;
        }

        return input;
    }
}
