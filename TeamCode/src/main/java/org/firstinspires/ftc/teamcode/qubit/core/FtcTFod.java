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

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * A class to manage tensor flow detections.
 */
public class FtcTFod {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final float MIN_RESULT_CONFIDENCE = 0.90f;
    private static final String TFOD_MODEL_FILE = "centerStage.tfLite";
    private static final String[] LABELS = {
            "ItemA",
            "ItemB",
            "ItemC",
            "ItemD"
    };

    public TfodProcessor tFod;
    private VisionPortal visionPortal;
    public boolean telemetryEnabled = true;
    private Telemetry telemetry;

    /**
     * Initialize standard Hardware interfaces
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        // Create the TensorFlow processor by using a builder.
        tFod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, FtcOpenCvCam.WEBCAM_1_NAME));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(FtcOpenCvCam.CAMERA_WIDTH, FtcOpenCvCam.CAMERA_HEIGHT));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(BuildConfig.DEBUG);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(tFod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tFod.setMinResultConfidence(MIN_RESULT_CONFIDENCE);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tFod, true);
        FtcLogger.exit();
    }

    /**
     * Pauses object detection processing.
     * Saves CPU cycles.
     */
    public void pause() {
        FtcLogger.enter();
        if (visionPortal != null) {
            visionPortal.stopStreaming();
            visionPortal.setProcessorEnabled(tFod, false);
        }

        FtcLogger.exit();
    }

    /**
     * Resumes object detection processing.
     */
    public void resume() {
        FtcLogger.enter();
        if (visionPortal != null) {
            visionPortal.setProcessorEnabled(tFod, true);
            visionPortal.resumeStreaming();
        }

        FtcLogger.exit();
    }

    /**
     * Stops the object detection processing.
     */
    public void stop() {
        FtcLogger.enter();
        if (visionPortal != null) {
            visionPortal.setProcessorEnabled(tFod, false);
            visionPortal.stopStreaming();
            visionPortal.close();
            visionPortal = null;
        }

        FtcLogger.exit();
    }

    /**
     * Displays TFod telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (telemetryEnabled && tFod != null) {
            List<Recognition> recognitions = tFod.getRecognitions();
            telemetry.addData("# Objects Detected", recognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : recognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addLine();
                telemetry.addData("Image", "%s (%.0f %% confidence)",
                        recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f, %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f",
                        recognition.getWidth(), recognition.getHeight());
            }
        }

        FtcLogger.exit();
    }
}
