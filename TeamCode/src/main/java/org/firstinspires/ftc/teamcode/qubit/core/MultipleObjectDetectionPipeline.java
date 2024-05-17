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

import org.firstinspires.ftc.teamcode.qubit.core.enumerations.GameElementTypeEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.TgeDetectionAlgorithmEnum;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class MultipleObjectDetectionPipeline extends OpenCvPipeline {
    private static final String TAG = "ModPipeline";
    private OpenCvWebcam openCvWebcam = null;
    private boolean annotateFrame = false;

    // Volatile because accessed by OpMode without syncObject
    public volatile boolean error = false;
    public volatile Exception lastException = null;
    private final ObjectDetectionByChannel odbChn;
    private final ObjectDetectionByContour odbCon;

    public final GameElement[] gameElements = new GameElement[]{
            GameElement.getGE(GameElementTypeEnum.BLUE),
            GameElement.getGE(GameElementTypeEnum.RED1),
            GameElement.getGE(GameElementTypeEnum.RED2)
    };

    public MultipleObjectDetectionPipeline(OpenCvWebcam openCvWebcam) {
        this.openCvWebcam = openCvWebcam;
        odbChn = new ObjectDetectionByChannel();
        odbCon = new ObjectDetectionByContour();
    }

    public void disableAnnotations() {
        annotateFrame = false;
        if (openCvWebcam != null) {
            openCvWebcam.pauseViewport();
        }
    }

    public void enableAnnotations() {
        annotateFrame = true;
        if (openCvWebcam != null) {
            openCvWebcam.resumeViewport();
        }
    }

    @Override
    public void init(Mat firstFrame) {
        odbChn.init(firstFrame);
        odbCon.init(firstFrame);
    }

    @Override
    public Mat processFrame(Mat frame) {
        error = false;
        lastException = null;
        try {
            odbChn.blotFrame(frame);
            for (GameElement gameElement : gameElements) {
                if (gameElement.tgeDetectionAlgorithm == TgeDetectionAlgorithmEnum.CONTOUR_AND_CHANNEL) {
                    boolean foundChn, foundCon;
                    odbChn.processFrame(frame, gameElement, annotateFrame);
                    foundChn = gameElement.elementFound();
                    odbCon.processFrame(frame, gameElement, annotateFrame);
                    foundCon = gameElement.elementFound();
                    if (foundChn == foundCon) {
                        gameElement.takeAttendance();
                    }
                } else if (gameElement.tgeDetectionAlgorithm == TgeDetectionAlgorithmEnum.CHANNEL) {
                    odbChn.processFrame(frame, gameElement, annotateFrame);
                    gameElement.takeAttendance();
                } else if (gameElement.tgeDetectionAlgorithm == TgeDetectionAlgorithmEnum.CONTOUR) {
                    odbCon.processFrame(frame, gameElement, annotateFrame);
                    gameElement.takeAttendance();
                }
            }
        } catch (Exception exception) {
            lastException = exception;
            error = true;
        }

        return frame;
    }

    @Override
    public void onViewportTapped() {
        /*
         * The viewport (if one was specified in the constructor) can also be dynamically "paused"
         * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
         * when you need your vision pipeline running, but do not require a live preview on the
         * robot controller screen. For instance, this could be useful if you wish to see the live
         * camera preview as you are initializing your robot, but you no longer require the live
         * preview after you have finished your initialization process; pausing the viewport does
         * not stop running your pipeline.
         *
         */
    }

    public void draw(Mat input, RotatedRect rotatedRect, Scalar color, int thickness) {
        Point[] points = new Point[4];
        try {
            rotatedRect.points(points);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, points[i], points[(i + 1) % 4], color, thickness);
            }
        } catch (Exception ignored) {
        }
    }
}
