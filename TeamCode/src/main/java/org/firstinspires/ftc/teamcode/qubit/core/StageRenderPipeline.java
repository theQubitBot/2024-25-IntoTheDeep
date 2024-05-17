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

import org.firstinspires.ftc.teamcode.qubit.core.enumerations.PipelineStageEnum;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class StageRenderPipeline extends OpenCvPipeline {
    private static final String TAG = "StageRenderPipeline";
    Mat yCbCrMat = new Mat();
    Mat yCrMat = new Mat();
    Mat yCbMat = new Mat();
    Mat thresholdMat = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    final Object contourSyncObject = new Object();

    PipelineStageEnum stageToRenderToViewport = PipelineStageEnum.YCbCr_CHANNEL1;
    final PipelineStageEnum[] stages = PipelineStageEnum.values();

    Point textAnchor = new Point(20, 20);

    public void decrementStageToRenderToViewport() {
        synchronized (stages) {
            stageToRenderToViewport =
                    stages[(stageToRenderToViewport.ordinal() - 1 + stages.length) % stages.length];
        }
    }

    public void incrementStageToRenderToViewport() {
        synchronized (stages) {
            stageToRenderToViewport = stages[(stageToRenderToViewport.ordinal() + 1) % stages.length];
        }
    }

    public int getContourCount() {
        int count;
        synchronized (contourSyncObject) {
            count = contoursList.size();
        }

        return count;
    }

    public PipelineStageEnum getStageToRenderToViewport() {
        PipelineStageEnum stageEnum;
        synchronized (stages) {
            stageEnum = stageToRenderToViewport;
        }

        return stageEnum;
    }

    @Override
    public Mat processFrame(Mat input) {
        synchronized (contourSyncObject) {
            contoursList.clear();
        }

        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */
        Imgproc.cvtColor(input, yCbCrMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCbCrMat, yCrMat, 1);
        Core.extractChannel(yCbCrMat, yCbMat, 2);
        Imgproc.threshold(yCbMat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
        synchronized (contourSyncObject) {
            Imgproc.findContours(thresholdMat, contoursList, new Mat(),
                    Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        }

        input.copyTo(contoursOnFrameMat);
        Imgproc.drawContours(contoursOnFrameMat, contoursList, -1,
                new Scalar(0, 0, 255), 3, 8);

        Imgproc.putText(input, "Ftc Test", textAnchor,
                Imgproc.FONT_HERSHEY_SIMPLEX, 1.5, FtcColorUtils.RGB_GREEN, 2);
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols() / 4.0,
                        input.rows() / 4.0),
                new Point(
                        input.cols() * (3f / 4f),
                        input.rows() * (3f / 4f)),
                FtcColorUtils.RGB_GREEN, 4);


        switch (getStageToRenderToViewport()) {
            case YCbCr_CHANNEL1: {
                return yCrMat;
            }

            case YCbCr_CHANNEL2: {
                return yCbMat;
            }

            case THRESHOLD: {
                return thresholdMat;
            }

            case CONTOURS_OVERLAID_ON_FRAME: {
                return contoursOnFrameMat;
            }

            case RAW_IMAGE:
            default: {
                return input;
            }
        }
    }

    @Override
    public void onViewportTapped() {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */
        incrementStageToRenderToViewport();
    }
}
