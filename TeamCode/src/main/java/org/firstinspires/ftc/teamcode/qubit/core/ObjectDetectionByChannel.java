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

import android.annotation.SuppressLint;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

public class ObjectDetectionByChannel extends ObjectDetectionBase {

    private final Mat yCrCbMat;
    private final Mat Cr;
    private final Mat Cb;
    private Mat regionCenterCr, regionRightCr;
    private Mat regionCenterCb, regionRightCb;
    private int avgCenter, avgRight;
    private final int avgThreshold = 130;
    private RotatedRect rRect;

    public ObjectDetectionByChannel() {
        yCrCbMat = new Mat();
        Cr = new Mat();
        Cb = new Mat();
        regionCenterCr = new Mat();
        regionCenterCb = new Mat();
        regionRightCr = new Mat();
        regionRightCb = new Mat();
    }

    /**
     * PERFORMANCE
     * Initializes various Mats and sub mats so that we don't need to
     * reinitialize them on every frame..
     *
     * @param firstFrame The very first frame that is processed.
     */
    public void init(Mat firstFrame) {
        Imgproc.cvtColor(firstFrame, yCrCbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCrCbMat, Cb, FtcColorUtils.CB_CHANNEL);
        Core.extractChannel(yCrCbMat, Cr, FtcColorUtils.CR_CHANNEL);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any change to the child affects the parent and vice-versa.
         */
        regionCenterCb = Cb.submat(centerRect);
        regionRightCb = Cb.submat(rightRect);

        regionCenterCr = Cr.submat(centerRect);
        regionRightCr = Cr.submat(rightRect);
    }

    /**
     * Process the current frame, looking for the game element.
     *
     * @param frame         The input frame to process.
     * @param gameElement   The game element to look for.
     * @param annotateFrame When true,draws annotations on the frame. Helps with debugging.
     */
    @SuppressLint("DefaultLocale")
    public void processFrame(Mat frame, GameElement gameElement, boolean annotateFrame) {
        Imgproc.cvtColor(frame, yCrCbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCrCbMat, Cb, FtcColorUtils.CB_CHANNEL);
        Core.extractChannel(yCrCbMat, Cr, FtcColorUtils.CR_CHANNEL);

        // Process Image
        if (gameElement.tag.equals(FtcColorUtils.TAG_BLUE)) {
            avgCenter = (int) Core.mean(regionCenterCb).val[0];
            avgRight = (int) Core.mean(regionRightCb).val[0];
        } else if (gameElement.tag.equals(FtcColorUtils.TAG_RED1) || gameElement.tag.equals(FtcColorUtils.TAG_RED2)) {
            avgCenter = (int) Core.mean(regionCenterCr).val[0];
            avgRight = (int) Core.mean(regionRightCr).val[0];
        } else {
            avgCenter = avgRight = 0;
        }

        gameElement.invalidate();
        if (avgCenter > avgThreshold) {
            rRect = new RotatedRect(FtcUtils.getMidpoint(centerRect), centerRect.size(), 0);
            gameElement.validate(rRect, FtcUtils.getAspectRatio(centerRect), centerRect.area());
            if (FtcUtils.DEBUG || annotateFrame) {
                Imgproc.rectangle(frame, gameElement.boundingRect, FtcColorUtils.RGB_YELLOW, gameElement.borderSize);
                Imgproc.putText(frame, String.format("%s: Avg %d", gameElement.tag, avgCenter),
                        new Point(gameElement.boundingRect.x, gameElement.boundingRect.y - 40),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, gameElement.elementColor, gameElement.borderSize);
            }
        } else if (avgRight > avgThreshold) {
            rRect = new RotatedRect(FtcUtils.getMidpoint(rightRect), rightRect.size(), 0);
            gameElement.validate(rRect, FtcUtils.getAspectRatio(rightRect), rightRect.area());
            if (FtcUtils.DEBUG || annotateFrame) {
                Imgproc.rectangle(frame, gameElement.boundingRect, FtcColorUtils.RGB_YELLOW, gameElement.borderSize);
                Imgproc.putText(frame, String.format("%s: Avg %d", gameElement.tag, avgRight),
                        new Point(gameElement.boundingRect.x, gameElement.boundingRect.y - 40),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, gameElement.elementColor, gameElement.borderSize);
            }
        }
    }
}
