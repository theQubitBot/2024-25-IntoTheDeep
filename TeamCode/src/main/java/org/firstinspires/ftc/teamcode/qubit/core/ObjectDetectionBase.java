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
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class ObjectDetectionBase {
    protected final Rect leftRect, centerRect, rightRect;
    protected final ArrayList<Rect> targetMatRects;

    public ObjectDetectionBase() {
        leftRect = new Rect(new Point(0, 0), new Point(0, 0));
        centerRect = new Rect(new Point(50, 230), new Point(370, 370));
        rightRect = new Rect(new Point(380, 230), new Point(640, 420));
        targetMatRects = new ArrayList<>(2);
        targetMatRects.add(centerRect);
        targetMatRects.add(rightRect);
    }

    /**
     * Blots out region of the frame that is outside the target areas.
     *
     * @param frame The input image frame.
     */
    protected void blotFrame(Mat frame) {
        // Save original frame.
        Mat cloneSubMat, frameSubMat;
        Mat clonedMat = frame.clone();

        // Blot out the entire original frame.
        Imgproc.rectangle(frame, FtcOpenCvCam.cameraRect, FtcColorUtils.RGB_WHITE, Imgproc.FILLED);

        // Copy the desired areas from clone to original frame.
        for (Rect rect : targetMatRects) {
            cloneSubMat = clonedMat.submat(rect);
            frameSubMat = frame.submat(rect);
            cloneSubMat.copyTo(frameSubMat);
            cloneSubMat.release();
            frameSubMat.release();
        }

        clonedMat.release();
    }
}
