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

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DrawRectPipeline extends OpenCvPipeline {
    private static final String TAG = "DrawRectPipeline";
    private static final int DELTA = 1;
    private final Point tl = new Point();
    private final Point br = new Point();
    private int maxHeight, maxWidth;

    public void adjustRect(Gamepad gamepad) {
        synchronized (tl) {
            if (gamepad.left_stick_x > 0) {
                if (tl.x + DELTA < br.x) tl.x += DELTA;
            } else if (gamepad.left_stick_x < 0) {
                if (tl.x - DELTA >= 0) tl.x -= DELTA;
            }

            // gamePad.y is inverted
            if (gamepad.left_stick_y < 0) {
                if (tl.y - DELTA >= 0) tl.y -= DELTA;
            } else if (gamepad.left_stick_y > 0) {
                if (tl.y + DELTA < br.y) tl.y += DELTA;
            }

            if (gamepad.right_stick_x > 0) {
                if (br.x + DELTA < maxWidth) br.x += DELTA;
            } else if (gamepad.right_stick_x < 0) {
                if (br.x - DELTA > tl.x) br.x -= DELTA;
            }

            // gamePad.y is inverted
            if (gamepad.right_stick_y < 0) {
                if (br.y - DELTA > tl.y) br.y -= DELTA;
            } else if (gamepad.right_stick_y > 0) {
                if (br.y + DELTA < maxHeight) br.y += DELTA;
            }

            tl.x = Range.clip(tl.x, 0, maxWidth);
            tl.y = Range.clip(tl.y, 0, maxHeight);
            br.x = Range.clip(br.x, 0, maxWidth);
            br.y = Range.clip(br.y, 0, maxHeight);
        }
    }

    @Override
    public void init(Mat firstFrame) {
        maxWidth = firstFrame.width();
        maxHeight = firstFrame.height();
        br.x = maxWidth;
        br.y = maxHeight;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public Mat processFrame(Mat frame) {
        try {
            Rect rect;
            synchronized (tl) {
                rect = new Rect(tl, br);
            }

            Imgproc.rectangle(frame, rect, FtcColorUtils.RGB_WHITE, 2);
            Imgproc.putText(frame, String.format("Rect x: %d, y: %d, width: %d, height: %d",
                            rect.x, rect.y, rect.width, rect.height),
                    new Point(5, 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, FtcColorUtils.RGB_WHITE, 2);
        } catch (Exception ignored) {
        }

        return frame;
    }
}
