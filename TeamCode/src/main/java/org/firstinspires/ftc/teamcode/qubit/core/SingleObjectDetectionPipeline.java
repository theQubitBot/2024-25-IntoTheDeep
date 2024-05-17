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
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SingleObjectDetectionPipeline extends OpenCvPipeline {
    private static final String TAG = "SodPipeline";

    // Pink, the default color                         Y  Cr     Cb    (Do not change Y)
    public static Scalar scalarLowerYCrCb = new Scalar(0, 160.0, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Volatile because accessed by OpMode without syncObject
    public volatile boolean error = false;
    public volatile Exception debug;

    private double borderLeftX;     //fraction of pixels from the left side of the cam to skip
    private double borderRightX;    //fraction of pixels from the right of the cam to skip
    private double borderTopY;      //fraction of pixels from the top of the cam to skip
    private double borderBottomY;   //fraction of pixels from the bottom of the cam to skip

    private int loopCounter = 0;
    private int pLoopCounter = 0;

    private final Mat yCrCbMat = new Mat();
    private final Mat processedMat = new Mat();

    private final Rect maxRect = new Rect(1, 1, 1, 1);
    private double maxArea = maxRect.area();
    private boolean first = false;

    private final GameElement[] gameElements = new GameElement[]{
            GameElement.getGE(GameElementTypeEnum.PINK)
    };

    public SingleObjectDetectionPipeline(double borderLeftX, double borderRightX,
                                         double borderTopY, double borderBottomY) {
        configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
    }

    public void configureScalarLower(double y, double cr, double cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(double y, double cr, double cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarLower(int y, int cr, int cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(int y, int cr, int cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }

    public void configureBorders(double borderLeftX, double borderRightX,
                                 double borderTopY, double borderBottomY) {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }

    @Override
    public Mat processFrame(Mat input) {
        try {
            int imageWidth = input.width();
            int imageHeight = input.height();

            // Process Image
            Imgproc.cvtColor(input, yCrCbMat, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(yCrCbMat, scalarLowerYCrCb, scalarUpperYCrCb, processedMat);
            // Remove Noise
            Imgproc.morphologyEx(processedMat, processedMat, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processedMat, processedMat, Imgproc.MORPH_CLOSE, new Mat());

            // Find Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processedMat, contours, new Mat(), Imgproc.RETR_LIST,
                    Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw Contours
            Imgproc.drawContours(input, contours, -1, FtcColorUtils.RGB_BLACK);

            // Loop Through Contours
            for (MatOfPoint contour : contours) {
                Point[] contourArray = contour.toArray();

                // Bound Rectangle if Contour is Large Enough
                if (contourArray.length >= 15) {
                    MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                    Rect rect = Imgproc.boundingRect(areaPoints);

                    if (rect.area() > maxArea
                            && rect.x + (rect.width / 2.0) > (borderLeftX * imageWidth)
                            && rect.x + (rect.width / 2.0) < imageWidth - (borderRightX * imageWidth)
                            && rect.y + (rect.height / 2.0) > (borderTopY * imageHeight)
                            && rect.y + (rect.height / 2.0) < imageHeight - (borderBottomY * imageHeight)

                            || loopCounter - pLoopCounter > 6
                            && rect.x + (rect.width / 2.0) > (borderLeftX * imageWidth)
                            && rect.x + (rect.width / 2.0) < imageWidth - (borderRightX * imageWidth)
                            && rect.y + (rect.height / 2.0) > (borderTopY * imageHeight)
                            && rect.y + (rect.height / 2.0) < imageHeight - (borderBottomY * imageHeight)
                    ) {
                        maxArea = rect.area();
                        FtcUtils.copyRect(rect, maxRect);
                        pLoopCounter++;
                        loopCounter = pLoopCounter;
                        first = true;
                    } else if (loopCounter - pLoopCounter > 10) {
                        maxArea = 0;
                        FtcUtils.zeroRect(maxRect);
                    }

                    areaPoints.release();
                }
                contour.release();
            }
            if (contours.isEmpty()) {
                FtcUtils.zeroRect(maxRect);
            }

            // Draw Rectangles If Area Is At Least the desired area size
            if (first && getRectArea() >= 625) {
                Imgproc.rectangle(input, maxRect, FtcColorUtils.RGB_GREEN, 2);
            }
            // Draw Borders
            Imgproc.rectangle(input, new Rect(
                    (int) (borderLeftX * imageWidth),
                    (int) (borderTopY * imageHeight),
                    (int) (imageWidth - (borderRightX * imageWidth) - (borderLeftX * imageWidth)),
                    (int) (imageHeight - (borderBottomY * imageHeight) - (borderTopY * imageHeight))
            ), FtcColorUtils.RGB_PINK, 2);

            // Display Data
            Imgproc.putText(input, "Area: " + getRectArea() + " Midpoint: " +
                            getRectMidpointXY().x + " , " + getRectMidpointXY().y,
                    new Point(5, imageHeight - 5), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6,
                    FtcColorUtils.RGB_WHITE, 2);

            loopCounter++;
        } catch (Exception e) {
            debug = e;
            error = true;
        }
        return input;
    }

    /*
     * Synchronize these operations as the user code could be incorrect otherwise,
     * i.e a property is read while the same rectangle is being processedMat in the pipeline,
     * leading to some values being not synced.
     */

    public double getRectMidpointX() {
        synchronized (maxRect) {
            return maxRect.x + (maxRect.width / 2.0);
        }
    }

    public double getRectMidpointY() {
        synchronized (maxRect) {
            return maxRect.y + (maxRect.height / 2.0);
        }
    }

    public Point getRectMidpointXY() {
        synchronized (maxRect) {
            return new Point(getRectMidpointX(), getRectMidpointY());
        }
    }

    public double getRectArea() {
        synchronized (maxRect) {
            return maxRect.area();
        }
    }
}
