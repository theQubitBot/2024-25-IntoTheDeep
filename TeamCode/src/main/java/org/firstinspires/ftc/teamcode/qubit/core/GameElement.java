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
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.GeometricShapeEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.TgeDetectionAlgorithmEnum;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * A class to manage the Team Game Element (TGE).
 */
public class GameElement {
    private static final String TAG = "GameElement";
    public static final int ATTENDANCE_MAX = 30;
    public static final int ATTENDANCE_THRESHOLD = 15;
    public double minAspectRatio, maxAspectRatio;
    public double area;
    public double aspectRatio;
    public Scalar elementColor;
    public Scalar lowerColorThreshold;
    public Scalar upperColorThreshold;
    public Rect minSize, maxSize;
    public final int borderSize;
    public int colorConversionCode;
    public String tag;
    public GeometricShapeEnum geometricShapeEnum;
    public boolean found;
    public Rect boundingRect;
    public RotatedRect rotatedRect;
    private final int[] attendanceRegister;
    private int attendanceIndex;

    public TgeDetectionAlgorithmEnum tgeDetectionAlgorithm;

    /**
     * Constructor. Sets the defaults for the TGE.
     */
    public GameElement() {
        // OpenCV processes about 6 FPS
        // Look back 5 seconds of processed frames
        attendanceRegister = new int[ATTENDANCE_MAX];
        attendanceIndex = 0;
        aspectRatio = 0;
        boundingRect = new Rect();
        rotatedRect = new RotatedRect();
        borderSize = 4;
        colorConversionCode = Imgproc.COLOR_RGB2YCrCb;
        geometricShapeEnum = GeometricShapeEnum.UNKNOWN;
        elementColor = FtcColorUtils.RGB_WHITE;
        tgeDetectionAlgorithm = TgeDetectionAlgorithmEnum.CONTOUR_AND_CHANNEL;
        found = false;
        lowerColorThreshold = new Scalar(0.0, 0.0, 0.0);
        upperColorThreshold = new Scalar(255.0, 255.0, 255.0);
        tag = FtcColorUtils.TAG_UNKNOWN;
        minSize = new Rect(0, 0, 80, 80);
        maxSize = new Rect(0, 0, 400, 400);

        // Team prop merges with spike mark, aspect ratio can be 3!
        minAspectRatio = 0.25;
        maxAspectRatio = 3.5;
    }

    public int attendanceCount() {
        int sum = 0;
        synchronized (attendanceRegister) {
            for (int i = 0; i < attendanceRegister.length; i++) {
                sum += attendanceRegister[i];
            }
        }

        return sum;
    }

    public boolean elementConsistentlyPresent() {
        return attendanceCount() >= ATTENDANCE_THRESHOLD;
    }

    public synchronized boolean elementFound() {
        return found;
    }

    public static GameElement getGE(GameElementTypeEnum gameElementType) {
        GameElement gameElement = new GameElement();
        switch (gameElementType) {
            case BLUE:
                gameElement.elementColor = FtcColorUtils.RGB_BLUE;
                gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
                gameElement.lowerColorThreshold = new Scalar(110.0, 20.0, 50.0);
                gameElement.upperColorThreshold = new Scalar(130.0, 255.0, 255.0);
                gameElement.tag = FtcColorUtils.TAG_BLUE;
                break;
            case BLACK:
                gameElement.elementColor = FtcColorUtils.RGB_BLACK;
                gameElement.lowerColorThreshold = new Scalar(0.0, 63.0, 63.0);
                gameElement.upperColorThreshold = new Scalar(31.0, 191.0, 191.0);
                gameElement.tag = FtcColorUtils.TAG_BLACK;
                break;
            case GRAY:
                gameElement.elementColor = FtcColorUtils.RGB_GRAY;
                gameElement.lowerColorThreshold = new Scalar(63.0, 63.0, 63.0);
                gameElement.upperColorThreshold = new Scalar(191.0, 191.0, 191.0);
                gameElement.tag = FtcColorUtils.TAG_GRAY;
                break;
            case GREEN:
                gameElement.elementColor = FtcColorUtils.RGB_GREEN;
                /* YCbCr
                gameElement.lowerColorThreshold = new Scalar(63.0, 0.0, 0.0);
                gameElement.upperColorThreshold = new Scalar(255.0, 120.0, 120.0);
                */

                //* HSV
                gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
                gameElement.lowerColorThreshold = new Scalar(45.0, 20.0, 50.0);
                gameElement.upperColorThreshold = new Scalar(86.0, 255.0, 255.0);
                //*/
                gameElement.tag = FtcColorUtils.TAG_GREEN;
                break;
            case ORANGE:
                gameElement.elementColor = FtcColorUtils.RGB_ORANGE;
                gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
                gameElement.lowerColorThreshold = new Scalar(11.0, 10.0, 10.0);
                gameElement.upperColorThreshold = new Scalar(24.0, 255.0, 255.0);
                gameElement.tag = FtcColorUtils.TAG_ORANGE;
                break;
            case PINK:
                gameElement.elementColor = FtcColorUtils.RGB_PINK;
                gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
                gameElement.lowerColorThreshold = new Scalar(150.0, 10.0, 10.0);
                gameElement.upperColorThreshold = new Scalar(170.0, 255.0, 255.0);
                gameElement.tag = FtcColorUtils.TAG_PINK;
                break;
            case PURPLE:
                gameElement.elementColor = FtcColorUtils.RGB_PURPLE;
                gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
                gameElement.lowerColorThreshold = new Scalar(140.0, 10.0, 10.0);
                gameElement.upperColorThreshold = new Scalar(160.0, 255.0, 255.0);
                gameElement.tag = FtcColorUtils.TAG_PURPLE;
                break;
            case RED1:
                gameElement.elementColor = FtcColorUtils.RGB_RED;
                gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
                gameElement.lowerColorThreshold = new Scalar(0.0, 20.0, 50.0);
                gameElement.upperColorThreshold = new Scalar(15.0, 255.0, 255.0);
                gameElement.tag = FtcColorUtils.TAG_RED1;
                break;
            case RED2:
                gameElement.elementColor = FtcColorUtils.RGB_RED;
                gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
                gameElement.lowerColorThreshold = new Scalar(165.0, 20.0, 50.0);
                gameElement.upperColorThreshold = new Scalar(180.0, 255.0, 255.0);
                gameElement.tag = FtcColorUtils.TAG_RED2;
                break;
            case YELLOW:
                // Yellow Junctions
                gameElement.elementColor = FtcColorUtils.RGB_YELLOW;
                gameElement.colorConversionCode = Imgproc.COLOR_RGB2HSV;
                gameElement.lowerColorThreshold = new Scalar(20.0, 100.0, 100.0);
                gameElement.upperColorThreshold = new Scalar(40.0, 255.0, 255.0);
                gameElement.tag = FtcColorUtils.TAG_YELLOW;
                break;
            case WHITE:
                gameElement.elementColor = FtcColorUtils.RGB_WHITE;
                gameElement.lowerColorThreshold = new Scalar(191.0, 63.0, 63.0);
                gameElement.upperColorThreshold = new Scalar(255.0, 191.0, 191.0);
                gameElement.tag = FtcColorUtils.TAG_WHITE;
                break;
        }

        return gameElement;
    }

    public synchronized Point getMidPoint() {
        return FtcUtils.getMidpoint(boundingRect);
    }

    public synchronized void invalidate() {
        area = 0;
        boundingRect = new Rect();
        rotatedRect = new RotatedRect();
        found = false;
    }

    private void markAbsent() {
        synchronized (attendanceRegister) {
            attendanceRegister[attendanceIndex] = 0;
            attendanceIndex = (attendanceIndex + 1) % attendanceRegister.length;
        }
    }

    private void markPresent() {
        synchronized (attendanceRegister) {
            attendanceRegister[attendanceIndex] = 1;
            attendanceIndex = (attendanceIndex + 1) % attendanceRegister.length;
        }
    }

    public void takeAttendance() {
        if (elementFound()) {
            markPresent();
        } else {
            markAbsent();
        }
    }

    public synchronized void validate(RotatedRect rRect, double aspectRatio, double area) {
        FtcUtils.copyRect(rRect, rotatedRect);
        FtcUtils.copyRect(rotatedRect.boundingRect(), boundingRect);
        this.area = area;
        this.aspectRatio = aspectRatio;
        found = true;
    }
}
