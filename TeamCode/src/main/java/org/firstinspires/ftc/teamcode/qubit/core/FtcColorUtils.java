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

import org.opencv.core.Scalar;

/**
 * Global utility functions
 */
public final class FtcColorUtils {
    private static final String TAG = "FtcColorUtils";
    public static final double Y_MIN = 0.0;
    public static final double Y_MAX = 255.0;
    public static final double CR_MIN = 0.0;
    public static final double CR_MAX = 255.0;
    public static final double CB_MIN = 0.0;
    public static final double CB_MAX = 255.0;
    public static final int Y_CHANNEL = 0, CR_CHANNEL = 1, CB_CHANNEL = 2;

    public static final Scalar RGB_BLACK = new Scalar(0, 0, 0);
    public static final Scalar RGB_BLUE = new Scalar(0, 0, 255);
    public static final Scalar RGB_GRAY = new Scalar(127, 127, 127);
    public static final Scalar RGB_GREEN = new Scalar(0, 255, 0);
    public static final Scalar RGB_ORANGE = new Scalar(255, 165, 0);
    public static final Scalar RGB_PINK = new Scalar(196, 23, 112);
    public static final Scalar RGB_PURPLE = new Scalar(255, 0, 255);
    public static final Scalar RGB_RED = new Scalar(255, 0, 0);
    public static final Scalar RGB_TEAL = new Scalar(3, 148, 252);
    public static final Scalar RGB_YELLOW = new Scalar(255, 255, 0);
    public static final Scalar RGB_WHITE = new Scalar(255, 255, 255);

    public static final String TAG_BLACK = "Black";
    public static final String TAG_BLUE = "Blue";
    public static final String TAG_GRAY = "Gray";
    public static final String TAG_GREEN = "Green";
    public static final String TAG_ORANGE = "Orange";
    public static final String TAG_PINK = "Pink";
    public static final String TAG_PURPLE = "Purple";
    public static final String TAG_RED1 = "Red1";
    public static final String TAG_RED2 = "Red2";
    public static final String TAG_TEAL = "Teal";
    public static final String TAG_UNKNOWN = "Unknown";
    public static final String TAG_YELLOW = "Yellow";
    public static final String TAG_WHITE = "White";
}
