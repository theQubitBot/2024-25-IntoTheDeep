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

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.lang.reflect.Modifier;
import java.util.concurrent.TimeUnit;

/**
 * Global utility functions
 */
public final class FtcUtils {
    public static final String TAG = ">";
    public static final boolean DEBUG = true;
    public static final double EPSILON0 = 0;
    public static final double EPSILON1 = 1e-1;
    public static final double EPSILON2 = 1e-2;
    public static final double EPSILON3 = 1e-3;
    public static final double EPSILON4 = 1e-4;
    public static final int AUTO_OP_DURATION = 30;
    public static final int AUTO_2_TELE_OP_TRANSITION_TIME = 8;
    public static final int TELE_OP_DURATION = 120;
    public static final int END_GAME_DURATION = 30;
    public static final int BUZZER_DURATION = 8;
    public static final int CYCLE_MS = 50;

    /* Constructor */
    public FtcUtils() {
    }

    public static Scalar add(Scalar a, Scalar b) {
        return new Scalar(a.val[0] + b.val[0],
                a.val[1] + b.val[1],
                a.val[2] + b.val[2],
                a.val[3] + b.val[3]);
    }

    public static Scalar average(Scalar a, Scalar b) {
        return new Scalar((a.val[0] + b.val[0]) / 2.0,
                (a.val[1] + b.val[1]) / 2.0,
                (a.val[2] + b.val[2]) / 2.0,
                (a.val[3] + b.val[3]) / 2.0);
    }

    /**
     * Determines if a and b are very close to each other.
     *
     * @param a The first number.
     * @param b The second number.
     * @return True, if a and b are less than EPSILON apart.
     */
    public static boolean areEqual(double a, double b, double epsilon) {
        return Math.abs(a - b) <= epsilon;
    }

    /**
     * Determines if a and b are very close to each other.
     *
     * @param a The first number.
     * @param b The second number.
     * @return True, if a and b are less than EPSILON apart.
     */
    public static boolean areEqual(float a, float b, double epsilon) {
        return Math.abs(a - b) <= epsilon;
    }

    public static boolean areEqual(int a, int b, int epsilon) {
        return Math.abs(a - b) <= epsilon;
    }

    /**
     * Copies source Rect into destination Rect
     *
     * @param source      The source rectangle.
     * @param destination The destination rectangle.
     */
    public static void copyRect(Rect source, Rect destination) {
        synchronized (destination) {
            destination.x = source.x;
            destination.y = source.y;
            destination.height = source.height;
            destination.width = source.width;
        }
    }

    /**
     * Copies source Rect into destination Rect
     *
     * @param source      The source rectangle.
     * @param destination The destination rectangle.
     */
    public static void copyRect(RotatedRect source, RotatedRect destination) {
        synchronized (destination) {
            destination.center.x = source.center.x;
            destination.center.y = source.center.y;
            destination.size.height = source.size.height;
            destination.size.width = source.size.width;
            destination.angle = source.angle;
        }
    }

    public static boolean lastNSeconds(ElapsedTime runtime, int nSeconds) {
        return runtime.seconds() >= (TELE_OP_DURATION - nSeconds);
    }

    public static boolean endGame(ElapsedTime runtime) {
        return runtime.seconds() >= (TELE_OP_DURATION - END_GAME_DURATION);
    }

    public static boolean hangInitiated(Gamepad gamePad1, Gamepad gamePad2, ElapsedTime runtime) {
        return
                // Either option button is pressed
                ((gamePad1.options || gamePad2.options) &&
                        ( // Debug mode
                                DEBUG ||
                                        // End game
                                        endGame(runtime) ||
                                        // both drivers initiate hang
                                        (gamePad1.options && gamePad2.options)));
    }

    public static boolean gameOver(ElapsedTime runtime) {
        // Include buzzer duration in game play time.
        return runtime.seconds() >= (TELE_OP_DURATION + BUZZER_DURATION);
    }

    /**
     * Zeros out the rect values.
     *
     * @param rect The rectangle to zero out.
     */
    public static void zeroRect(Rect rect) {
        synchronized (rect) {
            rect.x = 0;
            rect.y = 0;
            rect.height = 0;
            rect.width = 0;
        }
    }

    public static void zeroRect(RotatedRect rRect) {
        synchronized (rRect) {
            rRect.center.x = 0;
            rRect.center.y = 0;
            rRect.size.height = 0;
            rRect.size.width = 0;
            rRect.angle = 0;
        }
    }

    /**
     * Determines if a given value lies in a given range.
     *
     * @param value    The value to evaluate
     * @param minValue The minimum (inclusive) value
     * @param maxValue The maximum (inclusive) value
     * @return True if the given value is in the range [minValue, maxValue], false otherwise
     */
    public static boolean inRange(double value, double minValue, double maxValue) {
        return value >= minValue && value <= maxValue;
    }

    /**
     * Determines if a given value lies outside a given range.
     *
     * @param value    The value to evaluate
     * @param minValue The minimum value
     * @param maxValue The maximum value
     * @return True if the given value is outside the range [minValue, maxValue], false otherwise
     */
    public static boolean outsideRange(double value, double minValue, double maxValue) {
        return value < minValue || value > maxValue;
    }

    /**
     * Determines if a given values lies in a given array of values.
     *
     * @param array The value to evaluate.
     * @param value The array of values to use.
     * @return True if the given values is in the array of value, false otherwise.
     */
    public static boolean contains(int[] array, int value) {
        Assert.assertNotNull(array, "contains>array");
        for (int i : array) {
            if (i == value) return true;
        }

        return false;
    }

    public static double getMidpointX(Rect rect) {
        synchronized (rect) {
            return rect.x + (rect.width / 2.0);
        }
    }

    public static double getMidpointY(Rect rect) {
        synchronized (rect) {
            return rect.y + (rect.height / 2.0);
        }
    }

    public static Point getMidpoint(Rect rect) {
        return new Point(getMidpointX(rect), getMidpointY(rect));
    }

    public static double getAspectRatio(Rect rect) {
        synchronized (rect) {
            if (rect.height == 0)
                return Double.MAX_VALUE;
            else
                return (double) rect.width / (double) rect.height;
        }
    }

    /**
     * Returns ratio of width to height of the input rectangle.
     *
     * @param rect The input rectangle
     * @return The aspect ratio of the rectangle.
     */
    public static double getAspectRatio(RotatedRect rect) {
        synchronized (rect) {
            if (areEqual(rect.size.height, 0, EPSILON4))
                return Double.MAX_VALUE;
            else
                return rect.size.width / rect.size.height;
        }
    }

    public static int sign(double number) {
        return number >= 0 ? 1 : -1;
    }

    /**
     * Sleep until given deadline expires.
     *
     * @param deadline The remaining duration to sleep until.
     */
    public static void sleep(Deadline deadline) {
        if (deadline != null && !deadline.hasExpired()) {
            sleep(deadline.timeRemaining(TimeUnit.MILLISECONDS));
        }
    }

    /**
     * Sleep for given milli seconds.
     *
     * @param milliseconds The time to sleep in milliseconds.
     */
    public static void sleep(long milliseconds) {
        try {
            if (milliseconds >= 0) {
                Thread.sleep(milliseconds);
            }
        } catch (InterruptedException ignored) {
        }
    }

    /**
     * Serializes an object into its JSon string representation.
     *
     * @param o The object to serialize.
     * @return The JSon string representation of the input object.
     */
    public static String serialize(Object o) {
        GsonBuilder gsonBuilder = new GsonBuilder();

        // Allow serialization of public fields only
        gsonBuilder.excludeFieldsWithModifiers(Modifier.ABSTRACT, Modifier.FINAL, Modifier.INTERFACE,
                Modifier.PRIVATE, Modifier.PROTECTED, Modifier.STATIC,
                Modifier.TRANSIENT, Modifier.VOLATILE);

        // Creates a Gson instance based on the current configuration
        Gson gson = gsonBuilder.create();
        return gson.toJson(o);
    }

    /**
     * Deserializes the given JSon string into an object of given type.
     *
     * @param data    The JSon string representation of the object.
     * @param classOf The type of the object to deserialize to.
     * @param <T>     The type of object.
     * @return The deserialized object.
     */
    public static <T> T deserialize(String data, Class<T> classOf) {
        return SimpleGson.getInstance().fromJson(data, classOf);
    }
}
