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

package org.firstinspires.ftc.teamcode.qubit.core.enumerations;

/**
 * Geometric shapes are:
 * TARGET_UNKNOWN = 0 sides
 * TRIANGLE = 3 sides
 * SQUARE = 4 sides, aspect ratio = 1
 * RECTANGLE = 4 sides, aspect ratio != 1
 * PENTAGON = 5 sides
 * HEXAGON = 6 sides
 * CIRCLE >= 7 sides
 */
public enum GeometricShapeEnum {
    UNKNOWN(0, "Unknown"),
    TRIANGLE(3, "Triangle"),
    SQUARE(4, "Square"),
    RECTANGLE(4, "Rectangle"),
    PENTAGON(5, "Pentagon"),
    HEXAGON(6, "Hexagon"),
    CIRCLE(7, "Circle");

    public final int sides;
    public final String name;

    GeometricShapeEnum(int sides, String name) {
        this.sides = sides;
        this.name = name;
    }

    public boolean match(long sides) {
        if (this.sides == 6) return sides >= this.sides;
        return sides == this.sides;
    }
}
