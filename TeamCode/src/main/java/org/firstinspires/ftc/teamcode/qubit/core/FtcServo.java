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

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

/**
 * A cache enabled (performant) Servo implementation.
 */
public class FtcServo implements Servo {
    public static final double MID_POSITION = (Servo.MIN_POSITION + Servo.MAX_POSITION) / 2.0;
    public static final double LARGE_INCREMENT = 0.0005;
    public static final double SMALL_INCREMENT = 0.0001;

    // Invalid position must be far away from zero.
    public static final double INVALID_POSITION = -997;
    private final Servo servo;

    // PERFORMANCE
    // Servo position set takes about 2.8 ms.
    // Use a simple and effective servo position caching mechanism.
    private double currentPosition = INVALID_POSITION;

    public FtcServo(Servo servo) {
        this.servo = servo;
    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        if (!FtcUtils.areEqual(currentPosition, position, FtcUtils.EPSILON4)) {
            currentPosition = position;
            servo.setPosition(position);
        }
    }

    @Override
    public double getPosition() {
        if (FtcUtils.areEqual(currentPosition, INVALID_POSITION, FtcUtils.EPSILON4)) {
            currentPosition = servo.getPosition();
        }

        return currentPosition;
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servo.getVersion();
    }

    /**
     * Moves the servo to end position in a controlled fashion.
     * Execution blocks till servo reaches the end position.
     *
     * @param endPosition End servo position.
     */
    public void controlledMove(double endPosition) {
        double currentPosition = getPosition();
        double range = endPosition - currentPosition;
        int signum = (int) Math.signum(range);
        double step = 0.00025 * signum;

        // if end position is less than current, then range, signum and step are all negative.
        // The < condition will be true till current position decrements to end position.
        // So, using signum handles all scenarios without us having to worry about signs and
        // correct comparisons.
        while (currentPosition * signum < endPosition * signum) {
            currentPosition += step;
            currentPosition = Range.clip(currentPosition, Servo.MIN_POSITION, Servo.MAX_POSITION);
            setPosition(currentPosition);
            FtcUtils.sleep(1);
        }
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        servo.close();
    }
}
