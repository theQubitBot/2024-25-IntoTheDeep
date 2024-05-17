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

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.OperationMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Locale;

/**
 * A class to manage the automatic aiming Turret.
 */
public class FtcTurret extends FtcSubSystem {
    private static final String TAG = "FtcTurret";
    public static final String HORIZONTAL_SERVO_NAME = "hServo";
    public static final String VERTICAL_SERVO_NAME = "vServo";

    private static final double MAX_LEFT_POSITION = 0.1400;
    private static final double MID_HORIZONTAL_POSITION = 0.5000;
    private static final double MAX_RIGHT_POSITION = 0.8600;

    private static final double MAX_UP_POSITION = 0.5500;
    private static final double MID_VERTICAL_POSITION = 0.5000;
    private static final double MAX_DOWN_POSITION = 0.4400;

    private static final double H_INCREMENT = 0.001;
    private static final double V_INCREMENT = 0.001;

    private static final double HORIZONTAL_MARGIN = 5.0;
    private static final double VERTICAL_MARGIN = 5.0;

    private final boolean turretEnabled = false;
    public boolean telemetryEnabled = true;
    public OperationMode operationMode = OperationMode.AUTOMATIC;
    private Telemetry telemetry = null;
    private FtcAprilTag ftcAprilTag = null;
    public FtcServo hServo = null;
    public FtcServo vServo = null;

    /* Constructor */
    public FtcTurret() {
    }

    /**
     * Initialize standard Hardware interfaces
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        if (turretEnabled) {
            hServo = new FtcServo(hardwareMap.get(Servo.class, HORIZONTAL_SERVO_NAME));
            vServo = new FtcServo(hardwareMap.get(Servo.class, VERTICAL_SERVO_NAME));

            // Set Servo to rotate max amount
            hServo.scaleRange(Servo.MIN_POSITION, Servo.MAX_POSITION);
            vServo.scaleRange(Servo.MIN_POSITION, Servo.MAX_POSITION);

            // Set servo directions
            hServo.setDirection(Servo.Direction.FORWARD);
            vServo.setDirection(Servo.Direction.FORWARD);

            ftcAprilTag = new FtcAprilTag();
            ftcAprilTag.init(hardwareMap, telemetry);

            showTelemetry();
            telemetry.addData(TAG, "initialized");
        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Operates the turret automatically using AprilTag detection.
     */
    public void operate() {
        if (turretEnabled && ftcAprilTag != null) {
            AprilTagDetection detection = ftcAprilTag.getFirstDetection();
            if (detection != null && detection.center != null) {
                double oldServoPosition = hServo.getPosition();
                double newServoPosition = oldServoPosition;
                // Camera center is FtcWebcam.CAMERA_WIDTH / 2.0,
                if (detection.center.x > FtcOpenCvCam.CAMERA_WIDTH / 2.0 + HORIZONTAL_MARGIN) {
                    // AprilTag is to the right of the image
                    newServoPosition += H_INCREMENT;
                } else if (detection.center.x < FtcOpenCvCam.CAMERA_WIDTH / 2.0 - HORIZONTAL_MARGIN) {
                    // AprilTag is to the left of the image
                    newServoPosition -= H_INCREMENT;
                }

                newServoPosition = Range.clip(newServoPosition, MAX_LEFT_POSITION, MAX_RIGHT_POSITION);
                hServo.setPosition(newServoPosition);

                oldServoPosition = vServo.getPosition();
                newServoPosition = oldServoPosition;
                if (detection.center.y > FtcOpenCvCam.CAMERA_HEIGHT / 2.0 + VERTICAL_MARGIN) {
                    // AprilTag is below the center of the image
                    newServoPosition -= V_INCREMENT;
                } else if (detection.center.y < FtcOpenCvCam.CAMERA_HEIGHT / 2.0 - VERTICAL_MARGIN) {
                    // AprilTag is above  the center of the image
                    newServoPosition += V_INCREMENT;
                }

                newServoPosition = Range.clip(newServoPosition, MAX_DOWN_POSITION, MAX_UP_POSITION);
                vServo.setPosition(newServoPosition);
            }
        }
    }

    /**
     * Operates the turret using the gamePads.
     *
     * @param gamePad1 The first gamePad to use.
     * @param gamePad2 The second gamePad to use.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2) {
        if (turretEnabled) {
            double oldServoPosition = hServo.getPosition();
            double newServoPosition = oldServoPosition;
            if (gamePad1.dpad_right) {
                newServoPosition += H_INCREMENT;
            } else if (gamePad1.dpad_left) {
                newServoPosition -= H_INCREMENT;
            }

            newServoPosition = Range.clip(newServoPosition, MAX_LEFT_POSITION, MAX_RIGHT_POSITION);
            hServo.setPosition(newServoPosition);

            oldServoPosition = vServo.getPosition();
            newServoPosition = oldServoPosition;
            if (gamePad1.dpad_up) {
                newServoPosition += V_INCREMENT;
            } else if (gamePad1.dpad_down) {
                newServoPosition -= V_INCREMENT;
            }

            newServoPosition = Range.clip(newServoPosition, MAX_DOWN_POSITION, MAX_UP_POSITION);
            vServo.setPosition(newServoPosition);
        }
    }

    /**
     * Displays turret servo telemetry. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (turretEnabled && telemetryEnabled) {
            telemetry.addData(TAG, String.format(Locale.US, "Horizontal %5.4f, Vertical %5.4f",
                    hServo.getPosition(), vServo.getPosition()));

            if (ftcAprilTag != null) {
                AprilTagDetection detection = ftcAprilTag.getFirstDetection();
                if (detection != null && detection.center != null) {
                    telemetry.addData(TAG, String.format(Locale.US, "Center %6.0f %6.0f (pixels)",
                            detection.center.x, detection.center.y));
                } else {
                    telemetry.addData(TAG, "No AprilTag detected");
                }
            }
        }

        FtcLogger.exit();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
        FtcLogger.enter();
        if (turretEnabled) {
            hServo.getController().pwmEnable();
            vServo.getController().pwmEnable();
            hServo.setPosition(MID_HORIZONTAL_POSITION);
            vServo.setPosition(MID_VERTICAL_POSITION);
        }

        FtcLogger.exit();
    }

    /**
     * Stops the turret.
     */
    public void stop() {
        FtcLogger.enter();
        if (turretEnabled) {
            hServo.getController().pwmDisable();
            vServo.getController().pwmEnable();
        }

        FtcLogger.exit();
    }
}
