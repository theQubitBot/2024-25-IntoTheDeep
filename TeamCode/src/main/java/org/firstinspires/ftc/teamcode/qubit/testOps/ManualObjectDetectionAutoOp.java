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

package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcOpenCvCam;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.ManualObjectDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous(group = "TestOp")
public class ManualObjectDetectionAutoOp extends LinearOpMode {
    private OpenCvWebcam openCvWebcam;

    @Override
    public void runOpMode() {
        telemetry.addData(">", "Initializing. Please wait...");
        telemetry.update();

        // OpenCV webcam
        openCvWebcam = FtcOpenCvCam.createWebcam(hardwareMap, FtcOpenCvCam.WEBCAM_1_NAME);

        //OpenCV Pipeline
        ManualObjectDetectionPipeline openCvPipeline;
        openCvWebcam.setPipeline(openCvPipeline = new ManualObjectDetectionPipeline());

        // Webcam Streaming
        openCvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvWebcam.startStreaming(
                        FtcOpenCvCam.CAMERA_WIDTH, FtcOpenCvCam.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the webcam could not be opened
                 */
            }
        });

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(openCvWebcam, 0);

        telemetry.addData(">", "Initialization complete. Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (openCvPipeline.error) {
                telemetry.addData("Exception: ", openCvPipeline.lastException);
            } else {
                if (gamepad1.left_stick_x > 0) {
                    openCvPipeline.IncreaseLowerHue();
                } else if (gamepad1.left_stick_x < 0) {
                    openCvPipeline.DecreaseLowerHue();
                }

                if (gamepad1.right_stick_x > 0) {
                    openCvPipeline.IncreaseUpperHue();
                } else if (gamepad1.right_stick_x < 0) {
                    openCvPipeline.DecreaseUpperHue();
                }
            }

            telemetry.addData("Usage", "Use left and right joysticks to adjust Hue constraints");
            telemetry.addData("HSV limits", "[%.0f, %.0f]",
                    openCvPipeline.gameElement.lowerColorThreshold.val[0],
                    openCvPipeline.gameElement.upperColorThreshold.val[0]);
            telemetry.update();
            FtcUtils.sleep(100);
        }
    }
}
