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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.TeamPropLocationEnum;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * A class to manage the EasyOpenCV and execute the pipeline.
 */
public class FtcOpenCvCam extends FtcSubSystem {
    static final String TAG = "FtcOpenCvCam";
    public static final String WEBCAM_1_NAME = "Webcam 1";
    public static final String WEBCAM_2_NAME = "Webcam 2";

    public static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    public static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    public static final double CAMERA_ASPECT_RATIO = (double) CAMERA_HEIGHT / (double) CAMERA_WIDTH;
    public static final Rect cameraRect = new Rect(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT);
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public OpenCvWebcam openCvWebcam = null;
    private Boolean initializationIsSuccessful;
    public boolean showTelemetry = true;
    public AprilTagDetectionPipeline atdPipeline = null;
    public MultipleObjectDetectionPipeline modPipeline = null;

    /**
     * Initialize standard Hardware interfaces
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initializationIsSuccessful = true;

        // Initialize frontWebcam
        openCvWebcam = createWebcam(hardwareMap, WEBCAM_1_NAME);

        if (openCvWebcam == null) {
            initializationIsSuccessful = false;
            showCameraFailure();
        } else {
            // Initialize OpenCV pipeline
            atdPipeline = new AprilTagDetectionPipeline();
            modPipeline = new MultipleObjectDetectionPipeline(openCvWebcam);
            openCvWebcam.setPipeline(modPipeline);

            // Start frontWebcam streaming
            openCvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    openCvWebcam.startStreaming(
                            CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        }

        if (FtcUtils.DEBUG) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            if (dashboard != null) {
                telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
                dashboard.startCameraStream(openCvWebcam, 0);
            }
        }

        telemetry.addData(TAG,
                "Camera initialization process is complete. Check logs for success/failure.");
        FtcLogger.exit();
    }

    public static OpenCvWebcam createWebcam(HardwareMap hardwareMap, String cameraName) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());
        return OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, cameraName),
                        cameraMonitorViewId);
    }

    /**
     * Get the team prop location as determined by the pipeline.
     *
     * @param allianceColor Given alliance color. Only team props that match alliance color are evaluated.
     * @return The team prop location (left, center or right).
     */
    public TeamPropLocationEnum getTeamPropPosition(AllianceColorEnum allianceColor) {
        TeamPropLocationEnum teamPropLocation = TeamPropLocationEnum.LEFT;
        try {
            if (webcamIsWorking(openCvWebcam) && modPipeline != null) {
                for (GameElement ge : modPipeline.gameElements) {
                    Point midPoint;
                    synchronized (ge) {
                        if (ge.elementFound() && ge.elementConsistentlyPresent()) {
                            if ((allianceColor == AllianceColorEnum.BLUE && ge.tag.equals(FtcColorUtils.TAG_BLUE)) ||
                                    (allianceColor == AllianceColorEnum.RED && (ge.tag.equals(FtcColorUtils.TAG_RED1) || ge.tag.equals(FtcColorUtils.TAG_RED2)))) {
                                midPoint = FtcUtils.getMidpoint(ge.boundingRect);

                                if (midPoint.x < 300) {
                                    teamPropLocation = TeamPropLocationEnum.CENTER;
                                    break;
                                } else if (midPoint.x > 400) {
                                    teamPropLocation = TeamPropLocationEnum.RIGHT;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        } catch (Exception ignored) {
        }

        return teamPropLocation;
    }

    public void showCameraFailure() {
        if (!initializationIsSuccessful) {
            telemetry.addLine();
            telemetry.addData(TAG, "Webcam failed to start. STOP and RE-INITIALIZE.");
            telemetry.addLine();
        }
    }

    public void stop() {
        if (initializationIsSuccessful && webcamIsWorking(openCvWebcam)) {
            openCvWebcam.stopStreaming();
            openCvWebcam.closeCameraDevice();
        }
    }

    private Boolean webcamIsWorking(OpenCvWebcam openCvWebcam) {
        return openCvWebcam != null && openCvWebcam.getFrameCount() > 0 && openCvWebcam.getFps() > 0;
    }
}
