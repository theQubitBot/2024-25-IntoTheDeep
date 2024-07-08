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

package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcDriveTrain;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.TeamPropLocationEnum;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

/**
 * A base class to provide common variables and methods.
 */
public class OptionBase {
    public static final double RADIAN90;
    public static final double RADIAN45;
    private final long DRIVE_TIME_TO_BACKDROP = 1000; // milliseconds
    protected LinearOpMode autoOpMode;
    protected FtcBot robot;
    protected SampleMecanumDrive drive;

    protected Pose2d startPose, pose1, pose2, pose3, pose4;
    protected Vector2d v1, v2, v3, v4, v5;
    protected Trajectory t1, t2, t3, t4, t5;

    static {
        RADIAN90 = Math.toRadians(90);
        RADIAN45 = Math.toRadians(45);
    }

    public OptionBase(LinearOpMode autoOpMode, FtcBot robot, SampleMecanumDrive drive) {
        this.autoOpMode = autoOpMode;
        this.robot = robot;
        this.drive = drive;
    }

    /**
     * Initializes variables and state.
     */
    protected void initialize() {
        FtcLogger.enter();

        // Starting position
        startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        FtcLogger.exit();
    }

    protected void startCrawlingToBackProp() {
        robot.driveTrain.setDrivePower(-FtcDriveTrain.FORWARD_SLO_MO_POWER, -FtcDriveTrain.FORWARD_SLO_MO_POWER,
                -FtcDriveTrain.FORWARD_SLO_MO_POWER, -FtcDriveTrain.FORWARD_SLO_MO_POWER);
        FtcUtils.sleep(DRIVE_TIME_TO_BACKDROP);
    }

    protected void stopCrawlingToBackProp() {
        robot.driveTrain.stop();
        ;
    }

    protected double lcrValue(double leftValue, double centerValue, double rightValue) {
        if (robot.config.teamPropLocation == TeamPropLocationEnum.LEFT) return leftValue;
        if (robot.config.teamPropLocation == TeamPropLocationEnum.CENTER) return centerValue;
        return rightValue;
    }
}
