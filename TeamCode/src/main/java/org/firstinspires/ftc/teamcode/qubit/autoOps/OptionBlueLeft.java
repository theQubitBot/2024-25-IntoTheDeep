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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.TeamPropLocationEnum;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

/**
 * A class to implement autonomous objective
 */
public class OptionBlueLeft extends OptionBase {

    public OptionBlueLeft(LinearOpMode autoOpMode, FtcBot robot, SampleMecanumDrive drive) {
        super(autoOpMode, robot, drive);
    }

    public OptionBlueLeft init() {
        super.initialize();
        return this;
    }

    /**
     * Executes the autonomous workflow.
     */
    public void execute() {
        FtcLogger.enter();

        if (!autoOpMode.opModeIsActive()) return;
        pose1 = new Pose2d(
                lcrValue(27, 32.25, 27),
                lcrValue(18, 6.5, -5),
                -RADIAN45);
        pose2 = new Pose2d(
                lcrValue(19, 26.25, 32.25),
                lcrValue(39, 39, 39),
                -RADIAN90);

        // Move along the back prop to the center
        pose3 = new Pose2d(
                lcrValue(50, 50, 50),
                lcrValue(34, 34, 34),
                -RADIAN90);
        v3 = new Vector2d(pose3.getX(), pose3.getY());

        // Move close to the perimeter for parking
        pose4 = new Pose2d(
                lcrValue(50, 50, 50),
                lcrValue(51, 51, 51),
                -RADIAN90);
        v4 = new Vector2d(pose4.getX(), pose4.getY());

        if (!autoOpMode.opModeIsActive()) return;
        if (robot.config.teamPropLocation == TeamPropLocationEnum.RIGHT) {
            t1 = drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(pose1, pose1.getHeading()).build();
        } else {
            t1 = drive.trajectoryBuilder(startPose).lineToLinearHeading(pose1).build();
        }

        drive.followTrajectory(t1);

        if (!autoOpMode.opModeIsActive()) return;
        t2 = drive.trajectoryBuilder(t1.end(), true).lineToLinearHeading(pose2).build();
        drive.followTrajectory(t2);

        if (!autoOpMode.opModeIsActive()) return;
        startCrawlingToBackProp();
        stopCrawlingToBackProp();

        if (!autoOpMode.opModeIsActive()) return;
        t3 = drive.trajectoryBuilder(t2.end()).splineToConstantHeading(v3, RADIAN90)
                .splineToConstantHeading(v4, RADIAN90).build();
        drive.followTrajectory(t3);

        FtcLogger.exit();
    }
}
