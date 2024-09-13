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
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

/**
 * A class to implement autonomous objective
 */
public class OptionRight extends OptionBase {

    public OptionRight(LinearOpMode autoOpMode, FtcBot robot, SampleMecanumDrive drive) {
        super(autoOpMode, robot, drive);
    }

    public OptionRight init() {
        super.initialize();
        return this;
    }

    /**
     * Executes the autonomous workflow.
     */
    public void execute() {
        FtcLogger.enter();
        pose1 = new Pose2d(
                lcrValue(24, 29.25, 23),
                lcrValue(6.25, 0, -11.5),
                lcrValue(RADIAN45, 0, 0));
        v1 = new Vector2d(pose1.getX(), pose1.getY());

        pose3 = new Pose2d(3, -9, -RADIAN90);
        v3 = new Vector2d(pose3.getX(), pose3.getY());

        v4 = new Vector2d(pose3.getX(), 53);
        v5 = new Vector2d(
                lcrValue(19.5, 25, 31),
                lcrValue(90, 90, 90));

        if (!autoOpMode.opModeIsActive()) return;

        drive.followTrajectory(t1);

        if (!autoOpMode.opModeIsActive()) return;
            t3 = drive.trajectoryBuilder(t1.end(), true)
                    .lineToLinearHeading(pose3).build();
            drive.followTrajectory(t3);


        if (!autoOpMode.opModeIsActive()) return;
        t4 = drive.trajectoryBuilder(t3.end(), true)
                .splineToConstantHeading(v4, RADIAN90).build();
        drive.followTrajectory(t4);

        if (!autoOpMode.opModeIsActive()) return;
        t5 = drive.trajectoryBuilder(t4.end(), true)
                .splineToConstantHeading(v5, RADIAN90).build();
        drive.followTrajectory(t5);

        if (!autoOpMode.opModeIsActive()) return;
        startCrawlingToBackProp();
        stopCrawlingToBackProp();

        FtcLogger.exit();
    }
}
