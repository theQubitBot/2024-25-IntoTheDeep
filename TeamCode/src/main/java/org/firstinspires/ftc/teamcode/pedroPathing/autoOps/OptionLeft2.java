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

package org.firstinspires.ftc.teamcode.pedroPathing.autoOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcIntake;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLift;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;

/**
 * A class to implement autonomous objective
 */
@Config
public class OptionLeft2 extends OptionBase {
    Pose scorePose = new Pose(-4, 13.5, RADIAN0);

    // First yellow Sample
    public Pose pickup1ControlPose = new Pose(7.5, 31.2, -RADIAN30);
    public Pose pickup1Pose = new Pose(17, 35, -RADIAN30);

    // Second yellow Sample
    public Pose pickup2Pose = new Pose(13.5, 28, RADIAN45);

    // Third yellow Sample
    public Pose pickup3ControlPose = new Pose(17.5, 16, RADIAN90);
    public Pose pickup3Pose = new Pose(24, 29, RADIAN135);

    // Park
    public Pose parkControlPose = new Pose(28, 40, -RADIAN15);
    public Pose parkPose = new Pose(58, 23, -RADIAN45);

    PathChain scorePreloadPath, parkPath,
            pickup1, pickup2, pickup3, score1, score2, score3;

    public static class Params {
        public boolean executeTrajectories = true, executeRobotActions = false;
        public boolean deliverPreloaded = true,
                deliver1 = false, deliver2 = false, deliver3 = false,
                park = false;
    }

    public static Params PARAMS = new Params();

    public OptionLeft2(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
        super(autoOpMode, robot, follower);
        follower.setStartingPose(startPose);
    }

    public OptionLeft2 init() {
        super.initialize();

        // preloaded sample
        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions)
                        robot.intake.flipDown(false);
                })
                .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME, () -> {
                    if (PARAMS.executeRobotActions)
                        robot.lift.move(FtcLift.POSITION_HIGH_BASKET, FtcLift.POSITION_FLOOR, false);
                })
                .build();

        // first yellow
        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),
                        new Point(pickup1ControlPose),
                        new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.flipDown(false);
                })
                .addTemporalCallback(100, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.spinIn(false);
                })
                .addTemporalCallback(1000, () -> {
                    if (PARAMS.executeRobotActions) robot.lift.resetLiftIfTouchPressed();
                })
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.flipDelivery(false);
                })
                .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.flipHorizontal(false);
                })
                .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME + 100, () -> {
                    if (PARAMS.executeRobotActions)
                        robot.lift.move(FtcLift.POSITION_HIGH_BASKET, FtcLift.POSITION_FLOOR, false);
                })
                .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME + 200, () -> {
                    // In case sample is stuck, evict it.
                    if (PARAMS.executeRobotActions) robot.intake.spinOut();
                })
                .build();

        // second yellow
        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.flipDown(false);
                })
                .addTemporalCallback(10, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.spinIn(false);
                })
                .addTemporalCallback(1200, () -> {
                    if (PARAMS.executeRobotActions) robot.lift.resetLiftIfTouchPressed();
                }).build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.flipDelivery(false);
                })
                .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.flipHorizontal(false);
                })
                .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME + 100, () -> {
                    if (PARAMS.executeRobotActions)
                        robot.lift.move(FtcLift.POSITION_HIGH_BASKET, FtcLift.POSITION_FLOOR, false);
                })
                .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME + 200, () -> {
                    // In case sample is stuck, evict it.
                    if (PARAMS.executeRobotActions) robot.intake.spinOut();
                })
                .build();

        // third yellow
        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),
                        new Point(pickup3ControlPose),
                        new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.flipDown(false);
                })
                .addTemporalCallback(10, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.spinIn(false);
                })
                .addTemporalCallback(1200, () -> {
                    if (PARAMS.executeRobotActions) robot.lift.resetLiftIfTouchPressed();
                })
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.flipDelivery(false);
                })
                .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.flipHorizontal(false);
                })
                .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME + 100, () -> {
                    if (PARAMS.executeRobotActions)
                        robot.lift.move(FtcLift.POSITION_HIGH_BASKET, FtcLift.POSITION_FLOOR, false);
                })
                .addTemporalCallback(FtcIntake.FLIP_TRAVEL_TIME + 200, () -> {
                    // In case sample is stuck, evict it.
                    if (PARAMS.executeRobotActions) robot.intake.spinOut();
                })
                .build();

        // park
        parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.flipHorizontal(false);
                })
                .addTemporalCallback(10, () -> {
                    if (PARAMS.executeRobotActions) robot.intake.spinStop();
                })
                .addTemporalCallback(1200, () -> {
                    if (PARAMS.executeRobotActions) robot.lift.resetLiftIfTouchPressed();
                })
                .build();

        return this;
    }

    /**
     * Executes the autonomous workflow.
     */
    public void execute() {
        FtcLogger.enter();

        // Deliver preloaded sample
        if (!saveAndTest()) return;
        if (PARAMS.executeRobotActions) robot.intake.flipDown(false);
        if (PARAMS.deliverPreloaded) {
            if (PARAMS.executeTrajectories) runFollower(scorePreloadPath, true, 2500);
            if (PARAMS.executeRobotActions) robot.arm.moveBackward(true);
            if (PARAMS.executeRobotActions) robot.arm.moveForward(true);
            if (PARAMS.executeRobotActions)
                robot.lift.move(FtcLift.POSITION_FLOOR, FtcLift.POSITION_FLOOR, false);
        }

        // Deliver first yellow sample
        if (!saveAndTest()) return;
        if (PARAMS.deliver1) {
            if (PARAMS.executeTrajectories) runFollower(pickup1, false, 3000);
            if (PARAMS.executeTrajectories) runFollower(score1, true, 3000);
            if (PARAMS.executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH_BASKET, FtcLift.POSITION_FLOOR, true);
            if (PARAMS.executeRobotActions) robot.arm.moveBackward(true);
            if (PARAMS.executeRobotActions) robot.arm.moveForward(true);
            if (PARAMS.executeRobotActions)
                robot.lift.move(FtcLift.POSITION_FLOOR, FtcLift.POSITION_FLOOR, false);
        }

        // Deliver second yellow sample
        if (!saveAndTest()) return;
        if (PARAMS.deliver2) {
            if (PARAMS.executeTrajectories) runFollower(pickup2, false, 3000);
            if (PARAMS.executeTrajectories) runFollower(score2, true, 3000);
            if (PARAMS.executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH_BASKET, FtcLift.POSITION_FLOOR, true);
            if (PARAMS.executeRobotActions) robot.arm.moveBackward(true);
            if (PARAMS.executeRobotActions) robot.arm.moveForward(true);
            if (PARAMS.executeRobotActions)
                robot.lift.move(FtcLift.POSITION_FLOOR, FtcLift.POSITION_FLOOR, false);
        }

        // Deliver third yellow sample
        if (!saveAndTest()) return;
        if (PARAMS.deliver3) {
            if (PARAMS.executeTrajectories) runFollower(pickup3, false, 3000);
            if (PARAMS.executeTrajectories) runFollower(score3, true, 3000);
            if (PARAMS.executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH_BASKET, FtcLift.POSITION_FLOOR, true);
            if (PARAMS.executeRobotActions) robot.arm.moveBackward(true);
            if (PARAMS.executeRobotActions) robot.arm.moveForward(true);
            if (PARAMS.executeRobotActions)
                robot.lift.move(FtcLift.POSITION_FLOOR, FtcLift.POSITION_FLOOR, false);
        }

        // Park
        if (!saveAndTest()) return;
        if (PARAMS.park) {
            if (PARAMS.executeTrajectories) runFollower(parkPath, true, 3000);
            if (PARAMS.executeRobotActions) robot.flag.raise(false);
            if (PARAMS.executeRobotActions) {
                robot.lift.stop();
            }
        }

        FtcLogger.exit();
    }
}
