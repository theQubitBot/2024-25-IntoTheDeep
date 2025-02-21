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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLift;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

/**
 * A class to implement autonomous objective
 */
@Config
public class OptionRight extends OptionBase {
    // Preloaded Specimen
    Pose specimen1FinalPose = new Pose(0, 28, RADIAN0);

    // Push the first floor sample
    Pose sample1PickControl1 = new Pose(15.8, 2, RADIAN0);
    Pose sample1PickFinal = new Pose(30.5, 25.5, RADIAN45);
    Pose sample1DropFinal = new Pose(30.5, 16, -RADIAN45);


    // Push the second floor sample
    Pose sample2PickControl1 = new Pose(30.5, 20, RADIAN0);
    Pose sample2PickFinal = new Pose(37.8, 25.8, RADIAN45);
    Pose sample2DropFinal = new Pose(30.5, 16, -RADIAN45);

    // Push the third floor sample
    Pose sample3PickControl1 = new Pose(36, 26.6, RADIAN45);
    Pose sample3PickFinal = new Pose(46.4, 26.6, RADIAN45);
    Pose sample3DropFinal = new Pose(33, 1, RADIAN0);

    // Deliver second Specimen
    Pose specimen2DeliverControl1 = new Pose(33, 6, RADIAN0);
    Pose specimen2DeliverControl2 = new Pose(-1, 6, RADIAN90);
    Pose specimen2DeliverFinal = new Pose(-1, 25.5, RADIAN180);

    // Deliver third Specimen
    Pose specimen3PickFinal = new Pose(33, -1, RADIAN180);
    Pose specimen3DeliverControl1 = new Pose(33, 6, RADIAN90);
    Pose specimen3DeliverControl2 = new Pose(-2, 6, RADIAN90);
    Pose specimen3DeliverFinal = new Pose(-2, 28.5, RADIAN0);

    // Deliver fourth Specimen
    Pose specimen4PickFinal = new Pose(33, 0.5, RADIAN0);
    Pose specimen4DeliverControl1 = new Pose(33, 6, RADIAN90);
    Pose specimen4DeliverControl2 = new Pose(-3, 6, RADIAN90);
    Pose specimen4DeliverFinal = new Pose(-3, 28.5, RADIAN180);

    // Deliver fifth Specimen
    Pose specimen5PickFinal = new Pose(33, 0.5, RADIAN180);
    Pose specimen5DeliverControl1 = new Pose(33, 6, RADIAN90);
    Pose specimen5DeliverControl2 = new Pose(-4, 6, RADIAN90);
    Pose specimen5DeliverFinal = new Pose(-4, 28.5, RADIAN0);

    // Park
    public Pose parkFinalPose = new Pose(35, 4, RADIAN0);
    PathChain specimen1DeliveryPath,
            sample1PickPath, sample1DropPath,
            sample2PickPath, sample2DropPath,
            sample3PickPath, sample3DropPath,
            specimen2DeliveryPath,
            specimen3PickPath, specimen3DeliveryPath,
            specimen4PickPath, specimen4DeliveryPath,
            specimen5PickPath, specimen5DeliveryPath,
            parkPath;

    public static class Params {
        public boolean executeTrajectories = true, executeRobotActions = false;
        public boolean deliverSpecimen1 = true,
                pushSample1 = true, pushSample2 = true, pushSample3 = true,
                deliverSpecimen2 = true, deliverSpecimen3 = true, deliverSpecimen4 = false, deliverSpecimen5 = false,
                park = true;
    }

    public static Params PARAMS = new Params();

    public OptionRight(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
        super(autoOpMode, robot, follower);
        follower.setStartingPose(startPose);
    }

    public OptionRight init() {
        super.initialize();

        // first specimen
        specimen1DeliveryPath = follower.pathBuilder()
                .addBezierLine(new Point(startPose), new Point(specimen1FinalPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // pick sample 1
        sample1PickPath = follower.pathBuilder()
                .addBezierCurve(new Point(specimen1FinalPose), new Point(sample1PickControl1), new Point(sample1PickFinal))
                .setLinearHeadingInterpolation(specimen1FinalPose.getHeading(), sample1PickFinal.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) releaseLeftSpecimen.run();
                })
                .addTemporalCallback(10, () -> {
                    if (PARAMS.executeRobotActions) lift2Low.run();
                })
                .addTemporalCallback(750, () -> {
                    if (PARAMS.executeRobotActions) intakeFlipDown.run();
                })
                .addTemporalCallback(800, () -> {
                    if (PARAMS.executeRobotActions) intakeSpinIn.run();
                })
                .build();

        // drop sample 1
        sample1DropPath = follower.pathBuilder()
                .addBezierLine(new Point(sample1PickFinal), new Point(sample1DropFinal))
                .setLinearHeadingInterpolation(sample1PickFinal.getHeading(), sample1DropFinal.getHeading())
                .addTemporalCallback(400, () -> {
                    if (PARAMS.executeRobotActions) intakeSpinOut.run();
                })
                .build();

        // pick sample 2
        sample2PickPath = follower.pathBuilder()
                .addBezierCurve(new Point(sample1DropFinal), new Point(sample2PickControl1),
                        new Point(sample2PickFinal))
                .setLinearHeadingInterpolation(sample1DropFinal.getHeading(), sample2PickFinal.getHeading())
                .addTemporalCallback(500, () -> {
                    if (PARAMS.executeRobotActions) intakeSpinIn.run();
                })
                .build();

        // drop sample 2
        sample2DropPath = follower.pathBuilder()
                .addBezierLine(new Point(sample2PickFinal), new Point(sample2DropFinal))
                .setLinearHeadingInterpolation(sample2PickFinal.getHeading(), sample2DropFinal.getHeading())
                .addTemporalCallback(400, () -> {
                    if (PARAMS.executeRobotActions) intakeSpinOut.run();
                })
                .build();

        // pick sample 3
        sample3PickPath = follower.pathBuilder()
                .addBezierCurve(new Point(sample2DropFinal), new Point(sample3PickControl1),
                        new Point(sample3PickFinal))
                .setLinearHeadingInterpolation(sample2DropFinal.getHeading(), sample3PickFinal.getHeading())
                .addTemporalCallback(500, () -> {
                    if (PARAMS.executeRobotActions) intakeSpinIn.run();
                })
                .build();

        // drop sample 3
        sample3DropPath = follower.pathBuilder()
                .addBezierLine(new Point(sample3PickFinal), new Point(sample3DropFinal))
                .setLinearHeadingInterpolation(sample3PickFinal.getHeading(), sample3DropFinal.getHeading())
                .addTemporalCallback(600, () -> {
                    if (PARAMS.executeRobotActions) intakeSpinOut.run();
                })
                .build();

        // deliver specimen 2
        specimen2DeliveryPath = follower.pathBuilder()
                .addBezierCurve(new Point(sample3DropFinal),
                        new Point(specimen2DeliverControl1), new Point(specimen2DeliverControl2),
                        new Point(specimen2DeliverFinal))
                .setLinearHeadingInterpolation(sample3DropFinal.getHeading(), specimen2DeliverFinal.getHeading())
                .addTemporalCallback(400, () -> {
                    if (PARAMS.executeRobotActions) {
                        intakeSpinStop.run();
                        intakeFlipHorizontal.run();
                    }
                })
                .build();

        // pick specimen 3
        specimen3PickPath = follower.pathBuilder()
                .addBezierLine(new Point(specimen2DeliverFinal), new Point(specimen3PickFinal))
                .setLinearHeadingInterpolation(specimen2DeliverFinal.getHeading(), specimen3PickFinal.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) releaseRightSpecimen.run();
                })
                .addTemporalCallback(10, () -> {
                    if (PARAMS.executeRobotActions) lift2Low.run();
                })
                .build();

        // deliver specimen 3
        specimen3DeliveryPath = follower.pathBuilder()
                .addBezierCurve(new Point(specimen3PickFinal),
                        new Point(specimen3DeliverControl1), new Point(specimen3DeliverControl2),
                        new Point(specimen3DeliverFinal))
                .setLinearHeadingInterpolation(specimen3PickFinal.getHeading(), specimen3DeliverFinal.getHeading())
                .build();

        // pick specimen 4
        specimen4PickPath = follower.pathBuilder()
                .addBezierLine(new Point(specimen3DeliverFinal), new Point(specimen4PickFinal))
                .setLinearHeadingInterpolation(specimen3DeliverFinal.getHeading(), specimen4PickFinal.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) releaseLeftSpecimen.run();
                })
                .addTemporalCallback(10, () -> {
                    if (PARAMS.executeRobotActions) lift2Low.run();
                })
                .build();

        // deliver specimen 4
        specimen4DeliveryPath = follower.pathBuilder()
                .addBezierCurve(new Point(specimen4PickFinal),
                        new Point(specimen4DeliverControl1), new Point(specimen4DeliverControl2), new Point(specimen4DeliverFinal))
                .setLinearHeadingInterpolation(specimen4PickFinal.getHeading(), specimen4DeliverFinal.getHeading())
                .build();

        // pick specimen 5
        specimen5PickPath = follower.pathBuilder()
                .addBezierLine(new Point(specimen4DeliverFinal), new Point(specimen5PickFinal))
                .setLinearHeadingInterpolation(specimen4DeliverFinal.getHeading(), specimen5PickFinal.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) releaseRightSpecimen.run();
                })
                .addTemporalCallback(10, () -> {
                    if (PARAMS.executeRobotActions) lift2Low.run();
                })
                .build();

        // deliver specimen 5
        specimen5DeliveryPath = follower.pathBuilder()
                .addBezierCurve(new Point(specimen5PickFinal),
                        new Point(specimen5DeliverControl1), new Point(specimen5DeliverControl2),
                        new Point(specimen5DeliverFinal))
                .setLinearHeadingInterpolation(specimen5PickFinal.getHeading(), specimen5DeliverFinal.getHeading())
                .build();

        // park
        parkPath = follower.pathBuilder()
                .addBezierLine(new Point(specimen5DeliverFinal), new Point(parkFinalPose))
                .setLinearHeadingInterpolation(specimen5DeliverFinal.getHeading(), parkFinalPose.getHeading())
                .addTemporalCallback(1, () -> {
                    if (PARAMS.executeRobotActions) releaseLeftSpecimen.run();
                })
                .addTemporalCallback(10, () -> {
                    if (PARAMS.executeRobotActions) lift2Low.run();
                })
                .build();

        return this;
    }

    /**
     * Executes the autonomous workflow.
     */
    public void execute() {
        FtcLogger.enter();

        // Deliver specimen1
        if (!saveAndTest()) return;
        if (PARAMS.executeRobotActions) intakeFlipHorizontal.run();
        if (PARAMS.deliverSpecimen1) {
            if (PARAMS.executeRobotActions) {
                intakeFlipHorizontal.run();
                grabLeftSpecimen.run();
                lift2HighChamber.run();
            }

            if (PARAMS.executeTrajectories) runFollower(specimen1DeliveryPath, true, 2000);

            // Ensure lift has reached correct height
            if (PARAMS.executeRobotActions) lift2HighChamberBlocking.run();

            // Wait till lift stops swaying
            FtcUtils.sleep(FtcLift.LIFT_STOP_SWAYING_TIME);

            // Deliver
            if (PARAMS.executeRobotActions) lift2HighChamberDeliveryBlocking.run();
        }

        // Push sample1
        if (!saveAndTest()) return;
        if (PARAMS.pushSample1) {
            if (PARAMS.executeTrajectories) runFollower(sample1PickPath, false, 2000);
            if (PARAMS.executeRobotActions) robot.lift.resetLiftIfTouchPressed();
            if (PARAMS.executeTrajectories) runFollower(sample1DropPath, false, 2000);
        }

        // Push sample2
        if (!saveAndTest()) return;
        if (PARAMS.pushSample2) {
            if (PARAMS.executeTrajectories) runFollower(sample2PickPath, false, 2000);
            if (PARAMS.executeTrajectories) runFollower(sample2DropPath, false, 2000);
        }

        // Push sample3
        if (!saveAndTest()) return;
        if (PARAMS.pushSample3) {
            if (PARAMS.executeTrajectories) runFollower(sample3PickPath, false, 2000);
            if (PARAMS.executeTrajectories) runFollower(sample3DropPath, false, 2000);
        }


        // Deliver specimen2
        if (!saveAndTest()) return;
        if (PARAMS.deliverSpecimen2) {
            if (PARAMS.executeRobotActions) grabRightSpecimen.run();
            if (PARAMS.executeRobotActions) lift2HighChamber.run();
            if (PARAMS.executeTrajectories) runFollower(specimen2DeliveryPath, true, 2000);

            // Ensure lift has reached correct height
            if (PARAMS.executeRobotActions) lift2HighChamberBlocking.run();

            // Wait till lift stops swaying
            FtcUtils.sleep(FtcLift.LIFT_STOP_SWAYING_TIME);

            // Deliver
            if (PARAMS.executeRobotActions) lift2HighChamberDeliveryBlocking.run();
        }

        // Deliver specimen3
        if (!saveAndTest()) return;
        if (PARAMS.deliverSpecimen3) {
            if (PARAMS.executeTrajectories) runFollower(specimen3PickPath, true, 2000);
            if (PARAMS.executeRobotActions) robot.lift.resetLiftIfTouchPressed();
            if (PARAMS.executeRobotActions) grabLeftSpecimen.run();
            if (PARAMS.executeRobotActions) lift2HighChamber.run();
            if (PARAMS.executeTrajectories) runFollower(specimen3DeliveryPath, true, 2000);

            // Ensure lift has reached correct height
            if (PARAMS.executeRobotActions) lift2HighChamberBlocking.run();

            // Wait till lift stops swaying
            FtcUtils.sleep(FtcLift.LIFT_STOP_SWAYING_TIME);

            // Deliver
            if (PARAMS.executeRobotActions) lift2HighChamberDeliveryBlocking.run();
        }

        // Deliver specimen4
        if (!saveAndTest()) return;
        if (PARAMS.deliverSpecimen4) {
            if (PARAMS.executeTrajectories) runFollower(specimen4PickPath, true, 2000);
            if (PARAMS.executeRobotActions) robot.lift.resetLiftIfTouchPressed();
            if (PARAMS.executeRobotActions) grabRightSpecimen.run();
            if (PARAMS.executeRobotActions) lift2HighChamber.run();
            if (PARAMS.executeTrajectories) runFollower(specimen4DeliveryPath, true, 2000);

            // Ensure lift has reached correct height
            if (PARAMS.executeRobotActions) lift2HighChamberBlocking.run();

            // Wait till lift stops swaying
            FtcUtils.sleep(FtcLift.LIFT_STOP_SWAYING_TIME);

            // Deliver
            if (PARAMS.executeRobotActions) lift2HighChamberDeliveryBlocking.run();
        }

        // Deliver specimen5
        if (!saveAndTest()) return;
        if (PARAMS.deliverSpecimen5) {
            if (PARAMS.executeTrajectories) runFollower(specimen5PickPath, true, 2000);
            if (PARAMS.executeRobotActions) robot.lift.resetLiftIfTouchPressed();
            if (PARAMS.executeRobotActions) grabLeftSpecimen.run();
            if (PARAMS.executeRobotActions) lift2HighChamber.run();
            if (PARAMS.executeTrajectories) runFollower(specimen5DeliveryPath, true, 2000);

            // Ensure lift has reached correct height
            if (PARAMS.executeRobotActions) lift2HighChamberBlocking.run();

            // Wait till lift stops swaying
            FtcUtils.sleep(FtcLift.LIFT_STOP_SWAYING_TIME);

            // Deliver
            if (PARAMS.executeRobotActions) lift2HighChamberDeliveryBlocking.run();
        }

        // Park
        if (!saveAndTest()) return;
        if (PARAMS.park) {
            if (PARAMS.executeTrajectories) runFollower(parkPath, false, 2000);
            if (PARAMS.executeRobotActions) robot.lift.resetLiftIfTouchPressed();
        }

        FtcLogger.exit();
    }
}
