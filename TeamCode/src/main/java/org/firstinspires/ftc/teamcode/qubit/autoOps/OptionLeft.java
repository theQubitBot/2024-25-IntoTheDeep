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

package org.firstinspires.ftc.teamcode.qubit.autoOps;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLift;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

/**
 * A class to implement autonomous objective
 */
//@Config
public class OptionLeft extends OptionBase {
    public static class Params {
        public double v1x = -5, v1y = 13,
                v2x = 9.5, v2y = 29,
                v3x = -8, v3y = 13,
                v4x = 16, v4y = 30.896,
                v5x = -5, v5y = 13,
                v6x = 12, v6y = 26,
                v7x = -4, v7y = 8,
                v8x = 56, v8y = 25;

        public double h1 = -20, h2 = 100, h3 = 135, hp = -45;

        public boolean deliverPreloaded = true,
                deliverFirstYellow = true, deliverSecondYellow = true, deliverThirdYellow = true,
                park = true;
    }

    public static OptionLeft.Params PARAMS = new OptionLeft.Params();

    public OptionLeft(LinearOpMode autoOpMode, FtcBot robot, MecanumDrive drive) {
        super(autoOpMode, robot, drive);

        // Starting position
        startPose = new Pose2d(0, 0, 0);

        // preloaded sample
        v1 = new Vector2d(PARAMS.v1x, PARAMS.v1y);
        pose1 = new Pose2d(v1, RADIAN0);

        // first yellow sample
        v2 = new Vector2d(PARAMS.v2x, PARAMS.v2y);
        pose2 = new Pose2d(v2, Math.toRadians(PARAMS.h1));
        v3 = new Vector2d(PARAMS.v3x, PARAMS.v3y);
        pose3 = new Pose2d(v3, RADIAN0);

        // second yellow sample
        v4 = new Vector2d(PARAMS.v4x, PARAMS.v4y);
        pose4 = new Pose2d(v4, Math.toRadians(PARAMS.h2));
        v5 = new Vector2d(PARAMS.v5x, PARAMS.v5y);
        pose5 = new Pose2d(v5, RADIAN0);

        // third yellow sample
        v6 = new Vector2d(PARAMS.v6x, PARAMS.v6y);
        pose6 = new Pose2d(v6, Math.toRadians(PARAMS.h3));
        v7 = new Vector2d(PARAMS.v7x, PARAMS.v7y);
        pose7 = new Pose2d(v7, RADIAN0);

        // park
        v8 = new Vector2d(PARAMS.v8x, PARAMS.v8y);
        pose8 = new Pose2d(v8, Math.toRadians(PARAMS.hp));
    }

    public OptionLeft init() {
        super.initialize();

        // preloaded sample
        tab1 = drive.actionBuilder(startPose)
                .strafeToConstantHeading(v1);
        a1 = tab1.build();

        // first yellow sample
        tab2 = tab1.endTrajectory().fresh()
                .setTangent(RADIAN15)
                .splineToLinearHeading(pose2, pose2.heading);
        a2 = tab2.build();

        tab3 = tab2.endTrajectory().fresh()
                .strafeToLinearHeading(v3, startPose.heading);
        a3 = tab3.build();

        // second yellow sample
        tab4 = tab3.endTrajectory().fresh()
                .strafeToLinearHeading(v4, pose4.heading);
        a4 = tab4.build();

        tab5 = tab4.endTrajectory().fresh()
                .strafeToLinearHeading(v5, startPose.heading);
        a5 = tab5.build();

        // third yellow sample
        tab6 = tab5.endTrajectory().fresh()
                .setTangent(RADIAN15)
                .splineToLinearHeading(pose6, pose6.heading);
        a6 = tab6.build();

        tab7 = tab6.endTrajectory().fresh()
                .strafeToLinearHeading(v7, startPose.heading);
        a7 = tab7.build();

        // park
        tab8 = tab7.endTrajectory().fresh()
                .setTangent(RADIAN45)
                .splineToLinearHeading(pose8, pose8.heading);
        a8 = tab8.build();

        return this;
    }

    /**
     * Executes the autonomous workflow.
     */
    public void execute(boolean executeTrajectories, boolean executeRobotActions) {
        FtcLogger.enter();

        // Deliver preloaded sample
        if (!saveAndTest()) return;
        if (executeRobotActions) robot.intake.flipDown(false);
        if (PARAMS.deliverPreloaded) {
            if (executeRobotActions) robot.intake.flipDown(false);
            if (executeTrajectories) Actions.runBlocking(a1);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH_BASKET, FtcLift.POSITION_FLOOR, true);
            if (executeRobotActions) robot.arm.moveBackward(true);
            if (executeRobotActions) robot.arm.moveForward(true);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_FLOOR, FtcLift.POSITION_FLOOR, false);
        }

        // Deliver first yellow sample
        if (!saveAndTest()) return;
        if (PARAMS.deliverFirstYellow) {
            if (executeRobotActions) robot.intake.flipDown(false);
            if (executeRobotActions) robot.intake.spinIn(false);
            if (executeTrajectories) Actions.runBlocking(a2);
            if (executeRobotActions) robot.lift.resetLiftIfTouchPressed();
            if (executeRobotActions) robot.intake.flipDelivery(true);
            if (executeRobotActions) robot.intake.flipHorizontal(false);
            // In case sample is stuck, evict it.
            if (executeRobotActions) robot.intake.spinOut();
            if (executeTrajectories) Actions.runBlocking(a3);
            if (executeRobotActions) robot.intake.spinStop();
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH_BASKET, FtcLift.POSITION_FLOOR, true);

            if (executeRobotActions) robot.arm.moveBackward(true);
            if (executeRobotActions) robot.arm.moveForward(true);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_FLOOR, FtcLift.POSITION_FLOOR, false);
        }

        // Deliver second yellow sample
        if (!saveAndTest()) return;
        if (PARAMS.deliverSecondYellow) {
            if (executeRobotActions) robot.intake.flipDown(false);
            if (executeRobotActions) robot.intake.spinIn(false);
            if (executeTrajectories) Actions.runBlocking(a4);
            if (executeRobotActions) robot.lift.resetLiftIfTouchPressed();
            if (executeRobotActions) robot.intake.flipDelivery(true);
            if (executeRobotActions) robot.intake.flipHorizontal(false);
            // In case sample is stuck, evict it.
            if (executeRobotActions) robot.intake.spinOut();
            if (executeTrajectories) Actions.runBlocking(a5);
            if (executeRobotActions) robot.intake.spinStop();
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH_BASKET, FtcLift.POSITION_FLOOR, true);
            if (executeRobotActions) robot.arm.moveBackward(true);
            if (executeRobotActions) robot.arm.moveForward(true);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_FLOOR, FtcLift.POSITION_FLOOR, false);
        }

        // Deliver third yellow sample
        if (!saveAndTest()) return;
        if (PARAMS.deliverThirdYellow) {
            if (executeRobotActions) robot.intake.flipDown(false);
            if (executeRobotActions) robot.intake.spinIn(false);
            if (executeTrajectories) Actions.runBlocking(a6);
            if (executeRobotActions) robot.lift.resetLiftIfTouchPressed();
            if (executeRobotActions) robot.intake.flipDelivery(true);
            if (executeRobotActions) robot.intake.flipHorizontal(false);
            // In case sample is stuck, evict it.
            if (executeRobotActions) robot.intake.spinOut();
            if (executeTrajectories) Actions.runBlocking(a7);
            if (executeRobotActions) robot.intake.spinStop();
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH_BASKET, FtcLift.POSITION_FLOOR, true);
            if (executeRobotActions) robot.arm.moveBackward(true);
            if (executeRobotActions) robot.arm.moveForward(false);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_FLOOR, FtcLift.POSITION_FLOOR, false);
        }

        // Park
        if (!saveAndTest()) return;
        if (PARAMS.park) {
            robot.intake.flipHorizontal(false);
            if (executeTrajectories) Actions.runBlocking(a8);
            if (executeRobotActions) robot.lift.resetLiftIfTouchPressed();
            if (executeRobotActions) robot.flag.raise(false);
            if (executeRobotActions) {
                robot.lift.stop();
            }
        }

        FtcLogger.exit();
    }
}
