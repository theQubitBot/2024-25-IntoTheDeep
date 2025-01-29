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
import org.firstinspires.ftc.teamcode.qubit.core.FtcLift;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;

/**
 * A class to implement autonomous objective
 */
@Config
public class OptionRight extends OptionBase {
    // Preloaded Specimen
    Pose scorePose = new Pose(-3.6, 30.5, RADIAN0);

    // Park
    public Pose parkControlPose1 = new Pose(-3.6, 11, RADIAN0);
    public Pose parkPose = new Pose(34, 6, RADIAN0);
    PathChain scorePreloadPath, parkPath;

    public static class Params {
        public boolean executeTrajectories = true, executeRobotActions = true;
        public boolean deliverPreloaded = true,
                park = true;
    }

    public static Params PARAMS = new Params();

    public OptionRight(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
        super(autoOpMode, robot, follower);
        follower.setStartingPose(startPose);
    }

    public OptionRight init() {
        super.initialize();

        // preloaded specimen
        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // park
        parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),
                        new Point(parkControlPose1),
                        new Point(parkPose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        return this;
    }

    /**
     * Executes the autonomous workflow.
     */
    public void execute() {
        FtcLogger.enter();

        // Deliver preloaded specimen
        if (!saveAndTest()) return;
        if (PARAMS.executeRobotActions) robot.intake.flipDown(false);
        if (PARAMS.deliverPreloaded) {
            if (PARAMS.executeRobotActions) robot.intake.flipDown(false);
            if (PARAMS.executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH_CHAMBER, FtcLift.POSITION_FLOOR, false);
            if (PARAMS.executeTrajectories) runFollower(scorePreloadPath, true, 2500);
            ;
            if (PARAMS.executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH_CHAMBER_DELIVERY, FtcLift.POSITION_FLOOR, true);
        }

        // Park
        if (!saveAndTest()) return;
        if (PARAMS.park) {
            if (PARAMS.executeRobotActions)
                robot.lift.move(FtcLift.POSITION_FLOOR, FtcLift.POSITION_FLOOR, false);
            if (PARAMS.executeTrajectories) runFollower(parkPath, true, 3000);
            if (PARAMS.executeRobotActions) robot.lift.resetLiftIfTouchPressed();
        }

        FtcLogger.exit();
    }
}
