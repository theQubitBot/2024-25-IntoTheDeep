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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcImu;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLift;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;

import java.util.concurrent.TimeUnit;

/**
 * A base class to provide common variables and methods.
 */
public class OptionBase {
    protected static final double RADIAN0;
    protected static final double RADIAN15;
    protected static final double RADIAN20;
    protected static final double RADIAN30;
    protected static final double RADIAN45;
    protected static final double RADIAN60;
    protected static final double RADIAN75;
    protected static final double RADIAN90;
    protected static final double RADIAN105;
    protected static final double RADIAN120;
    protected static final double RADIAN135;
    protected static final double RADIAN150;
    protected static final double RADIAN180;
    protected LinearOpMode autoOpMode;
    protected FtcBot robot;
    protected Follower follower;
    protected final Pose startPose = new Pose(0, 0, 0);

    static {
        RADIAN0 = Math.toRadians(0);
        RADIAN15 = Math.toRadians(15);
        RADIAN20 = Math.toRadians(20);
        RADIAN30 = Math.toRadians(30);
        RADIAN45 = Math.toRadians(45);
        RADIAN60 = Math.toRadians(60);
        RADIAN75 = Math.toRadians(75);
        RADIAN90 = Math.toRadians(90);
        RADIAN105 = Math.toRadians(105);
        RADIAN120 = Math.toRadians(120);
        RADIAN135 = Math.toRadians(135);
        RADIAN150 = Math.toRadians(150);
        RADIAN180 = Math.toRadians(180);
    }

    public OptionBase(LinearOpMode autoOpMode, FtcBot robot, Follower follower) {
        this.autoOpMode = autoOpMode;
        this.robot = robot;
        this.follower = follower;
    }

    /**
     * This method executes the path chain.
     */
    public void runFollower(PathChain pathChain, boolean holdEnd, long timeout) {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        follower.followPath(pathChain, holdEnd);
        if (timeout < 0) timeout = Long.MAX_VALUE;
        Deadline d = new Deadline(timeout, TimeUnit.MILLISECONDS);
        do {
            follower.update();
        } while (autoOpMode.opModeIsActive() && !d.hasExpired() && follower.isBusy());
        if (follower.isBusy()) follower.breakFollowing();
        autoOpMode.telemetry.addData(FtcUtils.TAG, "Path execution: %.0f ms", runtime.milliseconds());
        autoOpMode.telemetry.update();
    }

    public boolean saveAndTest() {
        if (autoOpMode.opModeIsActive()) {
            return true;
        } else {
            // Save settings for use by TeleOp
            FtcLift.endAutoOpLeftLiftPosition = robot.lift.getLeftPosition();
            FtcLift.endAutoOpRightLiftPosition = robot.lift.getRightPosition();
            if (robot.driveTrain.driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE &&
                    robot.driveTrain.driveTypeEnum == DriveTypeEnum.FIELD_ORIENTED_DRIVE) {
                robot.imu.read();
                FtcImu.endAutoOpHeading = robot.imu.getHeading();
            }

            return false;
        }
    }

    public void runCallbacks(PathChain pathChain, ElapsedTime pathChainElapsedTime) {
        for (PathCallback callback : pathChain.getCallbacks()) {
            if (!callback.hasBeenRun() && callback.getType() == PathCallback.TIME &&
                    pathChainElapsedTime.milliseconds() >= callback.getStartCondition()) {
                callback.run();
            }
        }
    }

    /**
     * Initializes variables and state.
     */
    protected void initialize() {
        FtcLogger.enter();

        FtcLogger.exit();
    }
}
