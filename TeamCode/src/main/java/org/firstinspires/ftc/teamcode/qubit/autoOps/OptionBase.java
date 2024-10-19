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

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

/**
 * A base class to provide common variables and methods.
 */
public class OptionBase {
    public static final double RADIAN0;
    public static final double RADIAN30;
    public static final double RADIAN45;
    public static final double RADIAN60;
    public static final double RADIAN90;
    public static final double RADIAN180;
    protected LinearOpMode autoOpMode;
    protected FtcBot robot;
    protected MecanumDrive drive;

    protected Pose2d startPose, pose1, pose2, pose3, pose4, pose5, pose6,
            pose7, pose8, pose9, pose10, pose11, pose12;
    protected Vector2d v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12;
    protected TrajectoryActionBuilder tab1, tab2, tab3, tab4, tab5, tab6,
            tab7, tab8, tab9, tab10, tab11, tab12;
    protected Action a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12;

    static {
        RADIAN0 = Math.toRadians(0);
        RADIAN30 = Math.toRadians(30);
        RADIAN45 = Math.toRadians(45);
        RADIAN60 = Math.toRadians(60);
        RADIAN90 = Math.toRadians(90);
        RADIAN180 = Math.toRadians(180);
    }

    public OptionBase(LinearOpMode autoOpMode, FtcBot robot, MecanumDrive drive) {
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
        FtcLogger.exit();
    }
}
