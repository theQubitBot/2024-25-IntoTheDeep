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

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

/**
 * A class to implement autonomous objective
 */
public class OptionRight extends OptionBase {

    boolean park = true;

    public OptionRight(LinearOpMode autoOpMode, FtcBot robot, MecanumDrive drive) {
        super(autoOpMode, robot, drive);
    }

    public OptionRight init() {
        super.initialize();

        // Park
        v1 = new Vector2d(4, -16);
        tab1 = drive.actionBuilder(startPose)
                .strafeToConstantHeading(v1);
        a1 = tab1.build();

        return this;
    }

    /**
     * Executes the autonomous workflow.
     */
    public void execute(boolean executeTrajectories, boolean executeRobotActions) {
        FtcLogger.enter();

        // Park
        if (!autoOpMode.opModeIsActive()) return;
        if (park) {
            if (executeTrajectories) Actions.runBlocking(a1);
            if (executeRobotActions) robot.intake.flipDown(false);
            if (executeRobotActions) robot.intake.spinStop();
            if (executeRobotActions) robot.rnp.retract(true);
        }

        FtcLogger.exit();
    }
}
