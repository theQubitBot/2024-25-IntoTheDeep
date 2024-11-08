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
public class OptionLeft extends OptionBase {

    boolean deliverPreloaded = true,
            deliverFirstYellow = true, deliverSecondYellow = true, deliverThirdYellow = true,
            park = true;

    public OptionLeft(LinearOpMode autoOpMode, FtcBot robot, MecanumDrive drive) {
        super(autoOpMode, robot, drive);
    }

    public OptionLeft init() {
        super.initialize();

        // preloaded sample
        v1 = new Vector2d(-5, 12.10);
        tab1 = drive.actionBuilder(startPose)
                .strafeToConstantHeading(v1);
        a1 = tab1.build();

        // first yellow sample
        v2 = new Vector2d(17, 11.5);
        tab2 = tab1.fresh()
                .strafeToConstantHeading(v2);
        a2 = tab2.build();

        v3 = new Vector2d(-17, -11.5);
        tab3 = tab2.fresh()
                .strafeToConstantHeading(v3);
        a3 = tab3.build();

        // second yellow sample
        v4 = new Vector2d(11, 9);
        tab4 = tab3.fresh()
                .strafeToLinearHeading(v4, RADIAN45);
        a4 = tab4.build();

        v5 = new Vector2d(-17, -15);
        tab5 = tab4.fresh()
                // It seems that fresh() resets the robot internally to (0,0) at current location.
                // RR remembers the robot org heading and reorients the bot towards org heading .
                .strafeToConstantHeading(v5);
        a5 = tab5.build();

        // third yellow sample
        v6 = new Vector2d(25, 14);
        tab6 = tab5.fresh()
                .strafeToLinearHeading(v6, RADIAN135);
        a6 = tab6.build();

        v7 = new Vector2d(-34, -24);
        tab7 = tab6.fresh()
                .strafeToLinearHeading(v7, RADIAN0);
        a7 = tab7.build();

        // park
        v8 = new Vector2d(65, 5);
        pose8 = new Pose2d(v8, -RADIAN45);
        tab8 = tab7.fresh()
                .setTangent(RADIAN45)
                .splineToLinearHeading(pose8, -RADIAN45);
        a8 = tab8.build();

        return this;
    }

    /**
     * Executes the autonomous workflow.
     */
    public void execute(boolean executeRobotActions) {
        FtcLogger.enter();

        // Deliver preloaded sample
        if (!autoOpMode.opModeIsActive()) return;
        if (deliverPreloaded) {
            if (executeRobotActions) robot.intake.flipHorizontal(false);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH, FtcLift.POSITION_LOW, true);
            Actions.runBlocking(a1);
            //reverseCrawlToBucket(true);
            if (executeRobotActions) robot.arm.moveBackward(true);
            if (executeRobotActions) robot.arm.moveForward(true);
        }

        // Deliver first yellow sample
        if (!autoOpMode.opModeIsActive()) return;
        if (deliverFirstYellow) {
            if (executeRobotActions) robot.intake.flipOut(false);
            if (executeRobotActions) robot.rnp.extend(false);
            if (executeRobotActions) robot.intake.spinIn();
            Actions.runBlocking(a2);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_LOW, FtcLift.POSITION_LOW, false);
            if (executeRobotActions) robot.rnp.retract(true);
            if (executeRobotActions) robot.intake.flipDelivery(true);
            if (executeRobotActions) robot.intake.spinStop();
            if (executeRobotActions) robot.intake.flipHorizontal(true);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH, FtcLift.POSITION_LOW, true);
            Actions.runBlocking(a3);
            //reverseCrawlToBucket(true);
            if (executeRobotActions) robot.arm.moveBackward(true);
            if (executeRobotActions) robot.arm.moveForward(true);
        }

        // Deliver second yellow sample
        if (!autoOpMode.opModeIsActive()) return;
        if (deliverSecondYellow) {
            if (executeRobotActions) robot.intake.flipOut(false);
            if (executeRobotActions) robot.intake.spinIn();
            Actions.runBlocking(a4);
            if (executeRobotActions) robot.rnp.extend(false);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_LOW, FtcLift.POSITION_LOW, false);
            if (executeRobotActions) robot.rnp.retract(true);
            if (executeRobotActions) robot.intake.flipDelivery(true);
            if (executeRobotActions) robot.intake.spinStop();
            if (executeRobotActions) robot.intake.flipHorizontal(true);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH, FtcLift.POSITION_LOW, true);
            Actions.runBlocking(a5);
            //reverseCrawlToBucket(true);
            if (executeRobotActions) robot.arm.moveBackward(true);
            if (executeRobotActions) robot.arm.moveForward(true);
        }

        // Deliver third yellow sample
        if (!autoOpMode.opModeIsActive()) return;
        if (deliverThirdYellow) {
            if (executeRobotActions) robot.intake.flipOut(false);
            if (executeRobotActions) robot.rnp.extend(false);
            if (executeRobotActions) robot.intake.spinIn();
            Actions.runBlocking(a6);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_LOW, FtcLift.POSITION_LOW, false);
            if (executeRobotActions) robot.rnp.retract(true);
            if (executeRobotActions) robot.intake.flipDelivery(true);
            if (executeRobotActions) robot.intake.spinStop();
            if (executeRobotActions) robot.intake.flipHorizontal(true);
            if (executeRobotActions)
                robot.lift.move(FtcLift.POSITION_HIGH, FtcLift.POSITION_LOW, true);
            Actions.runBlocking(a7);
            //reverseCrawlToBucket(true);
            if (executeRobotActions) robot.arm.moveBackward(true);
            if (executeRobotActions) robot.arm.moveForward(true);
        }

        // Park
        if (!autoOpMode.opModeIsActive()) return;
        if (park) {
            if (executeRobotActions) robot.intake.flipDelivery(false);
            Actions.runBlocking(a8);
            robot.flag.raise(false);
            if (executeRobotActions) robot.lift.move(FtcLift.POSITION_LOW, FtcLift.POSITION_LOW, false);
        }

        FtcLogger.exit();
    }
}
