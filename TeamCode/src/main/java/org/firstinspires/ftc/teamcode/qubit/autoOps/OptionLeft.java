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

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLift;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcRelay;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

/**
 * A class to implement autonomous objective
 */
public class OptionLeft extends OptionBase {

    boolean deliverPreloaded = false, deliverSubmersible = false,
            deliverFirstYellow = false, deliverSecondYellow = false, deliverThirdYellow = false,
            park = false;

    public OptionLeft(LinearOpMode autoOpMode, FtcBot robot, MecanumDrive drive) {
        super(autoOpMode, robot, drive);
    }

    public OptionLeft init() {
        super.initialize();

        // preloaded specimen
        v1 = new Vector2d(14, -4);
        tab1 = drive.actionBuilder(startPose)
                .strafeToConstantHeading(v1);
        a1 = tab1.build();

        // submersible sample
        v2 = new Vector2d(16, -4);
        tab2 = tab1.fresh()
                .strafeToConstantHeading(v2);
        a2 = tab2.build();

        v3 = new Vector2d(20, 10);
        tab3 = tab2.fresh()
                .setReversed(true)
                .splineToConstantHeading(v3, RADIAN90);
        a3 = tab3.build();

        // first yellow sample
        v4 = new Vector2d(5, -7);
        tab4 = tab3.fresh()
                .setReversed(false)
                .strafeToLinearHeading(v4, -RADIAN60);
        a4 = tab4.build();

        v5 = new Vector2d(-5, 7);
        tab5 = tab4.fresh()
                .setReversed(true)
                .strafeToConstantHeading(v5);
        a5 = tab5.build();

        // second yellow sample
        v6 = new Vector2d(5, -5);
        tab6 = tab5.fresh()
                .setReversed(false)
                .strafeToLinearHeading(v6, -RADIAN45);
        a6 = tab6.build();

        v7 = new Vector2d(-5, 5);
        tab7 = tab6.fresh()
                .setReversed(true)
                .strafeToConstantHeading(v7);
        a7 = tab7.build();

        // third yellow sample
        v8 = new Vector2d(5, -3);
        tab8 = tab7.fresh()
                .setReversed(false)
                .strafeToLinearHeading(v8, -RADIAN45);
        a8 = tab8.build();

        v9 = new Vector2d(-5, 3);
        tab9 = tab8.fresh()
                .setReversed(true)
                .strafeToConstantHeading(v9);
        a9 = tab9.build();

        // park
        pose10 = new Pose2d(15, -10, -RADIAN90);
        tab10 = tab9.fresh()
                .setReversed(false)
                .splineToLinearHeading(pose10, -RADIAN90);
        a10 = tab10.build();

        return this;
    }

    /**
     * Executes the autonomous workflow.
     */
    public void execute() {
        FtcLogger.enter();

        // Deliver preloaded specimen
        if (!autoOpMode.opModeIsActive()) return;
        if (deliverPreloaded) {
            Actions.runBlocking(
                    new SequentialAction(
                            // Hold the specimen
                            new InstantAction(() -> robot.relay.spinHold()),
                            new ParallelAction(
                                    // Raise arm for high chamber
                                    new InstantAction(() -> robot.relay.moveArm(FtcRelay.ARM_HANG_POSITION)),
                                    // Move towards submersible
                                    a1,
                                    // Extend RNP
                                    new InstantAction(() -> robot.relay.rnpExtend())
                            ),
                            // Wait for RNP extension
                            new SleepAction(0.5),
                            // Deliver specimen by retracting RNP
                            new InstantAction(() -> robot.relay.rnpRetract()),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.spinStop()),
                                    new InstantAction(() -> robot.relay.rnpStop()),
                                    // Lower arm
                                    new InstantAction(() -> robot.relay.moveArm(FtcRelay.ARM_FORWARD_POSITION))
                            )
                    )
            );
        }

        // Deliver submersible sample
        if (!autoOpMode.opModeIsActive()) return;
        if (deliverSubmersible) {
            Actions.runBlocking(
                    new SequentialAction(
                            a2,
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.spinIn()),
                                    new InstantAction(() -> robot.relay.rnpExtend())
                            ),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.spinHold()),
                                    new InstantAction(() -> robot.relay.rnpRetract())
                            ),
                            a3,
                            // Deliver sample to high basket
                            new ParallelAction(
                                    new InstantAction(() -> robot.lift.move(FtcLift.POSITION_HIGH, FtcLift.POSITION_HIGH, false)),
                                    new InstantAction(() -> robot.relay.moveArm(FtcRelay.ARM_BACKWARD_POSITION)),
                                    new InstantAction(() -> robot.relay.rnpExtend())
                            ),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.rnpStop()),
                                    new InstantAction(() -> robot.relay.spinOut())
                            ),
                            new SleepAction(1.0),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.moveArm(FtcRelay.ARM_FORWARD_POSITION)),
                                    new InstantAction(() -> robot.relay.spinStop()),
                                    new InstantAction(() -> robot.relay.rnpRetract())
                            )
                    )
            );
        }

        // Deliver first yellow sample
        if (!autoOpMode.opModeIsActive()) return;
        if (deliverFirstYellow) {
            Actions.runBlocking(
                    new SequentialAction(
                            a4,
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.spinIn())
                            ),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.spinHold())
                            ),
                            a5,
                            // Deliver first yellow sample to high basket
                            new ParallelAction(
                                    new InstantAction(() -> robot.lift.move(FtcLift.POSITION_HIGH, FtcLift.POSITION_HIGH, false)),
                                    new InstantAction(() -> robot.relay.moveArm(FtcRelay.ARM_BACKWARD_POSITION)),
                                    new InstantAction(() -> robot.relay.rnpExtend())
                            ),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.rnpStop()),
                                    new InstantAction(() -> robot.relay.spinOut())
                            ),
                            new SleepAction(1.0),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.moveArm(FtcRelay.ARM_FORWARD_POSITION)),
                                    new InstantAction(() -> robot.relay.spinStop()),
                                    new InstantAction(() -> robot.relay.rnpRetract())
                            )
                    )
            );

        }
        // Deliver second yellow sample
        if (!autoOpMode.opModeIsActive()) return;
        if (deliverSecondYellow) {
            Actions.runBlocking(
                    new SequentialAction(
                            a6,
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.spinIn())
                            ),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.spinHold())
                            ),
                            a7,
                            // Deliver second yellow sample to high basket
                            new ParallelAction(
                                    new InstantAction(() -> robot.lift.move(FtcLift.POSITION_HIGH, FtcLift.POSITION_HIGH, false)),
                                    new InstantAction(() -> robot.relay.moveArm(FtcRelay.ARM_BACKWARD_POSITION)),
                                    new InstantAction(() -> robot.relay.rnpExtend())
                            ),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.rnpStop()),
                                    new InstantAction(() -> robot.relay.spinOut())
                            ),
                            new SleepAction(1.0),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.moveArm(FtcRelay.ARM_FORWARD_POSITION)),
                                    new InstantAction(() -> robot.relay.spinStop()),
                                    new InstantAction(() -> robot.relay.rnpRetract())
                            )
                    )
            );

        }

        // Deliver third yellow sample
        if (!autoOpMode.opModeIsActive()) return;
        if (deliverThirdYellow) {
            Actions.runBlocking(
                    new SequentialAction(
                            a8,
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.spinIn())
                            ),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.spinHold())
                            ),
                            a9,
                            // Deliver third yellow sample to high basket
                            new ParallelAction(
                                    new InstantAction(() -> robot.lift.move(FtcLift.POSITION_HIGH, FtcLift.POSITION_HIGH, false)),
                                    new InstantAction(() -> robot.relay.moveArm(FtcRelay.ARM_BACKWARD_POSITION)),
                                    new InstantAction(() -> robot.relay.rnpExtend())
                            ),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.rnpStop()),
                                    new InstantAction(() -> robot.relay.spinOut())
                            ),
                            new SleepAction(1.0),
                            new ParallelAction(
                                    new InstantAction(() -> robot.relay.moveArm(FtcRelay.ARM_FORWARD_POSITION)),
                                    new InstantAction(() -> robot.relay.spinStop()),
                                    new InstantAction(() -> robot.relay.rnpRetract())
                            )
                    )
            );

        }

        // Park
        if (!autoOpMode.opModeIsActive()) return;
        if (park) {
            Actions.runBlocking(a10);
        }

        FtcLogger.exit();
    }
}
