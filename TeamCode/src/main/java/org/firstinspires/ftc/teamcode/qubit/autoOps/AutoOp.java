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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.qubit.core.FtcBot;
import org.firstinspires.ftc.teamcode.qubit.core.FtcImu;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.AllianceColorEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.RobotPositionEnum;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

@Autonomous(group = "Official", preselectTeleOp = "DriverTeleOp")
public class AutoOp extends LinearOpMode {
    FtcBot robot = null;
    SampleMecanumDrive drive = null;

    @Override
    public void runOpMode() {
        FtcLogger.enter();
        initializeModules();
        processVisionDuringInit();
        executeAutonomousOperation();
        waitForEnd();
        FtcLogger.exit();
    }

    /**
     * Invokes autonomous operation based on match configuration.
     */
    private void executeAutonomousOperation() {
        FtcLogger.enter();
        if (robot.config.delayInSeconds > 0) {
            long countDown = robot.config.delayInSeconds;
            while (countDown > 0) {
                if (!opModeIsActive()) return;
                telemetry.addData(">", "Delaying start by %d seconds",
                        robot.config.delayInSeconds);
                telemetry.addData(">", "Countdown %d seconds", countDown);
                telemetry.update();
                FtcUtils.sleep(1000);
                countDown--;
            }
        }

        telemetry.addData(">", "Auto Op started.");
        telemetry.update();

        if (!opModeIsActive()) return;
        if (robot.config.allianceColor == AllianceColorEnum.RED) {
            if (robot.config.robotPosition == RobotPositionEnum.LEFT) {
                new OptionRedLeft(this, robot, drive).init().execute();
            } else {
                new OptionRedRight(this, robot, drive).init().execute();
            }
        } else { // BLUE
            if (robot.config.robotPosition == RobotPositionEnum.LEFT) {
                new OptionBlueLeft(this, robot, drive).init().execute();
            } else {
                new OptionBlueRight(this, robot, drive).init().execute();
            }
        }

        FtcLogger.exit();
    }

    /**
     * Initialize the variables, etc.
     */
    private void initializeModules() {
        FtcLogger.enter();
        telemetry.addData(">", "Initializing. Please wait...");
        telemetry.update();

        // Initialize robot.
        robot = new FtcBot();
        robot.init(hardwareMap, telemetry, true);
        robot.start();

        // Clear out any previous end heading of the robot.
        FtcImu.endAutoOpHeading = 0;

        if (FtcUtils.DEBUG) {
            robot.enableTelemetry();
        } else {
            // Disable telemetry to speed things up.
            robot.disableTelemetry();
        }

        // Initialize roadrunner for robot paths and trajectories
        // Must initialize this after robot.driveTrain initialization since driveTrain
        // sets the motors to run without encoders.
        drive = new SampleMecanumDrive(hardwareMap);

        FtcLogger.exit();
    }

    private void processVisionDuringInit() {
        FtcLogger.enter();

        while (opModeInInit()) {
            robot.config.showConfiguration();
            telemetry.addLine();
            robot.config.teamPropLocation = robot.openCvCam.getTeamPropPosition(robot.config.allianceColor);
            telemetry.addData(">", "TeamProp: %s", robot.config.teamPropLocation);
            if (robot.imu.isGyroDrifting()) {
                telemetry.addLine();
                telemetry.addData(">", "Gyro %.1f is DRIFTING! STOP and ReInitialize.",
                        robot.imu.getHeading());
            }

            telemetry.addData(">", "Waiting for driver to press play.");
            telemetry.update();
            FtcUtils.sleep(5);
        }

        // PERFORMANCE
        // Stop the camera to conserve CPU and battery.
        robot.openCvCam.stop();

        FtcLogger.exit();
    }

    /**
     * Waits for the autonomous operation to end.
     * If using FOD, saves gyro Heading for use by TeleOp.
     */
    private void waitForEnd() {
        FtcLogger.enter();

        do {
            // Save settings for use by TeleOp
            if (robot.driveTrain.driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE &&
                    robot.driveTrain.driveTypeEnum == DriveTypeEnum.FIELD_ORIENTED_DRIVE) {
                robot.imu.read();
                FtcImu.endAutoOpHeading = robot.imu.getHeading();
                telemetry.addData(">", "endGyroHeading=%.1f",
                        FtcImu.endAutoOpHeading);
                telemetry.addData(">", "Waiting for auto Op to end.");
                telemetry.update();
                FtcUtils.sleep(5);
            } else {
                // End Auto Op so that we can switch to Tele Op ASAP.
                // Ending soon also stops the robot and saves on battery.
                break;
            }
        } while (opModeIsActive());

        robot.stop();
        FtcLogger.exit();
    }
}
