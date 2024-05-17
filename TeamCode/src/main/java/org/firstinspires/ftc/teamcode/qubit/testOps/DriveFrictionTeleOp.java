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

package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcDriveTrain;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcMotor;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;

import java.util.Locale;

//@Disabled
@TeleOp(group = "TestOp")
public class DriveFrictionTeleOp extends OpMode {
    // Declare OpMode members
    private ElapsedTime runtime = null;
    private ElapsedTime loopTime = null;
    private boolean lastDPadUpPressed = false, lastDPadDownPressed = false;
    double newMotorPower = FtcMotor.ZERO_POWER, oldMotorPower = FtcMotor.ZERO_POWER;
    double rampUpDownPower = 0.01;
    double lfVelocity, lrVelocity, rfVelocity, rrVelocity, maxVelocity;
    FtcDriveTrain driveTrain = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FtcLogger.enter();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        telemetry.addData(">", "Initializing, please wait...");
        telemetry.update();
        driveTrain = new FtcDriveTrain(null);
        driveTrain.setDriveTypeAndMode(DriveTrainEnum.MECANUM_WHEEL_DRIVE, DriveTypeEnum.FIELD_ORIENTED_DRIVE);
        driveTrain.init(hardwareMap, telemetry);
        driveTrain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        FtcLogger.exit();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData(">", "Waiting for driver to press play");
        telemetry.update();
        FtcUtils.sleep(50);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        FtcLogger.enter();
        telemetry.addData(">", "Starting...");
        telemetry.update();
        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        driveTrain.telemetryEnabled = FtcUtils.DEBUG;
        FtcLogger.exit();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        FtcLogger.enter();
        // Show the elapsed game time and wheel power.
        loopTime.reset();

        telemetry.addData(">", "dPad up/down for power up/down");
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            if (!lastDPadUpPressed) {
                lastDPadUpPressed = true;
                newMotorPower += rampUpDownPower;
            }
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            if (!lastDPadDownPressed) {
                lastDPadDownPressed = true;
                newMotorPower -= rampUpDownPower;
            }
        } else {
            lastDPadUpPressed = false;
            lastDPadDownPressed = false;
        }

        newMotorPower = Range.clip(newMotorPower,
                FtcMotor.ZERO_POWER, FtcDriveTrain.MAXIMUM_FORWARD_POWER);
        if (newMotorPower != oldMotorPower) {
            driveTrain.setDrivePower(newMotorPower, newMotorPower, newMotorPower, newMotorPower);
            oldMotorPower = newMotorPower;
        }

        // Emit telemetry for graph plots in RoadRunner Dashboard
        lfVelocity = Math.abs(driveTrain.allMotors.get(0).getVelocity());
        lrVelocity = Math.abs(driveTrain.allMotors.get(1).getVelocity());
        rfVelocity = Math.abs(driveTrain.allMotors.get(2).getVelocity());
        rrVelocity = Math.abs(driveTrain.allMotors.get(3).getVelocity());
        maxVelocity = Math.max(Math.max(lfVelocity, lrVelocity), Math.max(rfVelocity, rrVelocity));
        telemetry.addData(">", String.format(Locale.US, "Motor power %.2f", newMotorPower));
        telemetry.addData("LF", maxVelocity - lfVelocity);
        telemetry.addData("LR", maxVelocity - lrVelocity);
        telemetry.addData("RF", maxVelocity - rfVelocity);
        telemetry.addData("RR", maxVelocity - rrVelocity);

        telemetry.addData(">", "Loop %.0f ms, cumulative %.0f seconds",
                loopTime.milliseconds(), runtime.seconds());
        telemetry.update();
        FtcLogger.exit();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        FtcLogger.enter();
        driveTrain.stop();
        this.telemetry.addData(">", "Tele Op stopped.");
        this.telemetry.update();
        FtcLogger.exit();
    }
}
