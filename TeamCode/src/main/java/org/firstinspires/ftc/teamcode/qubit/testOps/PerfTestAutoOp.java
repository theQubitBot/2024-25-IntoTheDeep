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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

import java.util.List;

/**
 * A performance test of raw hardware operations. This helps figure out what operations
 * may be slowing down the TeleOp loop.
 */
@Disabled
@Autonomous(group = "TestOp")
public class PerfTestAutoOp extends LinearOpMode {
    final int TEST_CYCLES = 1000;
    double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;
    double t7 = 0, t8 = 0, t9 = 0, t10 = 0;

    private DcMotorEx motor;
    private Servo servo;

    @Override
    public void runOpMode() {
        FtcLogger.enter();
        telemetry.addData(">", "Initializing. Please wait...");
        telemetry.update();

        motor = hardwareMap.get(DcMotorEx.class, "hookMotorOuter");
        servo = hardwareMap.get(Servo.class, "droneServo");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        ElapsedTime timer = new ElapsedTime();
        int cycles;
        telemetry.addData(">", "Waiting for driver to press play.");
        telemetry.update();
        waitForStart();

        displayCycleTimes("running test...");

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            motor.getCurrentPosition();
        }

        t1 = timer.milliseconds() / cycles;
        displayCycleTimes("getCurrentPosition.");

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            motor.getVelocity();
        }

        t2 = timer.milliseconds() / cycles;
        displayCycleTimes("getVelocity.");

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            motor.getPower();
        }

        t3 = timer.milliseconds() / cycles;
        displayCycleTimes("getPower.");

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            motor.setPower(0);
        }

        t4 = timer.milliseconds() / cycles;
        displayCycleTimes("setPower.");

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            servo.getPosition();
        }

        t5 = timer.milliseconds() / cycles;
        displayCycleTimes("getPosition.");

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            servo.setPosition((Servo.MAX_POSITION + Servo.MIN_POSITION) / 2.0);
        }

        t6 = timer.milliseconds() / cycles;
        displayCycleTimes("setPosition.");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            imu.getRobotYawPitchRollAngles();
        }

        t7 = timer.milliseconds() / cycles;
        displayCycleTimes("getRobotYawPitchRollAngles.");

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        }

        t8 = timer.milliseconds() / cycles;
        displayCycleTimes("getRobotAngularVelocity.");

        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            motor.getCurrentPosition();
            motor.getVelocity();
            motor.getPower();
            motor.setPower(0);
            servo.getPosition();
            servo.setPosition((Servo.MAX_POSITION + Servo.MIN_POSITION) / 2.0);
            imu.getRobotYawPitchRollAngles();
            imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        }

        t9 = timer.milliseconds() / cycles;

        displayCycleTimes("complete.");

        while (opModeIsActive()) {
            FtcUtils.sleep(100);
        }

        FtcLogger.exit();
    }

    void displayCycleTimes(String status) {
        telemetry.addData(">", status);
        telemetry.addData("Motor.getCurrentPosition", "%5.1f ms/cycle", t1);
        telemetry.addData("Motor.getVelocity", "%5.1f ms/cycle", t2);
        telemetry.addData("Motor.getPower", "%5.1f ms/cycle", t3);
        telemetry.addData("Motor.setPower", "%5.1f ms/cycle", t4);
        telemetry.addData("Servo.getPosition", "%5.1f ms/cycle", t5);
        telemetry.addData("Servo.setPosition", "%5.1f ms/cycle", t6);
        telemetry.addData("IMU.getRobotAngularVelocity", "%5.1f ms/cycle", t7);
        telemetry.addData("IMU.getRobotYawPitchRollAngles", "%5.1f ms/cycle", t8);
        telemetry.addData("all", "%5.1f ms/cycle", t9);
        telemetry.update();
    }
}
