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

package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcRnp;
import org.firstinspires.ftc.teamcode.qubit.core.FtcServo;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

//@Disabled
@TeleOp(group = "TestOp")
public class RnpCalibrationTeleOp extends OpMode {
    // Declare OpMode members
    private ElapsedTime runtime = null;
    private ElapsedTime loopTime = null;

    private TouchSensor extendLimitSwitch = null;
    private TouchSensor retractLimitSwitch = null;
    FtcServo servo = null;
    double position;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FtcLogger.enter();
        telemetry.addData(FtcUtils.TAG, "Initializing, please wait...");
        telemetry.update();

        extendLimitSwitch = hardwareMap.get(TouchSensor.class, FtcRnp.EXTEND_TOUCH_SENSOR);
        retractLimitSwitch = hardwareMap.get(TouchSensor.class, FtcRnp.RETRACT_TOUCH_SENSOR);

        servo = new FtcServo(hardwareMap.get(Servo.class, FtcRnp.RNP_SERVO_NAME));
        if (servo.getController().getPwmStatus() != ServoController.PwmStatus.ENABLED) {
            servo.getController().pwmEnable();
        }

        servo.setDirection(Servo.Direction.REVERSE);
        position = FtcRnp.RNP_STOP_POWER;
        servo.setPosition(position);

        FtcLogger.exit();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play");
        telemetry.update();
        FtcUtils.sleep(FtcUtils.CYCLE_MS);
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        FtcLogger.enter();
        telemetry.addData(FtcUtils.TAG, "Starting...");
        telemetry.update();
        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        FtcLogger.exit();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        FtcLogger.enter();
        loopTime.reset();

        if (gamepad1.dpad_up && !extendLimitSwitch.isPressed()) {
            position = FtcRnp.RNP_EXTEND_POWER;
        } else if (gamepad1.dpad_down && !retractLimitSwitch.isPressed()) {
            position = FtcRnp.RNP_RETRACT_POWER;
        } else {
            position = FtcRnp.RNP_STOP_POWER;
        }

        servo.setPosition(position);
        telemetry.addData("RnP", "dPad up/down");
        telemetry.addData("Position", "%5.4f", position);
        telemetry.addData("Limits", "extended: %b, retracted: %b",
                extendLimitSwitch.isPressed(), retractLimitSwitch.isPressed());
        telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
                loopTime.milliseconds(), runtime.seconds());
        telemetry.update();
        FtcUtils.sleep(FtcUtils.CYCLE_MS);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        FtcLogger.enter();
        telemetry.addData(FtcUtils.TAG, "Tele Op stopped.");
        telemetry.update();
        FtcLogger.exit();
    }
}
