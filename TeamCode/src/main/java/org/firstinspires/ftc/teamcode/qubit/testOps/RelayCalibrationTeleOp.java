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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcRelay;
import org.firstinspires.ftc.teamcode.qubit.core.FtcServo;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

//@Disabled
@TeleOp(group = "TestOp")
public class RelayCalibrationTeleOp extends OpMode {
    // Declare OpMode members
    private ElapsedTime runtime = null;
    private ElapsedTime loopTime = null;
    static final int CYCLE_MS = 50;           // period of each cycle
    static final String SERVO_NAME = "";

    Servo spinServo;
    double spinPower;
    Servo rackNPinionServo;
    double rackNPinionPosition;
    Servo leftLiftServo;
    Servo rightLiftServo;
    double leftLiftServoPosition;
    double rightLiftServoPosition;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FtcLogger.enter();
        telemetry.addData(">", "Initializing, please wait...");
        telemetry.update();

        spinServo = hardwareMap.get(Servo.class, FtcRelay.SPINNER_SERVO_NAME);
        rackNPinionServo = hardwareMap.get(Servo.class, FtcRelay.RACK_N_PINION_SERVO_NAME);
        leftLiftServo = hardwareMap.get(Servo.class, FtcRelay.LEFT_LIFT_SERVO_NAME);
        rightLiftServo = hardwareMap.get(Servo.class, FtcRelay.RIGHT_LIFT_SERVO_NAME);

        spinPower = FtcRelay.SPINNER_STOP_POWER;
        spinServo.setPosition(spinPower);

        rackNPinionPosition = FtcRelay.RACK_N_PINION_SERVO_FORWARD_POSITION;
        rackNPinionServo.setPosition(rackNPinionPosition);

        leftLiftServoPosition = FtcRelay.LEFT_LIFT_SERVO_FORWARD_POSITION;
        rightLiftServoPosition = FtcRelay.RIGHT_LIFT_SERVO_FORWARD_POSITION;
        leftLiftServo.setPosition(leftLiftServoPosition);
        rightLiftServo.setPosition(rightLiftServoPosition);

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
        double position = 0;
        Servo servo = null;

        if (gamepad1.dpad_up) {
            rackNPinionPosition += FtcServo.LARGE_INCREMENT;
            servo = rackNPinionServo;
            position = rackNPinionPosition;
        } else if (gamepad1.dpad_down) {
            rackNPinionPosition -= FtcServo.LARGE_INCREMENT;
            servo = rackNPinionServo;
            position = rackNPinionPosition;
        } else if (gamepad1.dpad_left) {
            spinPower += FtcServo.LARGE_INCREMENT;
            servo = spinServo;
            position = spinPower;
        } else if (gamepad1.dpad_right) {
            spinPower -= FtcServo.LARGE_INCREMENT;
            servo = spinServo;
            position = spinPower;
        } else if (gamepad1.right_trigger > 0.5) {
            leftLiftServoPosition += FtcServo.LARGE_INCREMENT;
            servo = leftLiftServo;
            position = leftLiftServoPosition;
        } else if (gamepad1.right_bumper) {
            leftLiftServoPosition -= FtcServo.LARGE_INCREMENT;
            servo = leftLiftServo;
            position = leftLiftServoPosition;
        }

        if (servo != null) {
            position = Range.clip(position, Servo.MIN_POSITION, Servo.MAX_POSITION);
            servo.setPosition(position);
        }

        telemetry.addData("hand delivery", "dPad up/down");
        telemetry.addData("hand fingers", "dPad left/right");
        telemetry.addData("arm", "right trigger/bumper");
        telemetry.addData("Position", "spin %5.4f rnp %5.4f leftLift %5.4f rightLift %5.4f",
                spinPower, rackNPinionPosition, leftLiftServoPosition, rightLiftServoPosition);
        telemetry.addData(">", "Loop %.0f ms, cumulative %.0f seconds",
                loopTime.milliseconds(), runtime.seconds());
        telemetry.update();
        FtcUtils.sleep(CYCLE_MS);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        FtcLogger.enter();
        telemetry.addData(">", "Tele Op stopped.");
        telemetry.update();
        FtcLogger.exit();
    }
}