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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcLift;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcMotor;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

import java.util.Locale;

//@Disabled
@TeleOp(group = "TestOp")
public class FtcLiftCalibrationTeleOp extends OpMode {
    private static final String TAG = "Lift";
    // Declare OpMode members
    private ElapsedTime runtime = null;
    private ElapsedTime loopTime = null;
    public FtcMotor leftLiftMotor = null;
    public FtcMotor rightLiftMotor = null;

    // Start point for the lift
    int currentPosition = FtcLift.POSITION_MINIMUM;
    int targetPosition = FtcLift.POSITION_MINIMUM;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FtcLogger.enter();
        telemetry.addData(">", "Initializing, please wait...");
        telemetry.update();
        leftLiftMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, FtcLift.LEFT_MOTOR_NAME));
        leftLiftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightLiftMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, FtcLift.RIGHT_MOTOR_NAME));
        rightLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FtcLift.endAutoOpLiftPosition = FtcLift.POSITION_MINIMUM;
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
        telemetry.addData(">", "Use DPad up/down to move lift");

        // Lift operation
        currentPosition = leftLiftMotor.getCurrentPosition();
        if (gamepad1.dpad_up) {
            targetPosition++;
        } else if (gamepad1.dpad_down) {
            targetPosition--;
        }

        targetPosition = Range.clip(
                targetPosition, FtcLift.POSITION_LOW, FtcLift.POSITION_HIGH);
        telemetry.addData("targetPosition", "%d", targetPosition);

        if (targetPosition != currentPosition) {
            // Must set motor position before setting motor mode.
            leftLiftMotor.setTargetPosition(targetPosition);
            leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            rightLiftMotor.setTargetPosition(targetPosition);
            rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }

        double liftPower = FtcMotor.ZERO_POWER;
        if (targetPosition > currentPosition) {
            liftPower = FtcLift.UP_POWER;
        } else if (targetPosition < currentPosition) {
            liftPower = FtcLift.DOWN_POWER;
        }

        leftLiftMotor.setPower(liftPower);
        rightLiftMotor.setPower(liftPower);
        telemetry.addData(TAG, String.format(Locale.US, "LLPower %.2f, LLDistance %d, RLPower %.2f, RLDistance %d",
                leftLiftMotor.getPower(), leftLiftMotor.getCurrentPosition(),
                rightLiftMotor.getPower(), rightLiftMotor.getCurrentPosition()));
        telemetry.addData(">", "Loop %.0f ms, cumulative %.0f seconds",
                loopTime.milliseconds(), runtime.seconds());
        telemetry.update();
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
