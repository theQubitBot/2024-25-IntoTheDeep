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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcMotor;
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

    FtcServo leftSpinServo;
    FtcServo rightSpinServo;
    FtcServo leftFlipServo;
    FtcServo rightFlipServo;
    FtcServo liftServo;
    FtcServo rackNPinionServo;
    double rackNPinionPosition;
    double spinPower;
    double flipPower;
    double liftServoPower;
    FtcMotor armMotor;
    int armPosition;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FtcLogger.enter();
        telemetry.addData(">", "Initializing, please wait...");
        telemetry.update();

        armMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, FtcRelay.ARM_MOTOR_NAME));
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armPosition = FtcRelay.ARM_FORWARD_POSITION;
        FtcRelay.endAutoOpArmPosition = FtcRelay.ARM_MINIMUM_POSITION;

        rackNPinionServo = new FtcServo(hardwareMap.get(Servo.class, FtcRelay.RACK_N_PINION_SERVO_NAME));
        rackNPinionServo.setDirection(Servo.Direction.REVERSE);
        rackNPinionPosition = FtcRelay.RACK_N_PINION_STOP_POWER;
        rackNPinionServo.setPosition(rackNPinionPosition);

        leftFlipServo = new FtcServo(hardwareMap.get(Servo.class, FtcRelay.LEFT_FLIP_SERVO_NAME));
        leftFlipServo.setDirection(Servo.Direction.REVERSE);
        rightFlipServo = new FtcServo(hardwareMap.get(Servo.class, FtcRelay.RIGHT_FLIP_SERVO_NAME));
        rightFlipServo.setDirection(Servo.Direction.REVERSE);

        flipPower = FtcRelay.FLIP_STOP_POWER;
        leftFlipServo.setPosition(flipPower);
        rightFlipServo.setPosition(flipPower);

        leftSpinServo = new FtcServo(hardwareMap.get(Servo.class, FtcRelay.LEFT_SPIN_SERVO_NAME));
        leftSpinServo.setDirection(Servo.Direction.FORWARD);
        rightSpinServo = new FtcServo(hardwareMap.get(Servo.class, FtcRelay.RIGHT_SPIN_SERVO_NAME));
        rightSpinServo.setDirection(Servo.Direction.FORWARD);

        spinPower = FtcRelay.SPIN_STOP_POWER;
        leftSpinServo.setPosition(spinPower);
        rightSpinServo.setPosition(spinPower);

        liftServo = new FtcServo(hardwareMap.get(Servo.class, FtcRelay.LIFT_SERVO_NAME));
        liftServo.setDirection(Servo.Direction.REVERSE);
        liftServoPower = FtcRelay.LIFT_SERVO_STOP_POWER;
        liftServo.setPosition(liftServoPower);

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
        FtcServo servo = null;
        double servoPosition = 0;

        FtcMotor motor = null;
        double motorPower;
        int currentPosition = armMotor.getCurrentPosition();
        int targetPosition = currentPosition;

        if (gamepad1.right_trigger > 0.5) {
            leftSpinServo.setPosition(FtcRelay.SPIN_IN_POWER);
            rightSpinServo.setPosition(FtcRelay.SPIN_IN_POWER);
        } else if (gamepad1.right_bumper) {
            leftSpinServo.setPosition(FtcRelay.SPIN_OUT_POWER);
            rightSpinServo.setPosition(FtcRelay.SPIN_OUT_POWER);
        } else {
            leftSpinServo.setPosition(FtcRelay.SPIN_STOP_POWER);
            rightSpinServo.setPosition(FtcRelay.SPIN_STOP_POWER);
        }

        // have not coded any gamepad functions for leftFlipServo, rightFlipServo, or liftServo

        if (gamepad1.dpad_up) {
            rackNPinionServo.setPosition(FtcRelay.RACK_N_PINION_EXTEND_POWER);
        } else if (gamepad1.dpad_down) {
            rackNPinionServo.setPosition(FtcRelay.RACK_N_PINION_RETRACT_POWER);
        } else {
            rackNPinionServo.setPosition(FtcRelay.RACK_N_PINION_STOP_POWER);
        }

        if (gamepad1.left_trigger > 0.5) {
            targetPosition = currentPosition - 10;
            motor = armMotor;
        } else if (gamepad1.left_bumper) {
            targetPosition = currentPosition + 10;
            motor = armMotor;
        } else {
            targetPosition = currentPosition;
        }

        if (servo != null) {
            servoPosition = Range.clip(servoPosition, Servo.MIN_POSITION, Servo.MAX_POSITION);
            servo.setPosition(servoPosition);
        }

        if (motor != null) {
            targetPosition = Range.clip(targetPosition, -FtcRelay.ARM_BACKWARD_POSITION, FtcRelay.ARM_BACKWARD_POSITION);

            if (targetPosition > currentPosition) {
                motorPower = FtcRelay.ARM_BACKWARD_POWER;
            } else if (targetPosition < currentPosition) {
                motorPower = FtcRelay.ARM_FORWARD_POWER;
            } else {
                motorPower = FtcMotor.ZERO_POWER;
            }

            motor.setTargetPosition(targetPosition);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setPower(motorPower);
        }

        telemetry.addData("spinner", "right trigger/bumper");
        telemetry.addData("RnP", "dPad up/down");
        telemetry.addData("arm", "left trigger/bumper");
        telemetry.addData("Position", "spin %5.4f rnp %5.4f arm %d",
                spinPower, rackNPinionPosition, currentPosition);
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
