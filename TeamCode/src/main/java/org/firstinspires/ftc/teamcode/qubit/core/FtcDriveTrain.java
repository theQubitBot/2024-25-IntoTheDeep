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

package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTrainEnum;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DriveTypeEnum;

import java.util.Arrays;
import java.util.List;

/**
 * A class to manage the robot drive train.
 * Note:  All hardware element names are camel case and have no spaces between words.
 * <p>
 * Motor:  Left  drive motor: "leftFrontMotor"
 * Motor:  Left  drive motor: "leftRearMotor"
 * Motor:  Right drive motor: "rightFrontMotor"
 * Motor:  Right drive motor: "rightRearMotor"
 */
public class FtcDriveTrain extends FtcSubSystem {
    private static final String TAG = "FtcDriveTrain";
    public static final double MAXIMUM_FORWARD_POWER = 0.80;
    public static final double MECANUM_POWER_BOOST_FACTOR = 1.00;
    public static final double MINIMUM_FORWARD_TELE_OP_POWER = 0.25;

    // Ideally, you would find the min power value by incrementing motor power by 0.01
    // and noting the min power at which the robot begins to move/turn.
    // Robot weight distribution would impact rotational inertia, which would impact
    // the turn value most.
    public static final double MAXIMUM_TURN_POWER = 0.80;
    public static final double MINIMUM_TURN_POWER = 0.25;
    public static final double FORWARD_SLO_MO_POWER = 0.25;
    public static final double STRAFE_SLO_MO_POWER = 0.40;

    // JITTER is ideally the minimum motor power to move a wheel when the robot is jacked up.
    // Empirically, the minimum power would be the one to overcome internal friction.
    // So, if the joystick is jittery, we can safely ignore joystick values below this.
    // Must set this to at least 0.01 for trigonometry to work.
    private static final double JITTER = 0.06;

    // Ramp up/down time is used for software assisted anti-lock braking (ABS) or
    // Electronic Stability Control (ESC), or Dynamic Stability Control (DSC).
    // Provides anti skid acceleration, anti-lock braking aka smooth acceleration/deceleration.
    // Smaller values result in tighter brakes. Experiment with your bot to find the right value.
    private static final double POWER_RAMP_UP_DOWN_TIME = 250.0; // milliseconds

    // The amount of proportional (p) feedback to apply for software assisted straight line steering.
    // Larger is more responsive, but less stable. Typical values lie in [0.01, 0.10] interval.
    static final double KP_DRIVE = 0.01;
    public DcMotorEx.ZeroPowerBehavior ZeroPowerBehavior = DcMotorEx.ZeroPowerBehavior.FLOAT;
    private final boolean driveTrainEnabled = true;
    public boolean telemetryEnabled = true;

    // Precision driving mode is driver controlled. Disabled at start.
    private boolean precisionDriveMode = false;
    private Telemetry telemetry = null;
    private final FtcBot parent;

    private FtcMotor leftFrontMotor = null;
    private FtcMotor leftRearMotor = null;
    private FtcMotor rightFrontMotor = null;
    private FtcMotor rightRearMotor = null;
    public List<FtcMotor> frontMotors = null, rearMotors = null;
    public List<FtcMotor> allMotors = null, activeMotors = null;

    // These default drive train and drive type are overridden in FtcBot.init()
    public DriveTrainEnum driveTrainEnum = DriveTrainEnum.UNKNOWN;
    public DriveTypeEnum driveTypeEnum = DriveTypeEnum.UNKNOWN;

    // This data is used for heading correction for an unbalanced robot.
    private double lastTheta = 0, lastHeading = 0;
    private final boolean useMotorEncoders = false;
    private final boolean enableUnbalancedRobotHeadingCorrection = false;
    private final boolean enableMecanumPowerBoost = true;
    private final boolean useLiftPositionForSpeedAdjustment = true;

    public FtcDriveTrain(FtcBot robot) {
        parent = robot;
    }

    /**
     * Evaluates if unbalanced robot heading should be automatically corrected
     * by the software.
     *
     * @return True if feature is enabled, false otherwise
     */
    private boolean AutomaticallyCorrectUnbalancedRobotHeading(Gamepad gamepad1, Gamepad gamepad2) {
        // Correction is off if list is higher than lower basket, on otherwise.
        boolean enableCorrectionForLowLiftPosition = parent == null || parent.lift == null ||
                parent.lift.getLeftPosition() < FtcLift.POSITION_LOW_BASKET;

        // Grant wants correction to be off when samples are being intook.
        boolean enableCorrectionForIntake = !(gamepad1.right_trigger >= 0.5) && !(gamepad2.right_trigger >= 0.5);

        return enableUnbalancedRobotHeadingCorrection && enableCorrectionForLowLiftPosition &&
                enableCorrectionForIntake;
    }

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @param telemetry   The telemetry to use.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;

        if (driveTrainEnabled) {
            if (driveTrainEnum == DriveTrainEnum.FRONT_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE) {
                leftFrontMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, "leftFrontMotor"));
                rightFrontMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, "rightFrontMotor"));
                leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                frontMotors = Arrays.asList(leftFrontMotor, rightFrontMotor);
                activeMotors = frontMotors;
            }

            if (driveTrainEnum == DriveTrainEnum.REAR_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE) {
                leftRearMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, "leftRearMotor"));
                rightRearMotor = new FtcMotor(hardwareMap.get(DcMotorEx.class, "rightRearMotor"));
                leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                rearMotors = Arrays.asList(leftRearMotor, rightRearMotor);
                activeMotors = rearMotors;
            }

            if (driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE) {
                allMotors = Arrays.asList(
                        leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
                activeMotors = allMotors;
            }

            // Reset motor encoders
            for (FtcMotor motor : activeMotors) {
                if (useMotorEncoders) {
                    // Resetting encoders is important. If the RC is not reset, encoder counts keep
                    // going up/down as team practices on the robot.
                    motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                    // After STOP_AND_RESET_ENCODER, must specify the runMode
                    // Otherwise motor power may stay off indefinitely.
                    motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                } else {
                    motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

            showTelemetry();
            telemetry.addData(TAG, "initialized");

        } else {
            telemetry.addData(TAG, "not enabled");
        }

        FtcLogger.exit();
    }

    /**
     * Strafing is meaningful only for MECANUM Drive.
     * Robot is considered strafing if it moves in the 90 degree cones
     * perpendicular to its forward direction.
     * <p>
     * \     |      /
     * \   |   /
     * strafing \ | /  strafing
     * -----------------------------------
     * strafing / | \  strafing
     * /   |   \
     * /     |     \
     *
     * @param robotHeading    Robot heading as per the Gyro (Degrees)
     * @param joystickHeading Joystick heading (Degrees)
     * @return True, if the robot is strafing.
     */
    private boolean isBotStrafing(double robotHeading, double joystickHeading) {
        double rH = FtcImu.normalize(robotHeading, AngleUnit.DEGREES);
        double jH = FtcImu.normalize(joystickHeading, AngleUnit.DEGREES);
        double delta = Math.abs(rH - jH);

        // Robot's zero heading is forward, joy stick's zero heading is towards right.
        boolean strafing = !(delta >= 45 && delta <= 135);
        if (FtcUtils.DEBUG && driveTrainEnabled && telemetryEnabled) {
            telemetry.addData("Strafing data", "rH %.2f jH %.2f strafing %b", +
                    rH, jH, strafing);
        }

        return strafing;
    }

    /**
     * Field Oriented Drive is a method of driving robot from the outside operator's perspective.
     * It moves the robot maintaining robot heading relative to the field.
     * Forward, reverse, turn, strafe are all relative to the field and are independent
     * of the direction of the robot.
     * <p>
     * Point Of View Drive is a method of driving robot from the robot's perspective.
     * This is similar to a person driving a vehicle in real world.
     * Forward, reverse, turn, strafe are all relative to the direction of the robot.
     *
     * @param gamePad1 The first gamePad to use for driving.
     * @param gamePad2 The second gamePad to use for driving.
     * @param loopTime The loopTime passed in by TeleOp that determines how fast the TeleOp loop
     *                 is executing. Shorter the loopTime, shorter the anti-skid braking power step.
     */
    public void operate(Gamepad gamePad1, Gamepad gamePad2, double loopTime, ElapsedTime runtime) {
        FtcLogger.enter();

        // Setup a variable for each side drive wheel to display power level for telemetry
        double leftFrontPower = FtcMotor.ZERO_POWER;
        double leftRearPower = FtcMotor.ZERO_POWER;
        double rightFrontPower = FtcMotor.ZERO_POWER;
        double rightRearPower = FtcMotor.ZERO_POWER;

        // Use left stick for forward/backward/strafe, use right stick for turn
        double y = -gamePad1.left_stick_y;
        double x = gamePad1.left_stick_x;
        double yMagnitude = Math.abs(y);
        double xMagnitude = Math.abs(x);
        double powerMagnitude = Math.max(yMagnitude, xMagnitude);
        double maxWheelPower;
        double joyStickHeading = 0;

        if (!FtcUtils.DEBUG && FtcUtils.gameOver(runtime)) {
            stop();
            return;
        }

        parent.imu.resetReadOnce();
        if (powerMagnitude > JITTER) {
            if (driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE) {
                // All angles are in radians, except for Gyro reading
                parent.imu.readOnce();
                joyStickHeading = Math.atan2(y, x);
                double finalBotHeading = joyStickHeading;
                if (driveTypeEnum == DriveTypeEnum.FIELD_ORIENTED_DRIVE) {
                    // Account for current Heading and Auto Op end heading.
                    finalBotHeading -= Math.toRadians(parent.imu.getHeading() + FtcImu.endAutoOpHeading);
                }

                // Calculate  formulaic wheel power.
                leftFrontPower = rightRearPower = Math.sin(finalBotHeading + Math.PI / 4.0) * powerMagnitude;
                rightFrontPower = leftRearPower = Math.sin(finalBotHeading - Math.PI / 4.0) * powerMagnitude;

                if (enableMecanumPowerBoost) {
                    // Boost power up to the powerMagnitude
                    maxWheelPower = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
                    maxWheelPower = Math.max(maxWheelPower, Math.abs(rightFrontPower));
                    maxWheelPower = Math.max(maxWheelPower, Math.abs(rightRearPower));
                    if (maxWheelPower < powerMagnitude) {
                        powerMagnitude *= MECANUM_POWER_BOOST_FACTOR;
                        leftFrontPower = powerMagnitude * leftFrontPower / maxWheelPower;
                        leftRearPower = powerMagnitude * leftRearPower / maxWheelPower;
                        rightFrontPower = powerMagnitude * rightFrontPower / maxWheelPower;
                        rightRearPower = powerMagnitude * rightRearPower / maxWheelPower;
                    }
                }
            } else if (yMagnitude > JITTER) {
                // With non holonomic drive, all wheels have the same power.
                leftFrontPower = leftRearPower = rightFrontPower = rightRearPower =
                        (MINIMUM_FORWARD_TELE_OP_POWER +
                                yMagnitude * (MAXIMUM_FORWARD_POWER - MINIMUM_FORWARD_TELE_OP_POWER))
                                * Math.signum(y);
            } else {
                // Nothing to do. Robot is stationary.
            }
        }

        double turn = gamePad1.right_stick_x;
        double turnMagnitude = Math.abs(turn);
        if (turnMagnitude <= JITTER) {
            turn = FtcMotor.ZERO_POWER;
            if (AutomaticallyCorrectUnbalancedRobotHeading(gamePad1, gamePad2)) {
                // Correction for left/right pull for an unbalanced robot.
                // Apply correction only if driver is not explicitly turning the robot.
                // Tangent comparison will maintain orientation when driver changes power
                // proportionally to slow-down/speed-up AND wants to maintain orientation.
                // Theta is the direction in which the driver wants to go.
                double theta = Math.atan2(y, x);
                if (powerMagnitude > JITTER && FtcUtils.areEqual(lastTheta, theta, FtcUtils.EPSILON2)) {
                    // Driver is changing power only, not orientation
                    // maintain last heading by turning the robot slightly, if needed.
                    parent.imu.readOnce();
                    double headingOffset = parent.imu.getHeadingOffset(lastHeading);
                    turn = parent.imu.getSteeringCorrection(headingOffset, KP_DRIVE);
                } else {
                    // Driver is changing robot orientation explicitly.
                    // Store orientation only, not the power or Gyro Heading.
                    lastTheta = theta;
                }
            }
        } else {
            int turnDirection = (int) Math.signum(turn);
            turn = MINIMUM_TURN_POWER + (turnMagnitude * (MAXIMUM_TURN_POWER - MINIMUM_TURN_POWER));
            turn *= turnDirection;
            if (AutomaticallyCorrectUnbalancedRobotHeading(gamePad1, gamePad2)) {
                // Driver is changing robot orientation explicitly.
                // Store new heading
                parent.imu.readOnce();
                lastHeading = parent.imu.getHeading();
            }
        }

        // Add turn power to drive power
        leftFrontPower += turn;
        leftRearPower += turn;
        rightFrontPower -= turn;
        rightRearPower -= turn;

        // Constrain powers in the range of [-MAXIMUM_FORWARD_POWER,MAXIMUM_FORWARD_POWER]
        // Not checking may cause the robot to drive at full speed
        maxWheelPower = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
        maxWheelPower = Math.max(maxWheelPower, Math.abs(rightFrontPower));
        maxWheelPower = Math.max(maxWheelPower, Math.abs(rightRearPower));

        if (maxWheelPower > MAXIMUM_FORWARD_POWER) {
            // Normalize drive power by dividing everything by max power.
            // It's positive so we don't need to worry about sign.
            leftFrontPower /= maxWheelPower;
            leftRearPower /= maxWheelPower;
            rightFrontPower /= maxWheelPower;
            rightRearPower /= maxWheelPower;
        }

        // Handle precision drive mode
        if (gamePad1.dpad_left) {
            // set global precision drive variable
            precisionDriveMode = true;
        } else if (gamePad1.dpad_right) {
            // set global precision drive variable
            precisionDriveMode = false;
        } else {
            // Maintain last precision drive mode.
        }

        if (precisionDriveMode) {
            // E.g. Robot is turning with minimum power
            // Precision mode should not make it turn using lower than minimum power
            maxWheelPower = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
            maxWheelPower = Math.max(maxWheelPower, Math.abs(rightFrontPower));
            maxWheelPower = Math.max(maxWheelPower, Math.abs(rightRearPower));

            if (maxWheelPower > JITTER) {
                // No matter the joystick magnitude, the max power will be
                // FORWARD_SLO_MO_POWER or STRAFE_SLO_MO_POWER for at least one wheel.
                double crawlFactor;
                parent.imu.readOnce();
                if (driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE &&
                        isBotStrafing(parent.imu.getHeading(), Math.toDegrees(joyStickHeading))) {
                    crawlFactor = maxWheelPower / STRAFE_SLO_MO_POWER;
                } else {
                    crawlFactor = maxWheelPower / FORWARD_SLO_MO_POWER;
                }

                leftFrontPower /= crawlFactor;
                leftRearPower /= crawlFactor;
                rightFrontPower /= crawlFactor;
                rightRearPower /= crawlFactor;
            }
        } else if (useLiftPositionForSpeedAdjustment && parent != null && parent.lift != null) {
            int liftPosition = Math.max(parent.lift.getLeftPosition(), parent.lift.getRightPosition());
            liftPosition = Range.clip(liftPosition, FtcLift.POSITION_MINIMUM, FtcLift.POSITION_HIGH_BASKET);

            // Scale reduces as lift rises
            double scaleFactor = (FtcLift.POSITION_HIGH_BASKET - liftPosition) *
                    (MAXIMUM_FORWARD_POWER - MINIMUM_FORWARD_TELE_OP_POWER) /
                    (FtcLift.POSITION_HIGH_BASKET - FtcLift.POSITION_FLOOR);

            // Scale can't be outside the wheel power limits.
            scaleFactor = Range.clip(scaleFactor, MINIMUM_FORWARD_TELE_OP_POWER, MAXIMUM_FORWARD_POWER);

            // Power factor goes down with scale, but we need minimum power to strafe.
            scaleFactor = STRAFE_SLO_MO_POWER + scaleFactor;

            // Scale power for all wheels
            leftFrontPower *= scaleFactor;
            leftRearPower *= scaleFactor;
            rightFrontPower *= scaleFactor;
            rightRearPower *= scaleFactor;
        }

        setDrivePowerSmooth(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower, loopTime);
        FtcLogger.exit();
    }

    /**
     * Send given power to respective wheels.
     */
    public void setDrivePower(double leftFrontPower, double leftRearPower,
                              double rightFrontPower, double rightRearPower) {
        FtcLogger.enter();
        if (driveTrainEnabled) {
            if (driveTrainEnum == DriveTrainEnum.FRONT_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE) {
                leftFrontMotor.setPower(leftFrontPower);
                rightFrontMotor.setPower(rightFrontPower);
            }

            if (driveTrainEnum == DriveTrainEnum.REAR_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE) {
                leftRearMotor.setPower(leftRearPower);
                rightRearMotor.setPower(rightRearPower);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Increase/decrease motor power incrementally for a smooth acceleration/deceleration.
     * Must call the method repeatedly (e.g. via tele Op).
     *
     * @param leftFrontPower  Left front motor power
     * @param leftRearPower   Left rear motor power
     * @param rightFrontPower Right front motor power
     * @param rightRearPower  Right rear motor power
     * @param loopTime        The loopTime passed in by TeleOp that determines how fast the TeleOp
     *                        loop is executing. Shorter the loopTime, smaller the braking power step.
     */
    private void setDrivePowerSmooth(double leftFrontPower, double leftRearPower,
                                     double rightFrontPower, double rightRearPower,
                                     double loopTime) {
        if (driveTrainEnum == DriveTrainEnum.FRONT_WHEEL_DRIVE ||
                driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE ||
                driveTrainEnum == DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE) {
            setDrivePowerSmooth(leftFrontMotor, leftFrontPower, loopTime);
            setDrivePowerSmooth(rightFrontMotor, rightFrontPower, loopTime);
        }

        if (driveTrainEnum == DriveTrainEnum.REAR_WHEEL_DRIVE ||
                driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE ||
                driveTrainEnum == DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE) {
            setDrivePowerSmooth(leftRearMotor, leftRearPower, loopTime);
            setDrivePowerSmooth(rightRearMotor, rightRearPower, loopTime);
        }
    }

    /**
     * Increases or decreases the power incrementally for a smooth acceleration/deceleration.
     * Must call the method repeatedly (e.g. via tele Op).
     *
     * @param motor    The motor to set the power to.
     * @param power    The power to set to the motor to.
     * @param loopTime The loopTime passed in by TeleOp that determines how fast the TeleOp loop
     *                 is executing. Shorter the loopTime, shorter the braking power step.
     */
    private void setDrivePowerSmooth(FtcMotor motor, double power, double loopTime) {
        FtcLogger.enter();
        if (driveTrainEnabled && motor != null) {
            // loopTime may not have been initialized or the bot is bare bone during development.
            // Minimum time is 1ms. A 0 value may lead to mo power increment!
            double loopTimeMs = Math.max(1, loopTime);

            // Power step needs to be directly proportional to the loop time.
            // Shorter loop time => use smaller power step, experiment and adjust
            // POWER_RAMP_UP_DOWN_TIME as needed. E.g.: If tele Op loop time is 5ms,
            // then it would require 5 * 1.0 / 200 = .025 power step increment per cycle
            // to go from stationary (0) to full (1) speed in 200ms.
            double powerStep = loopTimeMs * MAXIMUM_FORWARD_POWER / POWER_RAMP_UP_DOWN_TIME;
            double currentPower = motor.getPower();
            double delta = power - currentPower;
            double newPower;
            if (Math.abs(delta) >= powerStep)
                newPower = currentPower + powerStep * Math.signum(delta);
            else
                newPower = power;

            motor.setPower(newPower);
        }

        FtcLogger.exit();
    }

    public void setDriveTypeAndMode(DriveTrainEnum driveTrainEnum, DriveTypeEnum driveTypeEnum) {
        this.driveTrainEnum = driveTrainEnum;
        this.driveTypeEnum = driveTypeEnum;
    }

    /**
     * Sets the zero power behavior for the drive motors.
     *
     * @param zeroPowerBehavior The zero power behavior (brake or roll).
     */
    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        FtcLogger.enter();
        if (driveTrainEnabled) {
            ZeroPowerBehavior = zeroPowerBehavior;
            for (FtcMotor motor : activeMotors) {
                motor.setZeroPowerBehavior(ZeroPowerBehavior);
            }
        }

        FtcLogger.exit();
    }

    /**
     * Display driveTrain information. Helps with debugging.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (driveTrainEnabled && telemetryEnabled) {
            if (driveTrainEnum == DriveTrainEnum.FRONT_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE) {
                telemetry.addData("Left Front Motor", "power %.2f distance %d",
                        leftFrontMotor.getPower(), leftFrontMotor.getCurrentPosition());
                telemetry.addData("Right Front Motor", "power %.2f distance %d",
                        rightFrontMotor.getPower(), rightFrontMotor.getCurrentPosition());
            }

            if (driveTrainEnum == DriveTrainEnum.REAR_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.MECANUM_WHEEL_DRIVE ||
                    driveTrainEnum == DriveTrainEnum.TRACTION_OMNI_WHEEL_DRIVE) {
                telemetry.addData("Left Rear Motor", "power %.2f distance %d",
                        leftRearMotor.getPower(), leftRearMotor.getCurrentPosition());
                telemetry.addData("Right Rear Motor", "power %.2f distance %d",
                        rightRearMotor.getPower(), rightRearMotor.getCurrentPosition());
            }

            telemetry.addData(FtcUtils.TAG, "%s, %s", driveTrainEnum.name(),
                    driveTypeEnum.name());
        }

        FtcLogger.exit();
    }

    /**
     * Stops the robot by sending zero power to all drive motors.
     */
    public void stop() {
        FtcLogger.enter();
        setDrivePower(FtcMotor.ZERO_POWER, FtcMotor.ZERO_POWER, FtcMotor.ZERO_POWER, FtcMotor.ZERO_POWER);
        FtcLogger.exit();
    }
}
