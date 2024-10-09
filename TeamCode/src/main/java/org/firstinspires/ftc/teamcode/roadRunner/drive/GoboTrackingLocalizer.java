package org.firstinspires.ftc.teamcode.roadRunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.qubit.core.FtcGoBoDriver;

import java.util.Arrays;
import java.util.List;

@Config
public class GoboTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2000.0; // GoBilda odometer POD
    public static double WHEEL_RADIUS = (32.0 / 2.0) / 25.4; // in; goBilda odometry pod Omni wheels
    public static double GEAR_RATIO = 1.0; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -3.0; // X is the up and down direction
    public static double PARALLEL_Y = 1.875; // Y is the strafe direction

    public static double PERPENDICULAR_X = -3.0;
    public static double PERPENDICULAR_Y = -1.875;

    // Execute StraightTest (72") to get rough X_MULTIPLIER, then experiment to nail it down.
    // X_MULTIPLIER = measured via tape / Displayed by telemetry
    public static double X_MULTIPLIER = 96.0 / 96.7;

    // Tune SampleMecanumDrive.LATERAL_MULTIPLIER to get strafing right
    // Leave Y_MULTIPLIER at 1.0
    public static double Y_MULTIPLIER = 96.0 / 100.3; // Multiplier in the Y direction

    private final FtcGoBoDriver ftcGoBoDriver;
    int startEncoderXPosition = 0, startEncoderYPosition = 0;

    public GoboTrackingLocalizer(HardwareMap hardwareMap, FtcGoBoDriver ftcGoBoDriver) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.ftcGoBoDriver = ftcGoBoDriver;
        startEncoderXPosition = ftcGoBoDriver.getEncoderX();
        startEncoderYPosition = ftcGoBoDriver.getEncoderY();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return ftcGoBoDriver.getHeading(AngleUnit.RADIANS);
    }

    @Override
    public Double getHeadingVelocity() {
        return ftcGoBoDriver.getHeadingVelocity(AngleUnit.RADIANS);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(ftcGoBoDriver.getEncoderX() - startEncoderXPosition) * X_MULTIPLIER,
                encoderTicksToInches(ftcGoBoDriver.getEncoderY() - startEncoderYPosition) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                (ftcGoBoDriver.getVelocityX(DistanceUnit.INCH)) * X_MULTIPLIER,
                (ftcGoBoDriver.getVelocityY(DistanceUnit.INCH)) * Y_MULTIPLIER
        );
    }
}
