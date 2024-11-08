package org.firstinspires.ftc.teamcode.qubit.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.goBilda.GoBoDriver;

import java.util.concurrent.TimeUnit;

public class FtcGoBoDriver extends FtcSubSystem {
    private static final String TAG = "FtcGoBoDriver";
    public boolean telemetryEnabled = true;
    Telemetry telemetry = null;
    private GoBoDriver goboDriver;
    private Deadline deadline;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FtcLogger.enter();
        this.telemetry = telemetry;
        goboDriver = hardwareMap.get(GoBoDriver.class, "goboDriver");
        if (goboDriver != null) {
            goboDriver.setEncoderResolution(GoBoDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            goboDriver.setEncoderDirections(GoBoDriver.EncoderDirection.FORWARD,
                    GoBoDriver.EncoderDirection.FORWARD);
            goboDriver.resetPosAndIMU();
            goboDriver.bulkUpdate();
            deadline = new Deadline(1, TimeUnit.MILLISECONDS);
        }

        FtcLogger.exit();
    }

    public int getEncoderX() {
        refreshData();
        return goboDriver.getEncoderX();
    }

    public int getEncoderY() {
        refreshData();
        return goboDriver.getEncoderY();
    }

    /**
     * @return Heading in requested units.
     */
    public double getHeading(AngleUnit angleUnit) {
        refreshData();
        if (angleUnit == AngleUnit.DEGREES) {
            return FtcImu.normalize(Math.toDegrees(goboDriver.getHeading()), AngleUnit.DEGREES);
        } else {
            return FtcImu.normalize(goboDriver.getHeading(), AngleUnit.RADIANS);
        }
    }

    /**
     * @return Heading velocity in requested units / second
     */
    public double getHeadingVelocity(AngleUnit angleUnit) {
        refreshData();
        if (angleUnit == AngleUnit.DEGREES) {
            return Math.toDegrees(goboDriver.getHeadingVelocity());
        } else {
            return goboDriver.getHeadingVelocity();
        }
    }

    /**
     * @return the estimated X (forward) position of the robot in requested units
     */
    public double getPositionX(DistanceUnit distanceUnit) {
        refreshData();
        if (distanceUnit == DistanceUnit.MM) {
            return goboDriver.getPosX();
        } else {
            return DistanceUnit.MM.toInches(goboDriver.getPosX());
        }
    }

    /**
     * @return the estimated Y (Strafe) position of the robot in requested units
     */
    public double getPositionY(DistanceUnit distanceUnit) {
        refreshData();
        if (distanceUnit == DistanceUnit.MM) {
            return goboDriver.getPosY();
        } else {
            return DistanceUnit.MM.toInches(goboDriver.getPosY());
        }
    }

    /**
     * @return the estimated X (forward) velocity of the robot in requested units/sec
     */
    public double getVelocityX(DistanceUnit distanceUnit) {
        refreshData();
        if (distanceUnit == DistanceUnit.MM) {
            return goboDriver.getVelX();
        } else {
            return DistanceUnit.MM.toInches(goboDriver.getVelX());
        }
    }

    /**
     * @return the estimated Y (strafe) velocity of the robot in requested units/sec
     */
    public double getVelocityY(DistanceUnit distanceUnit) {
        refreshData();
        if (distanceUnit == DistanceUnit.MM) {
            return goboDriver.getVelY();
        } else {
            return DistanceUnit.MM.toInches(goboDriver.getVelY());
        }
    }

    public void refreshData() {
        if (deadline.hasExpired()) {
            goboDriver.bulkUpdate();
            deadline.reset();
        }
    }

    /**
     * Display IMU (gyro) telemetry.
     */
    public void showTelemetry() {
        FtcLogger.enter();
        if (telemetryEnabled) {
            telemetry.addData("Heading/Yaw", "%.2f", getHeading(AngleUnit.DEGREES));
        }

        FtcLogger.exit();
    }
}
