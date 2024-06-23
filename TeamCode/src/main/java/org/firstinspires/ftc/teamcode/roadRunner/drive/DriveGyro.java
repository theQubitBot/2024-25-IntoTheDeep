package org.firstinspires.ftc.teamcode.roadRunner.drive;

import static org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants.RUN_USING_ENCODER;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.roadRunner.util.AxisDirection;
import org.firstinspires.ftc.teamcode.roadRunner.util.BNO055IMUUtil;

/*
 * Variables shared between multiple drive types.
 */
public class DriveGyro {
    public static BNO055IMU imu;

    public static void init(HardwareMap hardwareMap) {
        if (RUN_USING_ENCODER || DriveVariables.use2WheelTrackingLocalizer) {
            // adjust the names of the following hardware devices to match your configuration
            imu = hardwareMap.get(BNO055IMU.class, "adafruitImu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

            // Disable logging to improve performance.
            parameters.loggingEnabled = false;
            imu.initialize(parameters);
            while (!imu.isGyroCalibrated()) {
                // Wait till gyro is fully calibrated.
                FtcUtils.sleep(100);
            }

            // If the hub containing the IMU you are using is mounted so that the "REV" logo does
            // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
            //
            //             | +Z axis
            //             |
            //             |
            //             |
            //      _______|_____________     +Y axis
            //     /       |_____________/|__________
            //    /   REV / EXPANSION   //
            //   /       / HUB         //
            //  /_______/_____________//
            // |_______/_____________|/
            //        /
            //       / +X axis
            //
            // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
            // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
            //
            // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
            // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);
            BNO055IMUUtil.remapZAxis(
                    imu, AxisDirection.POS_Z);
        }
    }

    public static double getRawExternalHeading() {
        if (DriveConstants.RUN_USING_ENCODER || DriveVariables.use2WheelTrackingLocalizer) {
            return imu.getAngularOrientation().firstAngle;
        } else {
            return 0;
        }
    }

    public static Double getExternalHeadingVelocity() {
        if (DriveConstants.RUN_USING_ENCODER || DriveVariables.use2WheelTrackingLocalizer) {
            // This must be changed to match your configuration
            //                           | Z axis
            //                           |
            //     (Motor Port Side)     |   / X axis
            //                       ____|__/____
            //          Y axis     / *   | /    /|   (IO Side)
            //          _________ /______|/    //      I2C
            //                   /___________ //     Digital
            //                  |____________|/      Analog
            //
            //                 (Servo Port Side)
            //
            // The positive x axis points toward the USB port(s)
            //
            // Adjust the axis rotation rate as necessary
            // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
            // flat on a surface

            return (double) imu.getAngularVelocity().zRotationRate;
        }

        return 0.0;
    }
}
