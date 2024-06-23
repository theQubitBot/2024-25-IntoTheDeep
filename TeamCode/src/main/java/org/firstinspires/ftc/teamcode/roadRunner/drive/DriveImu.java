package org.firstinspires.ftc.teamcode.roadRunner.drive;

import static org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants.RUN_USING_ENCODER;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * Variables shared between multiple drive types.
 */
public class DriveImu {
    public static IMU imu;

    public static void init(HardwareMap hardwareMap) {
        if (RUN_USING_ENCODER || DriveVariables.use2WheelTrackingLocalizer) {
            // adjust the names of the following hardware devices to match your configuration
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();

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
        }
    }

    public static double getRawExternalHeading() {
        if (DriveConstants.RUN_USING_ENCODER || DriveVariables.use2WheelTrackingLocalizer) {
            YawPitchRollAngles yawPitchRollAngles = imu.getRobotYawPitchRollAngles();
            return yawPitchRollAngles.getYaw(AngleUnit.RADIANS);
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

            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
            return (double) angularVelocity.zRotationRate;
        }

        return 0.0;
    }
}
