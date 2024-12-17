package org.firstinspires.ftc.teamcode.qubit.testOps;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

@Disabled
@Config
@SuppressLint("DefaultLocale")
@Autonomous(group = "Test")
public class MultiplierTunerTest extends LinearOpMode {
    public static double x = 24, y = 0;
    public static double xMultiplier = 1, yMultiplier = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        double xM = 0.7, yM = 1.44;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Vector2d v = new Vector2d(x * xMultiplier, y * yMultiplier);
        TrajectoryActionBuilder tab = drive.actionBuilder(startPose)
                .strafeToConstantHeading(v);
        Action a = tab.build();

        waitForStart();
        Actions.runBlocking(a);
        if (drive.pose.position.x != 0)
            xM = Math.abs(x / drive.pose.position.x);

        if (drive.pose.position.y != 0)
            yM = Math.abs(y / drive.pose.position.y);

        telemetry.addData("Expected", String.format("x %.2f y %.2f", x, y));

        telemetry.addData("RR thinks", String.format("x %.2f y %.2f",
                drive.pose.position.x, drive.pose.position.y));

        telemetry.addData("Multiplier", String.format("x %.2f y %.2f", xM, yM));

        telemetry.update();

        while (opModeIsActive()) {
            Thread.sleep(50);
        }
    }
}
