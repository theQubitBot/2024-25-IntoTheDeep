package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

//@Disabled
@Config
@TeleOp(group = "Autonomous Pathing Tuning")
public class StrafeTest extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 36;

    private boolean goLeft;

    private Follower follower;

    private Path leftPath;
    private Path rightPath;

    @Override
    public void init() {
        Pose startPose = new Pose(0, 0, 0);
        Pose leftPosition = new Pose(0, DISTANCE, 0);

        leftPath = new Path(new BezierLine(new Point(startPose), new Point(leftPosition)));
        leftPath.setConstantHeadingInterpolation(0);
        rightPath = new Path(new BezierLine(new Point(leftPosition), new Point(startPose)));
        rightPath.setConstantHeadingInterpolation(0);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        follower.followPath(leftPath, true);
        goLeft = true;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches left. The robot will go left and right continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy()) {
            if (gamepad1.b) {
                follower.followPath(rightPath, true);
                goLeft = false;
            } else if (gamepad1.x) {
                follower.followPath(leftPath, true);
                goLeft = true;
            }
        }

        telemetryA.addData("going Left", goLeft);
        follower.telemetryDebug(telemetryA);
    }
}
