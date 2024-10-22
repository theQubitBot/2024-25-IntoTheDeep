package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.qubit.core.FtcGoBoDriver;
import org.firstinspires.ftc.teamcode.qubit.core.FtcImu;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;

import java.util.Locale;

@Disabled
@TeleOp(group = "TestOp")
public class GoBoDriverTeleOp extends OpMode {
    // Declare OpMode members
    private ElapsedTime runtime = null;
    private ElapsedTime loopTime = null;
    FtcGoBoDriver ftcGoBoDriver = null;
    double oldTime = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FtcLogger.enter();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        telemetry.addData(">", "Initializing, please wait...");
        ftcGoBoDriver = new FtcGoBoDriver();
        ftcGoBoDriver.init(hardwareMap, telemetry);
        FtcImu.endAutoOpHeading = 0;
        telemetry.update();
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
        loopTime.reset();

        String position = String.format(Locale.US, "X: %.3f, Y: %.3f, H: %.3f",
                ftcGoBoDriver.getPositionX(DistanceUnit.INCH),
                ftcGoBoDriver.getPositionY(DistanceUnit.INCH),
                ftcGoBoDriver.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", position);

        String velocity = String.format(Locale.US, "XVel: %.3f, YVel: %.3f, HVel: %.3f",
                ftcGoBoDriver.getVelocityX(DistanceUnit.INCH),
                ftcGoBoDriver.getVelocityY(DistanceUnit.INCH),
                ftcGoBoDriver.getHeadingVelocity(AngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        telemetry.addData("Encoders", String.format(Locale.US, "X: %d, Y: %d",
                ftcGoBoDriver.getEncoderX(), ftcGoBoDriver.getEncoderY()));

        telemetry.addData(">", "Loop %.0f ms, cumulative %.0f seconds",
                loopTime.milliseconds(), runtime.seconds());
        telemetry.update();
        FtcLogger.exit();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        FtcLogger.enter();
        this.telemetry.addData(">", "Tele Op stopped.");
        this.telemetry.update();
        FtcLogger.exit();
    }
}
