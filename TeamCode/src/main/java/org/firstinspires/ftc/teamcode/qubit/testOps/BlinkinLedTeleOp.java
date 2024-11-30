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

package org.firstinspires.ftc.teamcode.qubit.testOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.qubit.core.FtcBlinkinLed;
import org.firstinspires.ftc.teamcode.qubit.core.FtcLogger;
import org.firstinspires.ftc.teamcode.qubit.core.FtcUtils;
import org.firstinspires.ftc.teamcode.qubit.core.enumerations.DisplayMode;

import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(group = "TestOp")
public class BlinkinLedTeleOp extends OpMode {
    // Declare OpMode members
    private ElapsedTime runtime = null;
    private ElapsedTime loopTime = null;

    /*
     * Change the pattern every LED_PERIOD seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 3;

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;

    FtcBlinkinLed blinkinLed = null;
    DisplayMode displayMode = DisplayMode.AUTO;

    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FtcLogger.enter();
        blinkinLed = new FtcBlinkinLed(null);
        blinkinLed.init(hardwareMap, telemetry);
        blinkinLed.telemetryEnabled = FtcUtils.DEBUG;
        FtcLogger.exit();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData(FtcUtils.TAG, "Waiting for driver to press play");
        telemetry.update();
        FtcUtils.sleep(10);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        FtcLogger.enter();
        telemetry.addData(FtcUtils.TAG, "Starting...");
        telemetry.update();
        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        FtcLogger.exit();
    }

    @Override
    public void loop() {
        FtcLogger.enter();
        loopTime.reset();
        telemetry.addData(FtcUtils.TAG, "a: Auto, b: Manual");
        telemetry.addLine();
        handleGamepad();

        if (displayMode == DisplayMode.AUTO) {
            doAutoDisplay();
        } else {
            /*
             * MANUAL mode: Pattern already set by game pad handler.
             */
        }

        telemetry.addData(FtcUtils.TAG, "Display: %s", displayMode.toString());
        if (displayMode == DisplayMode.MANUAL) {
            telemetry.addData(FtcUtils.TAG, "Left bumper: previous, Right bumper: next");
        }

        blinkinLed.showTelemetry();
        telemetry.addData(FtcUtils.TAG, "Loop %.0f ms, cumulative %.0f seconds",
                loopTime.milliseconds(), runtime.seconds());
        FtcLogger.exit();
    }

    /*
     * handleGamepad
     *
     * Responds to a gamepad button press.  Demonstrates rate limiting for
     * button presses.  If loop() is called every 10ms and and you don't rate
     * limit, then any given button press may register as multiple button presses,
     * which in this application is problematic.
     *
     * A: Manual mode, Right bumper displays the next pattern, left bumper displays the previous pattern.
     * B: Auto mode, pattern cycles, changing every LED_PERIOD seconds.
     */
    protected void handleGamepad() {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.a) {
            displayMode = DisplayMode.AUTO;
            gamepadRateLimit.reset();
        } else if (gamepad1.b) {
            displayMode = DisplayMode.MANUAL;
            gamepadRateLimit.reset();
        } else if ((displayMode == DisplayMode.MANUAL) && gamepad1.left_bumper) {
            blinkinLed.setPreviousPattern();
            gamepadRateLimit.reset();
        } else if ((displayMode == DisplayMode.MANUAL) && gamepad1.right_bumper) {
            blinkinLed.setNextPattern();
            gamepadRateLimit.reset();
        }
    }

    protected void doAutoDisplay() {
        if (ledCycleDeadline.hasExpired()) {
            blinkinLed.setNextPattern();
            ledCycleDeadline.reset();
        }
    }
}
