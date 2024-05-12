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

package org.firstinspires.ftc.teamcode.qubit.core;

/**
 * A class to asynchronously operate the arm.
 */
public class FtcArmAsyncExecutor implements Runnable {
    public enum ArmOperation {
        ReceiveOnly,
        DeliverOnly,
        DeliverAndReceive,
        NoOp,
        Stop
    }

    public static final String TAG = "FtcArmAsyncExecutor";
    private final FtcArm arm;
    private final Object operationLock;
    private ArmOperation armOperation;

    public FtcArmAsyncExecutor(FtcArm arm) {
        FtcLogger.enter();
        this.arm = arm;
        operationLock = new Object();
        armOperation = ArmOperation.NoOp;
        FtcLogger.exit();
    }

    /**
     * A threadsafe method to set the arm operation.
     *
     * @param armOperation       The arm operation to perform.
     * @param waitTillCompletion If true, method blocks till the operation is complete.
     */
    public void setOperation(ArmOperation armOperation, boolean waitTillCompletion) {
        synchronized (operationLock) {
            this.armOperation = armOperation;
        }

        if (waitTillCompletion) {
            do {
                FtcUtils.sleep(5);
                synchronized (operationLock) {
                    armOperation = this.armOperation;
                }

            } while (armOperation != ArmOperation.NoOp);
        }
    }

    /**
     * Background thread method to execute the requested arm operations.
     * Once the requested operation is complete, the next operation is set to NoOp.
     */
    public void run() {
        ArmOperation requestedOperation;
        while (true) {
            synchronized (operationLock) {
                requestedOperation = armOperation;
            }

            if (requestedOperation == ArmOperation.DeliverOnly) {
                if (!arm.armInPosition(FtcArm.ARM_POSITION_DELIVERY_FLUSH)) {
                    arm.move(FtcArm.ARM_POSITION_DELIVERY_FLUSH, true);
                }

                synchronized (operationLock) {
                    armOperation = ArmOperation.NoOp;
                }
            } else if (requestedOperation == ArmOperation.DeliverAndReceive) {
                if (!arm.armInPosition(FtcArm.ARM_POSITION_DELIVERY_FLUSH)) {
                    arm.move(FtcArm.ARM_POSITION_DELIVERY_FLUSH, true);
                    arm.move(FtcArm.ARM_POSITION_RECEIVE, true);
                }

                synchronized (operationLock) {
                    armOperation = ArmOperation.NoOp;
                }
            } else if (requestedOperation == ArmOperation.ReceiveOnly) {
                if (!arm.armInPosition(FtcArm.ARM_POSITION_RECEIVE)) {
                    arm.move(FtcArm.ARM_POSITION_RECEIVE, true);
                }

                synchronized (operationLock) {
                    armOperation = ArmOperation.NoOp;
                }
            } else if (requestedOperation == ArmOperation.NoOp) {
                FtcUtils.sleep(5);
            } else if (requestedOperation == ArmOperation.Stop) {
                break;
            }
        }

        FtcLogger.info(TAG, "Async thread stopped.");
    }
}


