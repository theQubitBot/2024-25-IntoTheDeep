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

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

public class CsvWriter {
    private static final String TAG = "CsvWriter";
    private static final String COMMA = ",";
    private Writer fileWriter;
    private String line;

    public CsvWriter(String filename) {
        try {
            fileWriter = new FileWriter(
                    AppUtil.ROBOT_DATA_DIR + File.separator + filename);
            Assert.assertNotNull(fileWriter, "CsvWriter>fileWriter");
            line = "";
        } catch (IOException e) {
            RobotLog.ee(TAG, e, e.getMessage());
        }
    }

    public void close() {
        FtcLogger.enter();
        try {
            if (fileWriter != null) {
                fileWriter.close();
                fileWriter = null;
            }
        } catch (IOException e) {
            RobotLog.ee(TAG, e, e.getMessage());
        }

        FtcLogger.exit();
    }

    public void flush() {
        FtcLogger.enter();
        Assert.assertNotNull(fileWriter, "flush>fileWriter");
        try {
            fileWriter.write(line + System.lineSeparator());
            line = "";
        } catch (IOException e) {
            RobotLog.ee(TAG, e, e.getMessage());
        }

        FtcLogger.exit();
    }

    public void append(String data) {
        FtcLogger.enter();
        if (!line.equals("")) line += COMMA;
        line += data;
        FtcLogger.exit();
    }

    public void append(Object data) {
        append(data.toString());
    }

    public void append(boolean data) {
        append(String.valueOf(data));
    }

    public void append(byte data) {
        append(String.valueOf(data));
    }

    public void append(char data) {
        append(String.valueOf(data));
    }

    public void append(short data) {
        append(String.valueOf(data));
    }

    public void append(int data) {
        append(String.valueOf(data));
    }

    public void append(long data) {
        append(String.valueOf(data));
    }

    public void append(float data) {
        append(String.valueOf(data));
    }

    public void append(double data) {
        append(String.valueOf(data));
    }
}
