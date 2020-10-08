package org.exponential.superclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class UnitTester extends LinearOpMode {
    public void trackIndex(int minIndex, int maxIndex) {
        int testIndex = 0;
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                if (testIndex != maxIndex) {
                    testIndex++;
                }
            } else if (gamepad1.dpad_down) {
                if (testIndex != minIndex) {
                    testIndex--;
                }
            } else if (gamepad1.dpad_left) {
                testIndex = minIndex;
            } else if (gamepad1.dpad_right) {
                testIndex = maxIndex;
            } else if (gamepad1.a) {
                runTest(testIndex);
            }
            telemetry.addData("testIndex", testIndex);
            telemetry.update();
        }
    }
    public abstract void runTest(int index);
}
