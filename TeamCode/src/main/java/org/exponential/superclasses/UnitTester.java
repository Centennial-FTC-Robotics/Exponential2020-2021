package org.exponential.superclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class UnitTester extends LinearOpMode {
    public void trackIndex(int minIndex, int maxIndex) {
        int testIndex = 0;
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                if (testIndex != maxIndex) {
                    testIndex++;
                    sleep(500);  // so that the cases don't increment instantly
                }
            } else if (gamepad1.dpad_down) {
                if (testIndex != minIndex) {
                    testIndex--;
                    sleep(500);
                }
            } else if (gamepad1.dpad_left) {
                testIndex = minIndex;
            } else if (gamepad1.dpad_right) {
                testIndex = maxIndex;
            } else if (gamepad1.a) {
                runTest(testIndex);
            }
            // this is to hopefully not override camera telemetry
            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                telemetry.addData("testIndex", testIndex);
                telemetry.update();
            }
        }
    }
    public abstract void runTest(int index);
}
