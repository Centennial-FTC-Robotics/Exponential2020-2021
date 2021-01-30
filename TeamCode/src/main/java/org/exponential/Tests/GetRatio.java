package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.utility.LogMaker;

@TeleOp
public class GetRatio extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        LogMaker logMaker = new LogMaker("powershotlog.txt");
        // +180 because we want the robot's front to be facing the exact opposite of the targets.
        //robot shoots backwards
        double targetAngle = 180 + Math.toDegrees(Math.atan2(72 - 0, 4 - 0));
        logMaker.write("Log for targetXPosition " + 4);
        logMaker.write("targetAngle: " + targetAngle);
        logMaker.write("atan value: " + Math.atan2(72 - 0, 4 - 0));
        logMaker.close();
    }
}
