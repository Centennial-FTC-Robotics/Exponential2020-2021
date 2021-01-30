package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.robots.OurRobot;

@Autonomous(group="Autonomous", name="OneRotateCommand")
public class OneRotateCommand extends LinearOpMode {
    OurRobot ourRobot;
    @Override
    public void runOpMode() throws InterruptedException {
        ourRobot = new OurRobot();
        ourRobot.initialize(this);
        ourRobot.odometry.setPosition(0, 0, 0);
        waitForStart();

        ourRobot.drivetrain.turnTo(90);
    }
}
