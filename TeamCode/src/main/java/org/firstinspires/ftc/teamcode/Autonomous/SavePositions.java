package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.robots.OurRobot;

@Autonomous(group="Autonomous", name="SavePositions")
public class SavePositions extends LinearOpMode {
    OurRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OurRobot();
        robot.initialize(this);
        robot.odometry.setPosition(0, 0, 90);
        robot.turret.setAngle(0);
        robot.odometry.savePosition();
        robot.turret.savePosition();
    }
}
