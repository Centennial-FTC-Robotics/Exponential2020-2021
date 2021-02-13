package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.robots.OurRobot;

@Autonomous(group="Autonomous", name="LAZYAUTO")
public class LAZYAUTO extends LinearOpMode {
    OurRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OurRobot();
        robot.initialize(this);

        waitForStart();
        robot.setUpServos();

        // starting on rightmost red tape, facing left
        robot.odometry.loadPosition();

        robot.drivetrain.moveTo(48, -63, 180);
    }
}
