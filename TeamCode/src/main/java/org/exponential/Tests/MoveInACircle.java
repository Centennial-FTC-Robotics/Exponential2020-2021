package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.ArcOdometry;
import org.exponential.mechanisms.DriveTrainParametric;
import org.exponential.mechanisms.Drivetrain;
import org.exponential.mechanisms.IMU;
import org.exponential.mechanisms.parametricEQ.ParametricEq;
import org.exponential.mechanisms.parametricEQ.State;
import org.exponential.robots.OurRobot;

import java.io.IOException;
@Autonomous
public class MoveInACircle extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        OurRobot expo = new OurRobot();

        expo.imu = new IMU();
        expo.imu.initialize(this);

        expo.odometry = new ArcOdometry(expo.imu);
        expo.odometry.initialize(this);
        expo.drivetrain = new DriveTrainParametric(expo.odometry);
        expo.drivetrain.initialize(this);

        waitForStart();
        ((DriveTrainParametric)expo.drivetrain).moveAlongParametricEq(new ParametricEq() {


            @Override
            public State getStateAtTime(double t) {
                double period = 10;
                double radius = 30;
                State returned = new State();
                returned.fieldX = radius * Math.cos(t / period * Math.PI * 2);
                returned.fieldY = radius * Math.sin(t / period * Math.PI * 2);
                returned.angle = 360*t/period + 90;
                returned.velX = -radius * Math.sin(t / period * Math.PI * 2) / period * Math.PI * 2;
                returned.velY = radius * Math.cos(t / period * Math.PI * 2) / period * Math.PI * 2;
                returned.angleVel = 360 / period  + 90;


                return returned;

            }

            @Override
            public boolean moveOn(double currentX, double currentY, double currentAngle, double velX, double velY, double velAngle) {
                return false;
            }
        });

    }
}
