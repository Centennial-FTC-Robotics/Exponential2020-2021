package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.ArcOdometry;
import org.exponential.mechanisms.DriveTrainParametric;
import org.exponential.mechanisms.Drivetrain;
import org.exponential.mechanisms.IMU;
import org.exponential.mechanisms.Odometry;
import org.exponential.robots.OurRobot;

@Disabled
@Autonomous
public class MoveTester extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        OurRobot expo = new OurRobot();
        expo.imu = new IMU();
        expo.imu.initialize(this);

        expo.odometry = new ArcOdometry(expo.imu);
        expo.odometry.initialize(this);
        expo.odometry.setPosition(0,0,0);
        expo.drivetrain = new DriveTrainParametric(expo.odometry);
        expo.drivetrain.initialize(this);

        double[] target = new double[3];
        boolean alreadyPressed = false;
        target[0]=30;

        while (opModeIsActive()) {
            int currentIndex = 0;
            while(currentIndex<3&&opModeIsActive()) {
                if (gamepad1.dpad_up && !alreadyPressed) {
                    target[currentIndex]+=3;
                    alreadyPressed = true;
                } else if (gamepad1.dpad_down && !alreadyPressed) {
                    target[currentIndex]-=3;
                    alreadyPressed = true;
                } else if (gamepad1.a && !alreadyPressed) {
                    currentIndex++;
                    alreadyPressed = true;
                } else if (!(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.a)) {
                    alreadyPressed = false;
                }

                telemetry.addData("target x", target[0]);
                telemetry.addData("target y", target[1]);
                telemetry.addData("target angle", 5*target[2]);
                telemetry.addData("pos x", expo.odometry.getxPos());
                telemetry.addData("pos y", expo.odometry.getyPos());
                telemetry.addData("pos angle", expo.odometry.getAngle());

                telemetry.update();
            }

            expo.drivetrain.moveTo(target[0], target[1], 5*target[2]);
            expo.drivetrain.performBrake();
        }

    }
}
