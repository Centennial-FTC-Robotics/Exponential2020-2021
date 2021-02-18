package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.mechanisms.IMU;
import org.exponential.mechanisms.Odometry;
import org.exponential.superclasses.ExpoOpMode;


@TeleOp
public class PrintOdometryPosition extends ExpoOpMode {

    @Override
    public void run() throws InterruptedException {
        Odometry oldOdometry = new Odometry(imu);
        oldOdometry.initialize(this);

        while(opModeIsActive()){

            setDrivetrainMotorToGamepad();

            telemetry.addData("old odometry: ", oldOdometry.getxPos()+", "+oldOdometry.getyPos()+", "+oldOdometry.getAngle());
            oldOdometry.update();

            telemetry.addData("arc odometry: ", odometry.getxPos()+", "+odometry.getyPos()+", "+odometry.getAngle());
            odometry.update();

            telemetry.update();
        }
    }
}
