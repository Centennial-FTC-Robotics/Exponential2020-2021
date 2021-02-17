package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.superclasses.ExpoOpMode;

@TeleOp
public class GetOdometryRatio extends ExpoOpMode {
    @Override
    public void run() throws InterruptedException {
        while (opModeIsActive()) {
            imu.update();
            telemetry.addData("horizontal enc per degree: ", odometry.horizontalEnc.getCurrentPosition() / imu.angle);
            telemetry.addData("left enc per degree: ", -odometry.forwardLeftEnc.getCurrentPosition() / imu.angle);
            telemetry.addData("right enc per degree: ", -odometry.forwardRightEnc.getCurrentPosition() / imu.angle);
            telemetry.update();
        }
    }
}
