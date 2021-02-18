package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.superclasses.ExpoOpMode;

@TeleOp
public class GetOdometryRatio extends ExpoOpMode {
    @Override
    public void run() throws InterruptedException {
        while (opModeIsActive()) {
            expo.drivetrain.setPowerFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            imu.update();
            telemetry.addData("horizontal enc per degree: ", odometry.horizontalEnc.getCurrentPosition() / imu.angle);
            telemetry.addData("left enc per degree: ", -odometry.forwardLeftEnc.getCurrentPosition() / imu.angle);
            telemetry.addData("right enc per degree: ", -odometry.forwardRightEnc.getCurrentPosition() / imu.angle);
            telemetry.addData("imu change in angle: ", imu.angle);
            telemetry.addData("encoder change in angle: ", -(odometry.forwardRightEnc.getCurrentPosition() -
                    odometry.forwardLeftEnc.getCurrentPosition())
                    / (odometry.vertRightEncPerDegree - odometry.vertLeftEncPerDegree));
            telemetry.addData("weighted average (in inches): ",
                    odometry.encToInch(odometry.weightedAverage(-odometry.forwardLeftEnc.getCurrentPosition(), -odometry.forwardRightEnc.getCurrentPosition())));
            telemetry.update();
        }
    }
}