package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.superclasses.ExpoOpMode;

@TeleOp
public class GetOdometryRatio extends ExpoOpMode {
    @Override
    public void run() throws InterruptedException {
        double pastIMUAngle = 0;
        int previousHoriEnc = 0;

        int previousLeftEnc = 0;
        int previousRightEnc = 0;

        while (opModeIsActive()) {
            expo.drivetrain.setPowerFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            imu.update();
            int currentLeft = odometry.forwardLeftEnc.getCurrentPosition();
            int currentRight = odometry.forwardRightEnc.getCurrentPosition();
            int currentHori= odometry.horizontalEnc.getCurrentPosition();
            int changeLeft = (-previousLeftEnc + currentLeft);
            int changeRight = -(-previousRightEnc + currentRight);
            int changeHori = -(-previousHoriEnc + currentHori);

            previousHoriEnc = currentHori;
            previousLeftEnc = currentLeft;
            previousRightEnc = currentRight;

            // telemetry.addData("horizontal enc per degree: ",  changeHori/ imu.angle);
            telemetry.addData("imu angle: ", imu.angle);
            telemetry.addData("encoder angle: ", (odometry.forwardLeftEnc.getCurrentPosition() - odometry.forwardRightEnc.getCurrentPosition())
                    / ((odometry.vertRightEncPerDegree - odometry.vertLeftEncPerDegree)));
            telemetry.addData("imu change in angle: ", imu.angle - pastIMUAngle);
            telemetry.addData("encoder change in angle: ", -(changeRight - changeLeft)
                    / (odometry.vertRightEncPerDegree - odometry.vertLeftEncPerDegree));
            // telemetry.addData("weighted average (in inches): ",
            //         odometry.encToInch(odometry.weightedAverage(changeLeft, changeRight)));
            telemetry.update();
            pastIMUAngle = imu.angle;
            sleep(100);
        }
    }
}