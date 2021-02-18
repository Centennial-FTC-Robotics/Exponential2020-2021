package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.robots.OurRobot;

@TeleOp
public class OdometryOffsetCalibration extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        waitForStart();

        OurRobot expo = new OurRobot();
        expo.initialize(this);
        while(opModeIsActive()){
            expo.drivetrain.setPowerFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            expo.imu.update();
            telemetry.addData("Ratio: ", expo.odometry.horizontalEnc.getCurrentPosition()/expo.imu.angle);
            telemetry.update();
        }
    }
}
