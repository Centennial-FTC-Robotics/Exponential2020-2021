package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.exponential.robots.OurRobot;

@TeleOp
public class turretTest extends LinearOpMode {
    OurRobot expo = new OurRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        expo.initialize(this);
        double targetX = 0, targetY = 72;

        expo.turret.setTarget(targetX, targetY);
        expo.shooter.setPower(1);

        while (opModeIsActive()) {
            expo.odometry.update();
            expo.turret.readjustTurretAngle();
            if (gamepad1.a) {
                expo.loader.loadAndUnload();
            }
            telemetry.addData("Current Position: ", "(" + expo.odometry.getxPos() + ", " + expo.odometry.getyPos() + ")");
            telemetry.addData("Pointing Towards: ", "(" + targetX + ", " + targetY + ")");
            telemetry.addData("Turret Angle: ", expo.turret.currentAngle);
            telemetry.update();
        }
    }

    public void turretPower(double power) {
        expo.turret.turretMotor.setPower(power);
    }

    public void turretPosition(int position) {
        expo.turret.turretMotor.setTargetPosition(position);
    }
}
