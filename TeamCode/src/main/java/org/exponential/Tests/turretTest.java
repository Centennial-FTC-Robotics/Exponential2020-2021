package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.exponential.mechanisms.Turret;
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
        // expo.shooter.setPower(0.382);
        expo.turret.pointAtTarget();

        while (opModeIsActive()) {
            expo.odometry.update();
            expo.turret.readjustTurretAngle();
            if (gamepad1.a) {
                expo.loader.loadAndUnload();
            }
            if(gamepad1.b){
                if(expo.turret.currentCommand== Turret.RELOAD){
                    expo.turret.pointAtTarget();
                } else {
                    expo.turret.pointToReloadPosition();
                }
                sleep(500);
            }
            expo.drivetrain.setPowerFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            telemetry.addData("Current Position: ", "(" + expo.odometry.getxPos() + ", " + expo.odometry.getyPos() + ")");
            telemetry.addData("Angle: ", expo.odometry.getAngle());
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
