package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.mechanisms.ArcOdometry;
import org.exponential.mechanisms.Drivetrain;
import org.exponential.mechanisms.IMU;
import org.exponential.robots.OurRobot;

@TeleOp
public class TurretAngleTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        OurRobot expo = new OurRobot();
        expo.initialize(this);
        expo.shooter.setPower(0.382);

        double targetAngle = 180;
        while (opModeIsActive()) {
            expo.turret.pointAtAngle();
            expo.odometry.update();
            expo.turret.readjustTurretAngle();
            expo.drivetrain.performBrake();

            if (gamepad1.dpad_up) {
                targetAngle += 5;
                sleep(250);
            } else if (gamepad1.dpad_down) {
                targetAngle -= 5;
                sleep(250);
            }

            if(gamepad1.a){
                expo.loader.loadAndUnload();
            }

            expo.turret.setTargetAngle(targetAngle);

            telemetry.addData("target angle (in terms of the field): ", targetAngle);
            telemetry.addData("current angle (in terms of the robot): ", expo.turret.currentAngle);

            telemetry.update();
        }

    }
}
