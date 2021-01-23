package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.mechanisms.ArcOdometry;
import org.exponential.mechanisms.Drivetrain;
import org.exponential.mechanisms.IMU;
import org.exponential.mechanisms.Odometry;
import org.exponential.robots.OurRobot;
import org.exponential.superclasses.Robot;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@TeleOp
public class PrintLocation extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        OurRobot expo = new OurRobot();
        expo.imu = new IMU();
        expo.imu.initialize(this);
        expo.odometry = new ArcOdometry(expo.imu);
        expo.odometry.initialize(this);
        expo.drivetrain = new Drivetrain(expo.odometry);
        expo.drivetrain.initialize(this);

        boolean fieldCentric = false;
        while(opModeIsActive()){
            if(gamepad1.a && !gamepad1.start){
                fieldCentric=!fieldCentric;
            }
            if(fieldCentric) {
                expo.drivetrain.setPowerFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            } else {
                expo.drivetrain.setPowerDriveMotors(getMotorPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
            }
            expo.odometry.update();

            telemetry.addData("x", expo.odometry.getxPos());
            telemetry.addData("y", expo.odometry.getyPos());
            telemetry.addData("angle", expo.odometry.getAngle());
            // telemetry.addData("normalized angle", IMU.normalize(expo.odometry.getAngle()));

            telemetry.update();

        }

    }

    public HashMap<String, Double> getMotorPowers(double x, double y, double rotate) {
        HashMap<String, Double> powers = new HashMap<>();
        powers.put("frontLeft", x + y + rotate);
        powers.put("backLeft", -x + y + rotate);
        powers.put("frontRight", -x + y - rotate);
        powers.put("backRight", x + y - rotate);

        double maxPower = Math.max(Math.max(Math.max(Math.abs(powers.get("frontLeft")),
                Math.abs(powers.get("backLeft"))), Math.abs(powers.get("frontRight"))), Math.abs(powers.get("backRight")));
        if (maxPower > 1) {
            for (Map.Entry<String, Double> mapElement : powers.entrySet()) {
                powers.put(mapElement.getKey(), mapElement.getValue() / maxPower);
            }
        }
        return powers;
    }
}