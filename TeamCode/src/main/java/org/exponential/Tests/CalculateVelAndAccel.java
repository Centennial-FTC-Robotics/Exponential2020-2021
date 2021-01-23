package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.mechanisms.ArcOdometry;
import org.exponential.mechanisms.DriveTrainParametric;
import org.exponential.mechanisms.IMU;
import org.exponential.robots.OurRobot;

import java.util.HashMap;
import java.util.Map;

//@TeleOp
public class CalculateVelAndAccel extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        OurRobot expo = new OurRobot();
        expo.imu = new IMU();
        expo.imu.initialize(this);
        //expo.odometry = new ArcOdometry(expo.imu);
        expo.odometry.initialize(this);
        DriveTrainParametric drive = (DriveTrainParametric) (expo.drivetrain = new DriveTrainParametric(expo.odometry));
        expo.drivetrain.initialize(this);

        waitForStart();

        double maxXVel = 0, maxYVel = 0, maxAngleVel = 0, maxXAccel = 0, maxYAccel = 0, maxAngleAccel = 0;
        double[] robotVel = {0, 0, 0};
        ElapsedTime timer = new ElapsedTime();
        double time = 0;
        boolean fieldCentric = false;
        while (opModeIsActive()) {
            expo.odometry.update();
            double timeDiff = (time - (time = timer.seconds()));
            double[] currentRobotVel = new double[3];
            currentRobotVel[0] = expo.odometry.toRobotCentric(expo.odometry.getxVel(), expo.odometry.getyVel())[0];
            currentRobotVel[1] = expo.odometry.toRobotCentric(expo.odometry.getxVel(), expo.odometry.getyVel())[1];
            currentRobotVel[2] = expo.odometry.getAngleVel();

            maxXVel = Math.max(currentRobotVel[0], maxXVel);
            maxYVel = Math.max(currentRobotVel[1], maxYVel);
            maxAngleVel = Math.max(currentRobotVel[2], maxAngleVel);

            maxXAccel = Math.max((currentRobotVel[0] - robotVel[0]) / timeDiff, maxXAccel);
            maxYAccel = Math.max((currentRobotVel[1] - robotVel[1]) / timeDiff, maxYAccel);
            maxAngleAccel = Math.max((currentRobotVel[2] - robotVel[2]) / timeDiff, maxAngleAccel);


            if (gamepad1.a && !gamepad1.start) {
                fieldCentric = !fieldCentric;
            }
            if (fieldCentric) {
                expo.drivetrain.setPowerFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            } else {
                expo.drivetrain.setPowerDriveMotors(getMotorPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
            }

            telemetry.addData("max X Vel: ", maxXVel);
            telemetry.addData("max Y Vel: ", maxYVel);
            telemetry.addData("max angle Vel: ", maxAngleVel);
            telemetry.addData("max X Accel: ", maxXAccel);
            telemetry.addData("max Y Accel: ", maxYAccel);
            telemetry.addData("max Angle Accel: ", maxAngleAccel);
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
