package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.superclasses.Mechanism;

import java.util.HashMap;

public class Drivetrain implements Mechanism {
    DcMotorEx frontLeft;
    DcMotorEx backLeft;
    DcMotorEx frontRight;
    DcMotorEx backRight;
    IMU imu;
    LinearOpMode opMode;
    Odometry positioning;

    // PID constants
    private double Kp = 0.1;
    private double Ki = 0.01;
    private double Kd = 0;
    private double tolerance = 0.75;

    private double angleKp = 0.02;
    private double angleKi = 0.01;
    private double angleKd = 0;

    public void initialize(LinearOpMode opMode) {
        frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // TODO: maybe not set to run with encoders if these encoders will be connected to odometry encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = new IMU();
        imu.initialize(opMode);

        this.opMode = opMode;

        positioning = new Odometry(imu);
        positioning.initialize(opMode);
    }

    public void setPowerDriveMotors(HashMap<String, Double> powers) {
        frontLeft.setPower(powers.get("frontLeft"));
        backLeft.setPower(powers.get("backLeft"));
        frontRight.setPower(powers.get("frontRight"));
        backRight.setPower(powers.get("backRight"));
    }

    private double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }


    public void moveTo(double targetX, double targetY, double targetAngle) {
        // a lot of random PID variables
        double areaX = 0;
        double areaY = 0;
        double velX;
        double velY;
        double disX;
        double disY;

        double disAngle;
        double areaAngle = 0;
        double velAngle;
        ElapsedTime timer = new ElapsedTime();
        double previousTime = 0;

        while (opMode.opModeIsActive() && distance(targetX, targetY, positioning.xPos, positioning.yPos) > tolerance) {
            positioning.update();

            double intervalTime = -previousTime + (previousTime = timer.seconds());

            // linear PID calculations
            disX = (targetX - positioning.xPos);
            disY = (targetY - positioning.yPos);
            areaX += disX * intervalTime;
            areaY += disY * intervalTime;
            velX = disX / intervalTime;
            velY = disY / intervalTime;

            // angle PID calculations
            disAngle = IMU.normalize(targetAngle - imu.angle);
            areaAngle = disAngle * intervalTime;
            velAngle = disAngle / intervalTime;

            // actually setting motor powers
            double powerX = Kp * disX + Ki * areaX + Kd * velX;
            double powerY = Kp * disY + Ki * areaY + Kd * velY;
            double powerAngle = angleKp * disAngle + angleKi * areaAngle + angleKd * velAngle;
            setPowerFieldCentric(powerX, powerY, powerAngle);

        }
    }

    // takes in velocity in terms of motor power
    public void setPowerFieldCentric(double xVel, double yVel, double angleVel) {
        double[] robotCentricVel = positioning.toRobotCentric(xVel, yVel);
        double x = robotCentricVel[0];
        double y = robotCentricVel[1];

        // TODO: I think I made a mistake here
        double sum = Math.abs(x) + Math.abs(y) + Math.abs(angleVel);
        if (sum > 1) {
            frontRight.setPower((-x + y - angleVel) / sum);
            backRight.setPower((x + y - angleVel) / sum);
            backLeft.setPower((-x + y + angleVel) / sum);
            frontLeft.setPower((x + y + angleVel) / sum);
        } else {
            frontRight.setPower((-x + y - angleVel));
            backRight.setPower((x + y - angleVel));
            backLeft.setPower((-x + y + angleVel));
            frontLeft.setPower((x + y + angleVel));
        }
    }


    private double[] getMotorPowers(double triggerX, double triggerY, double rotate) {
        double[] motorPowers = new double[4];
        double x;
        double y;

        if (triggerX == 0.0) {
            x = 0.0;
        } else {
            x = triggerX / Math.abs(triggerX) * Math.sqrt(Math.pow(triggerX, 2) + Math.pow(triggerY, 2))
                    * (Math.abs(triggerX)) / (Math.abs(triggerX) + Math.abs(triggerY));
        }
        if (triggerY == 0.0) {
            y = 0.0;
        } else {
            y = triggerY / Math.abs(triggerY) * Math.sqrt(Math.pow(triggerX, 2) + Math.pow(triggerY, 2))
                    * (Math.abs(triggerY)) / (Math.abs(triggerX) + Math.abs(triggerY));
        }

        double sum = Math.abs(x) + Math.abs(y) + Math.abs(rotate);
        if (sum > 1) {
            motorPowers[0] = (x + y - rotate) / sum;
            motorPowers[1] = (-x + y - rotate) / sum;
            motorPowers[2] = (x + y + rotate) / sum;
            motorPowers[3] = (-x + y + rotate) / sum;
        } else {
            motorPowers[0] = (x + y - rotate);
            motorPowers[1] = (-x + y - rotate);
            motorPowers[2] = (x + y + rotate);
            motorPowers[3] = (-x + y + rotate);
        }
        return motorPowers;
    }
}
