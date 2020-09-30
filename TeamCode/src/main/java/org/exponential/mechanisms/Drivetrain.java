package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.superclasses.Mechanism;

import java.util.HashMap;

public class Drivetrain implements Mechanism {

    class Odometry implements Runnable {
        DcMotorEx forwardLeftEnc;
        DcMotorEx forwardRightEnc;
        DcMotorEx horizontalEnc;

        Odometry(DcMotorEx forwardLeftEnc, DcMotorEx forwardRightEnc, DcMotorEx horizontalEnc) {
            this.forwardLeftEnc = forwardLeftEnc;
            this.forwardRightEnc = forwardRightEnc;
            this.horizontalEnc = horizontalEnc;
        }

        @Override
        public void run() {
            // for multithreading if we need it
            ElapsedTime timer = new ElapsedTime();
            while (opMode.opModeIsActive()) {
                update(timer.seconds());
                timer.reset();
            }
        }

        public void update(double timeElapsed) {
            // updates position, velocity, and angle according to how much time has elapsed


        }
    }

    DcMotorEx frontLeft;
    DcMotorEx backLeft;
    DcMotorEx frontRight;
    DcMotorEx backRight;
    IMU imu;
    LinearOpMode opMode;

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

    }

    public double[] toFieldCentric(double robotX, double robotY) {
        double angleRad = Math.toRadians(imu.angle);
        double centricX = robotX * Math.cos(angleRad - Math.PI / 2) - robotY * Math.sin(angleRad - Math.PI / 2);
        double centricY = robotY * Math.cos(angleRad - Math.PI / 2) + robotX * Math.sin(angleRad - Math.PI / 2);

        return new double[]{centricX, centricY};
    }

    public double[] toRobotCentric(double fieldX, double fieldY) {
        double angleRad = Math.toRadians(imu.angle);
        double robotX = fieldX * Math.cos(-angleRad + Math.PI / 2) - fieldY * Math.sin(-angleRad + Math.PI / 2);
        double robotY = fieldY * Math.cos(-angleRad + Math.PI / 2) + fieldX * Math.sin(-angleRad + Math.PI / 2);

        return new double[]{robotX, robotY};
    }

    public void setPowerDriveMotors(HashMap<String, Double> powers) {
        frontLeft.setPower(powers.get("frontLeft"));
        backLeft.setPower(powers.get("backLeft"));
        frontRight.setPower(powers.get("frontRight"));
        backRight.setPower(powers.get("backRight"));
    }
}
