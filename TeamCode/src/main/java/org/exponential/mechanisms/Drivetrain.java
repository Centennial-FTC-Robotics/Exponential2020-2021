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

    public void setPowerDriveMotors(HashMap<String, Double> powers) {
        frontLeft.setPower(powers.get("frontLeft"));
        backLeft.setPower(powers.get("backLeft"));
        frontRight.setPower(powers.get("frontRight"));
        backRight.setPower(powers.get("backRight"));
    }

    class Odometry implements Runnable {
        DcMotorEx forwardLeftEnc;
        DcMotorEx forwardRightEnc;
        DcMotorEx horizontalEnc;


        // all in field centric
        double xPos;
        double yPos;
        double angle;
        double xVel;
        double yVel;
        double angleVel;

        // encapsulated variables
        private int lastLeftEncPos;
        private int lastRightEncPos;
        private int lastHoriEncPos;

        Odometry(DcMotorEx forwardLeftEnc, DcMotorEx forwardRightEnc, DcMotorEx horizontalEnc) {
            this.forwardLeftEnc = forwardLeftEnc;
            this.forwardRightEnc = forwardRightEnc;
            this.horizontalEnc = horizontalEnc;

            forwardLeftEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            forwardRightEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            horizontalEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            forwardLeftEnc.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            forwardRightEnc.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            horizontalEnc.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            // TODO: make sure that the encoders are in the right directions, so maybe reverse directions if needed
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

        public double encToInch(int encoders) {
            //TODO: change once builders have actual measurements
            return 0;
        }

        @Override
        public void run() {
            ElapsedTime timer = new ElapsedTime();
            while (opMode.opModeIsActive()) {
                update(timer.seconds());
                timer.reset();
            }
        }

        public void update(double timeElapsed) {
            // updates position, velocity, and angle according to how much time has elapsed

            int leftEncChange = forwardLeftEnc.getCurrentPosition() - lastLeftEncPos;
            int rightEncChange = forwardRightEnc.getCurrentPosition() - lastRightEncPos;
            int horiEncChange = horizontalEnc.getCurrentPosition() - lastHoriEncPos;

            // does not call getCurrentPosition a second time because you would not account for encoder
            // readings from the time between the two calls
            lastLeftEncPos += leftEncChange;
            lastRightEncPos += rightEncChange;
            lastHoriEncPos += horiEncChange;

            // currently in robot centric
            double[] changeInPos = new double[]{encToInch(horiEncChange), encToInch((leftEncChange + rightEncChange) / 2)};
            // changes to field centric displacement vector
            changeInPos = toFieldCentric(changeInPos[0], changeInPos[1]);

            xPos += changeInPos[0];
            yPos += changeInPos[1];

            xVel = changeInPos[0] / timeElapsed;
            yVel = changeInPos[1] / timeElapsed;

            imu.update();
            double changeInAngle = imu.angle - angle;
            angleVel = changeInAngle / timeElapsed;

            angle += changeInAngle;

        }
    }
}
