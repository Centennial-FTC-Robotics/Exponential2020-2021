package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.superclasses.Mechanism;
import org.exponential.utility.Coordinate;
import org.exponential.utility.Line;
import org.exponential.utility.Path;

import java.util.ArrayList;
import java.util.HashMap;

public class Drivetrain implements Mechanism {
    DcMotorEx frontLeft;
    DcMotorEx backLeft;
    DcMotorEx frontRight;
    DcMotorEx backRight;
    IMU imu;
    LinearOpMode opMode;
    public Odometry positioning;

    // PID constants
    private double Kp = 0.035;
    //private double Kp = 0.04;
    private double Ki = /*0.1*/0.11;
    private double Kd = 0;
    private double tolerance = 1.5;
    private double angleTolerance = 5;
    private double angleKp = 0.03;
    private double angleKi = 0.00;
    private double angleKd = 0;

    double targetX = 0;
    double targetY = 0;
    double targetAngle = 0;

    public static final double LOOK_AHEAD_RADIUS = 12;
    Path path;
    Coordinate finalPathCoordinate;

    public Drivetrain(Odometry positioning) {
        this.positioning = positioning;
    }

    public void initialize(LinearOpMode opMode) {
        frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // TODO: maybe not set to run with encoders if these encoders will be connected to odometry encoders
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

    private double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }

    public void runPath(Path path) {
        this.path = path;

        finalPathCoordinate = path.getCoordinates().get(path.getCoordinates().size() - 1);
        new Thread(new Runnable() {
            public void run() {
                moveToPurePursuit();
            }
        }).start();

    }

    public void moveToPurePursuit() {
        double disAngle;
        double areaAngle = 0;
        double velAngle;
        ElapsedTime timer = new ElapsedTime();
        double previousTime = 0;

        double areaX = 0;
        double areaY = 0;
        double velX;
        double velY;
        double disX;
        double disY;

        while (opMode.opModeIsActive() && distance(targetX, targetY, positioning.xPos, positioning.yPos) > tolerance) {
            positioning.update();

            ArrayList<Coordinate> intersectionPoints = new ArrayList<Coordinate>();
            ArrayList<Line> lines = path.getLines();
            for (Line line : lines) {
                if (line.intersectsCircle(positioning.xPos, positioning.yPos, 12)) {
                    Coordinate[] points = line.intersectionPoints();
                    intersectionPoints.add(points[0]);
                    intersectionPoints.add(points[1]);
                }
            }
            Coordinate finalIntersection = intersectionPoints.get(intersectionPoints.size() - 1);
            targetX = finalIntersection.x;
            targetY = finalIntersection.y;

            double intervalTime = -previousTime + (previousTime = timer.seconds());

            disX = (targetX - positioning.xPos);
            disY = (targetY - positioning.yPos);

            // angle PID calculations
            disAngle = IMU.normalize(targetAngle - imu.angle);
            areaAngle = disAngle * intervalTime;
            velAngle = disAngle / intervalTime;

            double disXFromFinalPoint = Math.abs(targetX - finalPathCoordinate.x);
            double disYFromFinalPoint = Math.abs(targetY - finalPathCoordinate.y);
            if (disXFromFinalPoint < 1 && disYFromFinalPoint < 1) {
                // linear PID calculations

                areaX += disX * intervalTime;
                areaY += disY * intervalTime;
                velX = disX / intervalTime;
                velY = disY / intervalTime;

                double powerX = Kp * disX + Ki * areaX + Kd * velX;
                double powerY = Kp * disY + Ki * areaY + Kd * velY;
                double powerAngle = angleKp * disAngle + angleKi * areaAngle + angleKd * velAngle;
                setPowerFieldCentric(powerX, powerY, powerAngle);
            } else {
                // actually setting motor powers
                double powerX = disX;
                double powerY = disY;
                double powerAngle = angleKp * disAngle + angleKi * areaAngle + angleKd * velAngle;
                setPowerFieldCentric(powerX, powerY, powerAngle);
            }

        }
    }

    public void moveToTargetPosition() {
        moveToTargetPosition(this.Kp, this.Ki, this.Kd, this.angleTolerance, 999);
    }

    public void moveToTargetPosition(double angleTolerance) {
        moveToTargetPosition(this.Kp, this.Ki, this.Kd, angleTolerance, 999);
    }

    public void moveToTargetPositionStraight(double radius) {
        moveToTargetPosition(this.Kp, this.Ki, this.Kd, this.angleTolerance, radius);
    }
    public void moveToTargetPositionStraight(double radius, double angleTolerance) {
        moveToTargetPosition(this.Kp, this.Ki, this.Kd, angleTolerance, radius);
    }
    public void moveToTargetPosition(double Kp, double Ki, double Kd, double angleTolerance, double turnToTargetAngleRadius) {
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

        double angleTowardsTargetPos = Math.atan2(targetY - positioning.getyPos(), targetX - positioning.getxPos());
        //choose whether or not to make the front or back of robot face the target
        double distFromAngle = Math.abs(angleTowardsTargetPos - positioning.getAngle());
        if (distFromAngle > 90) {  // if the robot has to turn more than 90deg to face forward, face backwards instead
            angleTowardsTargetPos = IMU.normalize(angleTowardsTargetPos + 180);
        }
        while (opMode.opModeIsActive() && (distance(targetX, targetY, positioning.xPos, positioning.yPos) > tolerance)) {
            positioning.update();

            double intervalTime = -previousTime + (previousTime = timer.seconds());

            // linear PID calculations
            disX = (targetX - positioning.xPos);
            disY = (targetY - positioning.yPos);
            velX = disX / intervalTime;
            velY = disY / intervalTime;

            double distanceFromTarget = distance(targetX, targetY, positioning.xPos, positioning.yPos);
            if(distanceFromTarget < 5.0){

                areaX += disX * intervalTime;
                areaY += disY * intervalTime;
            } else {
                areaX = 0;
                areaY = 0;
            }
            // angle PID calculations
            if (distanceFromTarget < turnToTargetAngleRadius) {  // if robot is close enough to target, start turning towards target angle
                disAngle = IMU.normalize(targetAngle - positioning.angle);
            } else {  //otherwise, have the robot move straight towards the target
                disAngle = IMU.normalize(angleTowardsTargetPos - positioning.angle);
            }
            areaAngle = disAngle * intervalTime;
            velAngle = disAngle / intervalTime;

            // actually setting motor powers
            double powerX = Kp * disX + Ki * areaX + Kd * velX;
            double powerY = Kp * disY + Ki * areaY + Kd * velY;
            double powerAngle = angleKp * disAngle + angleKi * areaAngle + angleKd * velAngle;
            setPowerFieldCentric(powerX, powerY, powerAngle);


            opMode.telemetry.addData("dis X", disX);
            opMode.telemetry.addData("dis Y", disY);
            opMode.telemetry.addData("dis angle", disAngle);

            opMode.telemetry.addData("target X", targetX);
            opMode.telemetry.addData("target Y", targetY);
            opMode.telemetry.addData("target angle", targetAngle);

            /*
            opMode.telemetry.addData("power x", powerX);
            opMode.telemetry.addData("power Y", powerY);
            opMode.telemetry.addData("power angle", powerAngle);
            */

            opMode.telemetry.addData("x", positioning.getxPos());
            opMode.telemetry.addData("y", positioning.getyPos());
            opMode.telemetry.addData("angle", positioning.getAngle());

            opMode.telemetry.update();

        }

        while(Math.abs(IMU.normalize(positioning.angle-targetAngle)) > angleTolerance) {
            positioning.update();
            setPowerFieldCentric(0,0,Kp*IMU.normalize(targetAngle-positioning.angle));
        }
    }

    public void turnTo(double targetAngle) {
        moveTo(positioning.getxPos(), positioning.getyPos(), targetAngle);
    }

    public void turnTo(double targetAngle, double angleTolerance) {
        moveTo(positioning.getxPos(), positioning.getyPos(), targetAngle, angleTolerance);
    }

    public void moveTo(double targetX, double targetY, double targetAngle) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        moveToTargetPosition();
    }
    public void moveTo(double targetX, double targetY, double targetAngle, double angleTolerance) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        moveToTargetPosition(angleTolerance);
    }

    public void moveToStraight(double targetX, double targetY) {  // to just go directly towards the target, no targetAngle
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = Math.atan2(targetY - positioning.getyPos(), targetX - positioning.getxPos());
        moveToTargetPosition();
    }
    public void moveToStraight(double targetX, double targetY, double targetAngle, double radius) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        moveToTargetPositionStraight(radius);
    }

    public void moveToStraight(double targetX, double targetY, double targetAngle, double radius, double angleTolerance) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        moveToTargetPositionStraight(radius, angleTolerance);
    }

    public void moveTo(double targetX, double targetY, double targetAngle, double Kp, double Ki, double Kd) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        moveToTargetPosition(Kp, Ki, Kd, this.angleTolerance, 999);
    }


    public void moveRelative(double dx, double dy) {
        moveRelative(dx, dy, this.Kp, this.Ki, this.Kd);
    }
    public void moveRelative(double dx, double dy, double Kp, double Ki, double Kd) {
        moveTo(positioning.getxPos() + dx, positioning.getyPos() + dy, positioning.getAngle(), Kp, Ki, Kd);
    }
    public void setTargetPosition(double targetX, double targetY, double targetAngle) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
    }

    // takes in velocity in terms of motor power
    public void setPowerFieldCentric(double xVel, double yVel, double angleVel) {
        double[] robotCentricVel = positioning.toRobotCentric(xVel, yVel);
        double x = robotCentricVel[0];
        double y = robotCentricVel[1];

        // opMode.telemetry.addData("xVelField", x);
        // opMode.telemetry.addData("yVelField", y);

        double sum = Math.abs(x) + Math.abs(y) + Math.abs(angleVel);
        if (sum > 1) {
            frontRight.setPower((-x + y + angleVel) / sum);
            backRight.setPower((x + y + angleVel) / sum);
            backLeft.setPower((-x + y - angleVel) / sum);
            frontLeft.setPower((x + y - angleVel) / sum);
        } else {
            frontRight.setPower((-x + y + angleVel));
            backRight.setPower((x + y + angleVel));
            backLeft.setPower((-x + y - angleVel));
            frontLeft.setPower((x + y - angleVel));
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

    public void moveToFaceTarget(double xTarg, double yTarg){
        double targetAngle = Math.toDegrees(Math.atan2(yTarg-positioning.getyPos(), xTarg-positioning.getxPos()));
        moveTo(xTarg, yTarg, targetAngle);
    }

    public void performBrake() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void setKP(double P) {
        Kp = P;
    }

    public void setKI(double I) {
        Ki = I;
    }

    public void setKD(double D) {
        Kd = D;
    }

}
