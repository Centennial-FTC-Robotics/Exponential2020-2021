package org.exponential.mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

public class DriveTrainParametric extends Drivetrain {
    // idea is to have drivetrain methods that use feedforward-feedback on a parametric function
    // with motion profiling

    // feedforward constants
    private final static double xVelMax = 12; // inch per sec per motor power
    private final static double yVelMax = 10; // inch per sec per motor power
    private final static double rotate = 10; // degrees per sec per motor power
    private final static double xAccelMax = 4; // inch per sec^2
    private final static double yAccelMax = 3; // inch per sec^2

    // feedback constants
    private final static double KPBACK = 0.5;
    private final static double KIBACK = 0.1;
    private final static double KDBACK = 0;
    private final static double KPBACKANGLE = 0.5;
    private final static double KIBACKANGLE = 0.1;
    private final static double KDBACKANGLE = 0;

    public void moveAlongParametricEq(ParametricEq equation) {

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
        while (opMode.opModeIsActive() && !equation.moveOn(positioning.xPos, positioning.yPos,
                positioning.angle, positioning.xVel, positioning.yVel, positioning.angleVel)) {

            // updates position
            positioning.update();

            // updates time
            double intervalTime = -previousTime + (previousTime = timer.seconds());

            // finds what the position and velocity should be at this time
            State targetState = equation.getStateAtTime(timer.seconds());

            // linear PID calculations
            disX = (targetState.fieldX - positioning.xPos);
            disY = (targetState.fieldY - positioning.yPos);
            areaX += disX * intervalTime;
            areaY += disY * intervalTime;
            velX = disX / intervalTime;
            velY = disY / intervalTime;

            // Angle PID calculations
            disAngle = IMU.normalize(targetState.angle - imu.angle);
            areaAngle = disAngle * intervalTime;
            velAngle = disAngle / intervalTime;

            // Feedback portion
            double xNew = KPBACK * disX + KIBACK * areaX + KDBACK * velX;
            double yNew = KPBACK * disY + KIBACK * areaY + KDBACK * velY;
            double angleNew = KPBACKANGLE * disAngle + KIBACKANGLE * areaAngle + KDBACKANGLE * velAngle;

            // Feedforward portion
            xNew += targetState.velX;
            yNew += targetState.velY;
            angleNew += targetState.angleVel;

            // sets the velocity using encapsulated velocity method that uses motion profiling
            setVel(xNew, yNew, angleNew);

        }
    }


    // sets the velocity in inches per second
    // if the velocity cannot be achieved, then it goes to the top vel it can in the given direction
    public void setVel(double xInchPerSec, double yInchPerSec, double degreePerSec) {
        double[] robotVel = positioning.toRobotCentric(xInchPerSec, yInchPerSec);

        // vertical component (robot centric)
        double frontLeftPow = robotVel[1] / yVelMax;
        double frontRightPow = robotVel[1] / yVelMax;
        double backLeftPow = robotVel[1] / yVelMax;
        double backRightPow = robotVel[1] / yVelMax;

        // horizontal component (robot centric)
        frontLeftPow += robotVel[0] / xVelMax;
        frontRightPow -= robotVel[0] / xVelMax;
        backRightPow += robotVel[0] / xVelMax;
        backLeftPow -= robotVel[0] / xVelMax;

        // rotational component
        frontLeftPow -= degreePerSec / rotate;
        frontRightPow += degreePerSec / rotate;
        backRightPow += degreePerSec / rotate;
        backLeftPow -= degreePerSec / rotate;

        double sum = Math.max(Math.max(Math.max(Math.abs(frontLeftPow), Math.abs(frontRightPow)),
                Math.abs(backLeftPow)), Math.abs(backRightPow));

        if (sum > 1) {
            frontLeft.setPower(frontLeftPow / sum);
            frontRight.setPower(frontRightPow / sum);
            backLeft.setPower(backLeftPow / sum);
            backRight.setPower(backRightPow / sum);
        } else {
            frontLeft.setPower(frontLeftPow);
            frontRight.setPower(frontRightPow);
            backLeft.setPower(backLeftPow);
            backRight.setPower(backRightPow);
        }
    }


    public void moveTo(double x, double y, double angle) {
        // gives a little bit of elbow room so that the robot is not forced to go at the fastest
        // theoretically possible path
        double portionOfMaxAccel = 0.9; // how much leeway to give


    }

    public static double[] rotateCounterClock(double x, double y, double angleRad) {
        double[] rotated = new double[2];
        rotated[0] = x * Math.cos(angleRad) - y * Math.sin(angleRad);
        rotated[1] = y * Math.cos(angleRad) + x * Math.sin(angleRad);
        return rotated;


    }

    // time-differentiable vector-valued parametric equation for the position of our robot as a function of time
    static abstract class ParametricEq {

        // returns what the state of the robot should at time t
        public abstract State getStateAtTime(double t);

        // tells whether to end the strafe
        public abstract boolean moveOn(
                double currentX, double currentY, double currentAngle,
                double velX, double velY, double velAngle);
    }

    static class StraightLine extends ParametricEq {
        // parametric equation to go at a percentage of the fastest theoretical path

        // variables for calculation
        private double targetAngle;
        private double xFieldAccel;
        private double yFieldAccel;
        private double xMaxVel;
        private double yMaxVel;
        private double targetX;
        private double targetY;
        private double startX;
        private double startY;


        private boolean hitsMaxSpeed;
        private double accelTime;
        private double totalTime;


        StraightLine(double targetX, double targetY, double targetAngle,
                     double startX, double startY, double startAngle,
                     double portion /* fraction of the fastest possible straight path*/) {
            // assumes that the robot starts at velocity of 0 and ends at a velocity of 0

            this.startX = startX;
            this.startY = startY;
            this.targetX = targetX;
            this.targetY = targetY;
            this.targetAngle = targetAngle;


            // angle of displacement vector from the robot's perspective
            double robotDisplacementAngle = Math.atan2(targetY - startY, targetX - startX) - targetAngle + 90;
            robotDisplacementAngle = Math.toRadians(robotDisplacementAngle);
            double max = Math.max(
                    Math.abs(Math.sin(robotDisplacementAngle) / yAccelMax - Math.cos(robotDisplacementAngle) / xAccelMax),
                    Math.abs(Math.sin(robotDisplacementAngle) / yAccelMax + Math.cos(robotDisplacementAngle) / xAccelMax));

            // robot centric (assumes the robot is always oriented at target angle)
            double[] accel = {portion / max * xAccelMax * Math.cos(robotDisplacementAngle),
                    portion / max * yAccelMax * Math.sin(robotDisplacementAngle)};
            double[] maxVel = {xVelMax / max * Math.cos(robotDisplacementAngle),
                    yVelMax / max * Math.sin(robotDisplacementAngle)};

            // field centric
            accel = rotateCounterClock(accel[0], accel[1], robotDisplacementAngle - Math.PI / 2);
            xFieldAccel = accel[0];
            yFieldAccel = accel[1];
            maxVel = rotateCounterClock(maxVel[0], maxVel[1], robotDisplacementAngle - Math.PI / 2);
            xMaxVel = maxVel[0];
            yMaxVel = maxVel[1];

            // kinematic equations
            double xDis = targetX - startX;
            double yDis = targetY - startY;
            if (xFieldAccel != 0.0) {
                if (Math.sqrt(Math.abs(xDis) / xFieldAccel) > Math.abs(xMaxVel / xFieldAccel)) {
                    // robot reaches max velocity
                    hitsMaxSpeed = true;
                    accelTime = Math.abs(xMaxVel / xFieldAccel);
                    totalTime = Math.abs((xDis - xFieldAccel * (Math.pow(accelTime, 2))) / xMaxVel) + 2 * accelTime;
                } else {
                    // robot does not reach max velocity
                    hitsMaxSpeed = false;
                    accelTime = Math.sqrt(Math.abs(xDis) / xFieldAccel);
                    totalTime = 2 * accelTime;
                }
            } else {
                if (Math.sqrt(Math.abs(yDis) / yFieldAccel) > Math.abs(yMaxVel / yFieldAccel)) {
                    // robot reaches max velocity
                    hitsMaxSpeed = true;
                    accelTime = Math.abs(yMaxVel / yFieldAccel);
                    totalTime = Math.abs((yDis - yFieldAccel * (Math.pow(accelTime, 2))) / yMaxVel) + 2 * accelTime;
                } else {
                    // robot does not reach max velocity
                    hitsMaxSpeed = false;
                    accelTime = Math.sqrt(Math.abs(yDis) / yFieldAccel);
                    totalTime = 2 * accelTime;
                }
            }
            // ^don't ask me how this works because I don't know either
        }

        @Override
        public State getStateAtTime(double t) {
            State theState = new State();
            theState.angle = targetAngle;
            theState.angleVel = 0;
            if (hitsMaxSpeed) {
                if (t < accelTime) {
                    // speeding up
                    theState.fieldX = xFieldAccel / 2.0 * Math.pow(t, 2) + startX;
                    theState.velX = xFieldAccel * t;

                    theState.fieldY = yFieldAccel / 2.0 * Math.pow(t, 2) + startY;
                    theState.velY = yFieldAccel * t;
                } else if (t < totalTime - accelTime) {
                    // maintaining speed
                    theState.fieldX = xFieldAccel / 2.0 * Math.pow(accelTime, 2) + startX + xMaxVel * (t - accelTime);
                    theState.velX = xMaxVel;

                    theState.fieldY = yFieldAccel / 2.0 * Math.pow(accelTime, 2) + startY + yMaxVel * (t - accelTime);
                    theState.velY = yMaxVel;

                } else if (t <= totalTime) {
                    // slowing down
                    theState.fieldX = targetX - xFieldAccel / 2.0 * Math.pow(totalTime - t, 2);
                    theState.velX = xFieldAccel * (totalTime - t);

                    theState.fieldY = targetY - yFieldAccel / 2.0 * Math.pow(totalTime - t, 2);
                    theState.velY = yFieldAccel * (totalTime - t);

                } else {
                    // stopped
                    theState.fieldX = targetX;
                    theState.velX = 0;

                    theState.fieldY = targetY;
                    theState.velY = 0;

                }
            } else {
                if (t < accelTime) {
                    // speeding up
                    theState.fieldX = xFieldAccel / 2.0 * Math.pow(t, 2) + startX;
                    theState.velX = xFieldAccel * t;

                    theState.fieldY = yFieldAccel / 2.0 * Math.pow(t, 2) + startY;
                    theState.velY = yFieldAccel * t;

                } else if (t <= totalTime) {
                    // slowing down
                    theState.fieldX = targetX - xFieldAccel / 2.0 * Math.pow(totalTime - t, 2);
                    theState.velX = xFieldAccel * (totalTime - t);


                    theState.fieldY = targetY - yFieldAccel / 2.0 * Math.pow(totalTime - t, 2);
                    theState.velY = yFieldAccel * (totalTime - t);

                } else {
                    // stopped
                    theState.fieldX = targetX;
                    theState.velX = 0;

                    theState.fieldY = targetY;
                    theState.velY = 0;
                }
            }

            return theState;
        }

        @Override
        public boolean moveOn(double currentX, double currentY, double currentAngle, double velX, double velY, double velAngle) {
            return Math.sqrt(Math.pow(targetX - currentX, 2) + Math.pow(targetY - currentY, 2)) < 0.5;
        }
    }

    // the "state" of the robot
    // the velocity, angle, and position of the robot
    static class State {
        double fieldX;
        double fieldY;
        double angle;
        double velX;
        double velY;
        double angleVel;
    }
}
