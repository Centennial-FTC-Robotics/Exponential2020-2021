package org.exponential.mechanisms.parametricEQ;


import org.exponential.mechanisms.DriveTrainParametric;
import org.exponential.mechanisms.IMU;

public class StraightLine extends ParametricEq {
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
    private double startAngle;


    private boolean hitsMaxSpeed;
    private double accelTime;
    private double totalTime;

    private final double xAccelMax = DriveTrainParametric.xAccelMax;
    private final double yAccelMax = DriveTrainParametric.yAccelMax;

    private final double xVelMax = DriveTrainParametric.xVelMax;
    private final double yVelMax = DriveTrainParametric.yVelMax;


    public StraightLine(double targetX, double targetY, double targetAngle,
                        double startX, double startY, double startAngle,
                        double portion /* fraction of the fastest possible straight path*/) {
        // assumes that the robot starts at velocity of 0 and ends at a velocity of 0

        this.startX = startX;
        this.startY = startY;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        this.startAngle = startAngle;


        // angle of displacement vector from the robot's perspective
        double robotDisplacementAngle = Math.toDegrees(Math.atan2(targetY - startY, targetX - startX)) - targetAngle + 90;
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
        accel = rotateCounterClock(accel[0], accel[1], Math.toRadians(targetAngle) - Math.PI / 2);

        xFieldAccel = accel[0];
        yFieldAccel = accel[1];
        maxVel = rotateCounterClock(maxVel[0], maxVel[1], Math.toRadians(targetAngle) - Math.PI / 2);
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

        // sets the rotation velocity (not really that mathematically correct, more like eyeballing it)
        if(Math.abs(IMU.normalize(targetAngle-startAngle)) / DriveTrainParametric.rotate > t){
            if(IMU.normalize(targetAngle-startAngle)>0){
                theState.angleVel = DriveTrainParametric.rotate/2;
            } else {
                theState.angleVel = -DriveTrainParametric.rotate/2;
            }
        } else {
            theState.angleVel = 0;
        }
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
        return Math.sqrt(Math.pow(targetX - currentX, 2) + Math.pow(targetY - currentY, 2)) < 1;
    }


    public static double[] rotateCounterClock(double x, double y, double angleRad) {
        double[] rotated = new double[2];
        rotated[0] = x * Math.cos(angleRad) - y * Math.sin(angleRad);
        rotated[1] = y * Math.cos(angleRad) + x * Math.sin(angleRad);
        return rotated;


    }
}
