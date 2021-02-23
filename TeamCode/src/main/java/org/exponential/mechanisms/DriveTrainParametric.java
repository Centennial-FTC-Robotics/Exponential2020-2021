package org.exponential.mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.exponential.mechanisms.parametricEQ.ParametricEq;
import org.exponential.mechanisms.parametricEQ.State;
import org.exponential.mechanisms.parametricEQ.StraightLine;

import java.util.HashMap;

public class DriveTrainParametric extends Drivetrain {
    // idea is to have drivetrain methods that use feedforward-feedback on a parametric function
    // with motion profiling

    // feedforward constants
    public final static double xVelMax = 40; // inch per sec per motor power
    public final static double yVelMax = 48.96; // inch per sec per motor power
    public final static double rotate = 380; // degrees per sec per motor power
    public final static double xAccelMax = 1230.392; // inch per sec^2
    public final static double yAccelMax = 2290.862; // inch per sec^2

    // feedback constants
    private final static double KPBACK = 7;
    private final static double KIBACK = 1.5;
    private final static double KDBACK = 0;
    private final static double KPBACKANGLE = 10;
    private final static double KIBACKANGLE = 2;
    private final static double KDBACKANGLE = 0;

    public DriveTrainParametric(Odometry positioning) {
        super(positioning);
    }

    public void moveAlongParametricEq(ParametricEq equation) {

        // a lot of random PID variables
        double areaDis = 0;
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
            velX = disX / intervalTime;
            velY = disY / intervalTime;
            double distance = Math.sqrt(Math.pow(disX, 2) + Math.pow(disY, 2));
            if(disX*targetState.velX+disY*targetState.velY > 0){
                // robot is behind the target position
                areaDis+=intervalTime*distance;

            } else {
                // robot is in front of the target position
                areaDis = 0;
            }
            areaDis += distance * intervalTime;

            // Angle PID calculations
            disAngle = IMU.normalize(targetState.angle - positioning.angle);
            areaAngle = disAngle * intervalTime;
            velAngle = disAngle / intervalTime;
            if (Math.abs(disAngle) > 90) {
                areaAngle = Range.clip(areaAngle, -1, 1);
            }
            if(Math.abs(disAngle)<5){
                areaAngle = Range.clip(areaAngle, -0.5, 0.5);
            }


            // Feedback portion
            double xNew = KPBACK * disX + KDBACK * velX;
            double yNew = KPBACK * disY + KDBACK * velY;
            double angleNew = KPBACKANGLE * disAngle + KIBACKANGLE * areaAngle + KDBACKANGLE * velAngle;
            if (distance > 0) {
                // prevents a divide by 0
                xNew += KIBACK * areaDis * (disX) / distance;
                yNew += KIBACK * areaDis * (disY) / distance;
            }

            // Feedforward portion
            xNew += targetState.velX;
            yNew += targetState.velY;
            angleNew += targetState.angleVel;

            // sets the velocity using encapsulated velocity method that uses motion profiling
            setVel(xNew, yNew, angleNew);

            opMode.telemetry.addData("X: ", positioning.xPos);
            opMode.telemetry.addData("Y: ", positioning.yPos);
            opMode.telemetry.addData("Angle: ", positioning.angle);
            opMode.telemetry.addData("Area: ", areaDis);
            opMode.telemetry.addData("Area Angle: ", areaAngle);
            opMode.telemetry.addData("Dis X: ", disX);
            opMode.telemetry.addData("Dis Y: ", disY);
            opMode.telemetry.addData("Dis Angle: ", disAngle);
            opMode.telemetry.addData("Distance: ", distance);


            opMode.telemetry.update();
        }
        State targetState = equation.getStateAtTime(timer.seconds());
        super.moveTo(targetState.fieldX, targetState.fieldY, targetState.angle);
        performBrake();

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
        ParametricEq line = new StraightLine(x, y, angle, positioning.xPos, positioning.yPos, positioning.angle, portionOfMaxAccel);
        moveAlongParametricEq(line);
    }
}