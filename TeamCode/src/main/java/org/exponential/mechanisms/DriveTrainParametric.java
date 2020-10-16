package org.exponential.mechanisms;

public class DriveTrainParametric extends Drivetrain {
    // idea is to have drivetrain methods that use feedforward-feedback on a parametric function
    // with motion profiling

    // time-differentiable vector-valued parametric equation for the position of our robot as a function of time
    static abstract class ParametricEq {

        // returns what the state of the robot should at time t
        public abstract State getStateAtTime(double t);

        // tells whether to end the strafe
        public abstract boolean moveOn(double currentX, double currentY, double currentAngle,
                                       double velX, double velY, double velAngle);
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

    public void moveAlongParametricEq(ParametricEq equation) {
        while (opMode.opModeIsActive() && !equation.moveOn(positioning.xPos, positioning.yPos,
                positioning.angle, positioning.xVel, positioning.yVel, positioning.angleVel)) {
            // while opmode is active and the equation doesn't tell us to move on yet
            

        }
    }
}
