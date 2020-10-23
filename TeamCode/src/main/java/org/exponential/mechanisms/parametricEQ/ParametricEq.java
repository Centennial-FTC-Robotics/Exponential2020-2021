package org.exponential.mechanisms.parametricEQ;

import org.exponential.mechanisms.DriveTrainParametric;

// time-differentiable vector-valued parametric equation for the position of our robot as a function of time
public abstract class ParametricEq {
    // returns what the state of the robot should at time t
    public abstract State getStateAtTime(double t);

    // tells whether to end the strafe
    public abstract boolean moveOn(
            double currentX, double currentY, double currentAngle,
            double velX, double velY, double velAngle);
}