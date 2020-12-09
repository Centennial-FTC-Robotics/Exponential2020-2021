package org.exponential.mechanisms.parametricEQ;

public class CubicSpline extends ParametricEq{

    @Override
    public State getStateAtTime(double t) {
        return null;
    }

    @Override
    public boolean moveOn(double currentX, double currentY, double currentAngle, double velX, double velY, double velAngle) {
        return false;
    }
}
