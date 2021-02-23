package org.exponential.mechanisms;

public class ImprovedArcOdometry extends Odometry {

    public ImprovedArcOdometry(IMU imu) {
        super(imu);
    }

    public double[] rotateCounter(double[] coordinates, double angleDeg) {
        double angleRad = Math.toRadians(angleDeg);
        double newX = coordinates[0] * Math.cos(-angleRad) - coordinates[1] * Math.sin(-angleRad);
        double newY = coordinates[1] * Math.cos(-angleRad) + coordinates[0] * Math.sin(-angleRad);
        return new double[]{newX, newY};
    }

    public void update(double timeElapsed, double changeInAngle, int leftEncChange, int rightEncChange, int horiEncChange, boolean imuAngleUpdate) {
        if (changeInAngle == 0/* || imuAngleUpdate*/) {
            super.update(timeElapsed, changeInAngle, leftEncChange, rightEncChange, horiEncChange, imuAngleUpdate);
            return;
        }
        double average = weightedAverage(leftEncChange, rightEncChange);
        double horiEncAdjusted = horiEncChange - changeInAngle * super.horiEncPerDegree;
        double arcLength = Math.sqrt(Math.pow(average, 2) + Math.pow(horiEncAdjusted, 2));
        double alpha = Math.toDegrees(Math.atan2(horiEncAdjusted, average)); // positive alpha to the left of heading
        double[] displacement = {arcLength * (Math.cos(Math.toRadians(changeInAngle)) - 1) / Math.toRadians(changeInAngle),
                arcLength * Math.sin(Math.toRadians(changeInAngle)) / Math.toRadians(changeInAngle)};
        displacement = rotateCounter(displacement, alpha); // rotate to robot centric
        displacement = toFieldCentric(displacement[0], displacement[1]); // rotate to field centric
        displacement[0] = encToInch(displacement[0]);
        displacement[1] = encToInch(displacement[1]);
        xPos += displacement[0];
        yPos += displacement[1];
        xVel = displacement[0] / timeElapsed;
        yVel = displacement[1] / timeElapsed;
        angle += changeInAngle;
        angleVel = changeInAngle / timeElapsed;
    }
}
