package org.exponential.mechanisms;

public class ArcOdometry extends Odometry {
    public ArcOdometry(IMU imu) {
        super(imu);
    }

    public void update(double timeElapsed) {
        updateTimer.reset();

        // updates position, velocity, and angle according to how much time has elapsed

        int leftEncChange = forwardLeftEnc.getCurrentPosition() - lastLeftEncPos;
        int rightEncChange = forwardRightEnc.getCurrentPosition() - lastRightEncPos;
        int horiEncChange = horizontalEnc.getCurrentPosition() - lastHoriEncPos;

        // does not call getCurrentPosition a second time because you would not account for encoder
        // readings from the time between the two calls
        lastLeftEncPos += leftEncChange;
        lastRightEncPos += rightEncChange;
        lastHoriEncPos += horiEncChange;

        // updates angle
        imu.update();
        double changeInAngle = imu.angle - angle;
        angleVel = changeInAngle / timeElapsed;

        // currently in robot centric
        double[] changeInPos;

        if (changeInAngle == 0) {
            changeInPos = new double[]{
                    encToInch(horiEncChange - horiEncPerDegree * changeInAngle),
                    encToInch((leftEncChange + rightEncChange) / 2)
            };
        } else {
            // I know there's supposed to be an abs, just don't worry about it
            double radius = (encToInch(leftEncChange + rightEncChange) / (2 * Math.toRadians(changeInAngle)));

            changeInPos = new double[]{
                    encToInch(radius * (1 - Math.cos(Math.toRadians(changeInAngle)))),
                    encToInch(radius * Math.sin(Math.toRadians(changeInAngle)))
            };
        }

        // changes to field centric displacement vector
        changeInPos = toFieldCentric(changeInPos[0], changeInPos[1], imu.angle - changeInAngle);

        xPos += changeInPos[0];
        yPos += changeInPos[1];
        xVel = changeInPos[0] / timeElapsed;
        yVel = changeInPos[1] / timeElapsed;
        angle += changeInAngle;
        angleVel = changeInAngle / timeElapsed;
    }

    public double[] toFieldCentric(double robotX, double robotY, double angleBetween) {
        double angleRad = Math.toRadians(angleBetween);
        double centricX = robotX * Math.cos(angleRad - Math.PI / 2) - robotY * Math.sin(angleRad - Math.PI / 2);
        double centricY = robotY * Math.cos(angleRad - Math.PI / 2) + robotX * Math.sin(angleRad - Math.PI / 2);

        return new double[]{centricX, centricY};
    }
}