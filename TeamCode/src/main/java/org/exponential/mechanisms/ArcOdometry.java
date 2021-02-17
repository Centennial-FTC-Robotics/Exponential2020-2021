package org.exponential.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class ArcOdometry extends Odometry {
    /*public ArcOdometry(IMU imu, DcMotorEx leftEncoder, DcMotorEx backEncoder, DcMotorEx rightEncoder) {
        super(imu, leftEncoder, backEncoder, rightEncoder);
    }*/

    public ArcOdometry(IMU imu) {
        super(imu);
    }
    public void update(double timeElapsed, double changeInAngle, int leftEncChange, int rightEncChange, int horiEncChange, boolean imuAngleUpdate) {
        // currently in robot centric
        double[] changeInPos;

        if (changeInAngle == 0) {
            changeInPos = new double[]{
                    encToInch(horiEncChange),
                    encToInch((leftEncChange + rightEncChange) / 2)
            };
        } else {
            double radius = Math.abs(encToInch(leftEncChange + rightEncChange) / (2 * Math.toRadians(changeInAngle)));
            if (leftEncChange + rightEncChange > 0) {
                if (changeInAngle > 0) {
                    changeInPos = new double[]{
                            (radius * (Math.cos(Math.toRadians(Math.abs(changeInAngle))) - 1)),
                            (radius * Math.sin(Math.toRadians(Math.abs(changeInAngle))))
                    };
                } else {
                    changeInPos = new double[]{
                            (-radius * (Math.cos(Math.toRadians(Math.abs(changeInAngle))) - 1)),
                            (radius * Math.sin(Math.toRadians(Math.abs(changeInAngle))))
                    };
                }
            } else {
                if (changeInAngle > 0) {
                    changeInPos = new double[]{
                            (radius * (Math.cos(Math.toRadians(Math.abs(changeInAngle))) - 1)),
                            (-radius * Math.sin(Math.toRadians(Math.abs(changeInAngle))))
                    };
                } else {
                    changeInPos = new double[]{
                            (-radius * (Math.cos(Math.toRadians(Math.abs(changeInAngle))) - 1)),
                            (-radius * Math.sin(Math.toRadians(Math.abs(changeInAngle))))
                    };
                }
            }


            radius = Math.abs(encToInch(horiEncChange - horiEncPerDegree * changeInAngle) / Math.toRadians(changeInAngle));
            if (changeInAngle > 0) {
                if (horiEncChange - horiEncPerDegree * changeInAngle > 0) {
                    changeInPos[0] += radius * Math.sin(Math.toRadians(Math.abs(changeInAngle)));
                    changeInPos[1] += radius * (1 - Math.cos(Math.toRadians(Math.abs(changeInAngle))));

                } else {
                    changeInPos[0] += -radius * Math.sin(Math.toRadians(Math.abs(changeInAngle)));
                    changeInPos[1] += -radius * (1 - Math.cos(Math.toRadians(Math.abs(changeInAngle))));
                }
            } else if (changeInAngle < 0) {
                if (horiEncChange - horiEncPerDegree * changeInAngle > 0) {
                    changeInPos[0] += radius * Math.sin(Math.toRadians(Math.abs(changeInAngle)));
                    changeInPos[1] += -radius * (1 - Math.cos(Math.toRadians(Math.abs(changeInAngle))));
                } else {
                    changeInPos[0] += -radius * Math.sin(Math.toRadians(Math.abs(changeInAngle)));
                    changeInPos[1] += radius * (1 - Math.cos(Math.toRadians(Math.abs(changeInAngle))));
                }
            }
        }


        // opMode.telemetry.addData("delta angle: ", changeInAngle);
        // opMode.telemetry.addData("delta x: ", changeInPos[0]);
        // opMode.telemetry.addData("delta y: ", changeInPos[1]);

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