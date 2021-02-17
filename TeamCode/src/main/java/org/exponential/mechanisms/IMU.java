package org.exponential.mechanisms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.superclasses.Mechanism;

public class IMU implements Mechanism {

    BNO055IMU imu;
    public double angle;

    @Override
    public void initialize(LinearOpMode opMode) {
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        while (opMode.opModeIsActive() && !imu.isGyroCalibrated()) ;
        lastIMUReading = imu.getAngularOrientation().firstAngle;
    }


    public static double normalize(double angle) {
        // returns the angle geometrically equivalent to the input angle but between -180 and 180
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        }
        if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    private double lastIMUReading;

    public void update() {
        double newAngle = imu.getAngularOrientation().firstAngle;
        angle += normalize(newAngle - lastIMUReading);
        lastIMUReading = newAngle;
    }
}
