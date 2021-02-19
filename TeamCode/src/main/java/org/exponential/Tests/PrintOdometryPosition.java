package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.mechanisms.ArcOdometry;
import org.exponential.mechanisms.IMU;
import org.exponential.mechanisms.ImprovedArcOdometry;
import org.exponential.mechanisms.Odometry;
import org.exponential.superclasses.ExpoOpMode;


@TeleOp
public class PrintOdometryPosition extends ExpoOpMode {

    @Override
    public void run() throws InterruptedException {
        Odometry oldOdometry = new Odometry(imu);
        oldOdometry.initialize(this);

        Odometry noWeightedOld = new Odometry(imu){
            @Override
            public double weightedAverage(int leftEncChange, int rightEncChange) {
                return (leftEncChange+rightEncChange)/2.0;
            }
        };
        noWeightedOld.initialize(this);

        Odometry noWeightedNew = new ArcOdometry(imu){
            @Override
            public double weightedAverage(int leftEncChange, int rightEncChange) {
                return (leftEncChange+rightEncChange)/2.0;
            }
        };

        noWeightedNew.initialize(this);

        Odometry improved = new ImprovedArcOdometry(imu);

        improved.initialize(this);

        while(opModeIsActive()){

            setDrivetrainMotorToGamepad();

            telemetry.addData("old odometry with average: ", "");
            telemetry.addData("x: ", oldOdometry.getxPos());
            telemetry.addData("y: ", oldOdometry.getyPos());
            telemetry.addData("angle: ", oldOdometry.getAngle());
            oldOdometry.update();

            telemetry.addData("arc odometry with average: ", "");
            telemetry.addData("x: ", odometry.getxPos());
            telemetry.addData("y: ", odometry.getyPos());
            telemetry.addData("angle: ", odometry.getAngle());
            odometry.update();

            telemetry.addData("improved odometry with average: ", "");
            telemetry.addData("x: ", improved.getxPos());
            telemetry.addData("y: ", improved.getyPos());
            telemetry.addData("angle: ", improved.getAngle());
            improved.update();


/*
            telemetry.addData("old odometry without average: ", "");
            telemetry.addData("x: ", noWeightedOld.getxPos());
            telemetry.addData("y: ", noWeightedOld.getyPos());
            telemetry.addData("angle: ", noWeightedOld.getAngle());
            noWeightedOld.update();

            telemetry.addData("arc odometry without average: ", "");
            telemetry.addData("x: ", noWeightedNew.getxPos());
            telemetry.addData("y: ", noWeightedNew.getyPos());
            telemetry.addData("angle: ", noWeightedNew.getAngle());
            noWeightedNew.update();
*/



            telemetry.update();
        }
    }
}
