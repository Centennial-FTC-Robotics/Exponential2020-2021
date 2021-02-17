package org.exponential.superclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.mechanisms.Drivetrain;
import org.exponential.mechanisms.IMU;
import org.exponential.mechanisms.Intake;
import org.exponential.mechanisms.Loader;
import org.exponential.mechanisms.Odometry;
import org.exponential.mechanisms.Shooter;
import org.exponential.mechanisms.Turret;
import org.exponential.mechanisms.WobbleGoalMover;
import org.exponential.robots.OurRobot;

public abstract class ExpoOpMode extends LinearOpMode {
    public OurRobot expo = new OurRobot();
    public Odometry odometry = expo.odometry;
    public Drivetrain drivetrain = expo.drivetrain;
    public IMU imu = expo.imu;
    public Intake intake = expo.intake;
    public Loader loader = expo.loader;
    public Shooter shooter = expo.shooter;
    public Turret turret = expo.turret;
    public WobbleGoalMover wobbleGoalMover = expo.wobbleGoalMover;

    public void sleepMilliseconds(int milli){
        ElapsedTime timer = new ElapsedTime();
        while(milli > timer.milliseconds()){
            odometry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        expo.initialize(this);
        waitForStart();
    }

    public abstract void run() throws InterruptedException;
}
