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
    public Odometry odometry;
    public Drivetrain drivetrain;
    public IMU imu;
    public Intake intake;
    public Loader loader;
    public Shooter shooter;
    public Turret turret;
    public WobbleGoalMover wobbleGoalMover;

    public void sleepMilliSeconds(int milli){
        ElapsedTime timer = new ElapsedTime();
        while(milli > timer.milliseconds()){
            odometry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        expo.initialize(this);

        odometry = expo.odometry;
        Drivetrain drivetrain = expo.drivetrain;
        imu = expo.imu;
        intake = expo.intake;
        loader = expo.loader;
        shooter = expo.shooter;
        turret = expo.turret;
        wobbleGoalMover = expo.wobbleGoalMover;
        waitForStart();
        run();
    }

    public void setDrivetrainMotorToGamepad(){
        expo.drivetrain.setPowerFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
    }

    public abstract void run() throws InterruptedException;
}
