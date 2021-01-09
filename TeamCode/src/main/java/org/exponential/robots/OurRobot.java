package org.exponential.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.ArcOdometry;
import org.exponential.mechanisms.Camera;
import org.exponential.mechanisms.Drivetrain;
import org.exponential.mechanisms.IMU;
import org.exponential.mechanisms.Intake;
import org.exponential.mechanisms.Loader;
import org.exponential.mechanisms.Odometry;
import org.exponential.mechanisms.Shooter;
import org.exponential.mechanisms.WobbleGoalMover;
import org.exponential.superclasses.Robot;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class OurRobot implements Robot {
    public Drivetrain drivetrain;
    public Odometry odometry;
    public IMU imu;
    public Camera camera;
    public Intake intake;
    public Loader loader;
    public Shooter shooter;
    public WobbleGoalMover wobbleGoalMover;
    @Override
    public void initialize(LinearOpMode opMode) {
        List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        odometry = new ArcOdometry(imu);
        odometry.initialize(opMode);
        drivetrain = new Drivetrain(odometry);
        drivetrain.initialize(opMode);
        camera = new Camera();
        camera.initialize(opMode);
        intake = new Intake();
        intake.initialize(opMode);
        loader = new Loader();
        loader.initialize(opMode);
        shooter = new Shooter();
        shooter.initialize(opMode);
        wobbleGoalMover = new WobbleGoalMover();
        wobbleGoalMover.initialize(opMode);
        imu = new IMU();
        imu.initialize(opMode);
    }
}
