package org.exponential.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.ArcOdometry;
import org.exponential.mechanisms.CameraOpenCV;
import org.exponential.mechanisms.Drivetrain;
import org.exponential.mechanisms.IMU;
import org.exponential.mechanisms.Intake;
import org.exponential.mechanisms.Loader;
import org.exponential.mechanisms.Odometry;
import org.exponential.mechanisms.Shooter;
import org.exponential.mechanisms.Turret;
import org.exponential.mechanisms.WobbleGoalMover;
import org.exponential.superclasses.Robot;

import java.util.List;

public class OurRobot implements Robot {
    public Drivetrain drivetrain;
    public Odometry odometry;
    public IMU imu;
    public CameraOpenCV camera;
    public Intake intake;
    public Loader loader;
    public Shooter shooter;
    public WobbleGoalMover wobbleGoalMover;
    public Turret turret;
    public OurRobot() {
        camera = new CameraOpenCV();
    }

    public OurRobot(CameraOpenCV camera) {
        this.camera = camera;
    }

    @Override
    public void initialize(LinearOpMode opMode) {
        List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        // camera = new CameraOpenCV();
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
        /*odometry = new ArcOdometry(imu, opMode.hardwareMap.get(DcMotorEx.class, "leftOdometry"),
                opMode.hardwareMap.get(DcMotorEx.class, "frontLeft"),
                intake.intakeMotor);*/
        odometry = new ArcOdometry(imu);
        odometry.initialize(opMode);

        drivetrain = new Drivetrain(odometry);
        drivetrain.initialize(opMode);

        turret = new Turret(drivetrain);
        turret.initialize(opMode);
  /*      //right odometry: intake motor
        odometry.setEncoders(opMode.hardwareMap.get(DcMotorEx.class, "leftOdometry"),
                opMode.hardwareMap.get(DcMotorEx.class, "backOdometry"),
                intake.intakeMotor);*/


    }
}
