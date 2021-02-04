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
import org.exponential.mechanisms.TurretContinuous;
import org.exponential.mechanisms.WobbleGoalMover;
import org.exponential.superclasses.Robot;

import java.util.List;

import static org.exponential.utility.Utility.sleep;

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
    LinearOpMode opMode;
    public OurRobot(CameraOpenCV camera) {
        this.camera = camera;
    }

    @Override
    public void initialize(LinearOpMode opMode) {
        List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.opMode = opMode;
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

        //setUpServos();
  /*      //right odometry: intake motor
        odometry.setEncoders(opMode.hardwareMap.get(DcMotorEx.class, "leftOdometry"),
                opMode.hardwareMap.get(DcMotorEx.class, "backOdometry"),
                intake.intakeMotor);*/
    }

    public void setUpServos() {
        intake.setServoPositions();
        wobbleGoalMover.raise();
        wobbleGoalMover.clamp();
    }

    public void savePositions() {
        odometry.savePosition();
        turret.savePosition();
    }

    public void loadPositions() {
        odometry.loadPosition();
        turret.loadPosition();
    }

    public void shootPowerShotTargets(String side) {
        double[] targetXPositions;
        if (side.equals("red")) {
            targetXPositions = new double[] {2, 9.5, 17};
            //targetXPositions = new double[] {2 + 1.75, 9.5 + 2.75, 17 + 4.5};
        } else {
            targetXPositions = new double[] {-2, -9.5, -17};
        }
        //move to some position on field to start shooting
        drivetrain.moveTo(12, -6, odometry.getAngle());
        drivetrain.performBrake();

        turret.pointAtTarget();

        /*double robotX = odometry.getxPos();
        double robotY = odometry.getyPos();*/

        boolean first = true;
        //LogMaker logMaker = new LogMaker("powershotlog.txt");
        for (double targetXPosition: targetXPositions) {
            // +180 because we want the robot's front to be facing the exact opposite of the targets.
            //robot shoots backwards
            //double targetAngle = 180 + Math.toDegrees(Math.atan2(72 - robotY, targetXPosition - robotX));
            turret.setTarget(targetXPosition, 72);
            turret.readjustTurretAngle();
            opMode.sleep(250);
            if (first) {
                shooter.shootAtPowerShot();
                first = false;
            } else {
                shooter.powerShotReadjustPower();
            }
            opMode.sleep(1000);
            loader.loadAndUnload();

        }
        //logMaker.close();
        turret.setToReloadPosition();
        shooter.stopShooting();
    }

    public void shootAtHighGoal(String side) {
        double goalXPosition;
        if (side.equals("red")) {
            goalXPosition = 36;
        } else {
            goalXPosition = -36;
        }

        turret.setTarget(goalXPosition, 72);
        //move to some position on field to start shooting
        drivetrain.moveTo(goalXPosition, -6, odometry.getAngle());
        drivetrain.performBrake();

        turret.pointAtTarget();
        opMode.sleep(250);

        /*double robotX = odometry.getxPos();
        double robotY = odometry.getyPos();*/

        // double targetAngle = 180 + Math.toDegrees(Math.atan2(72 - robotY, goalXPosition - robotX));
        //drivetrain.turnTo(targetAngle, 4);

        boolean first = true;
        for (int i = 0; i < 3; i++) {
            if (first) {
                shooter.shootAtHighGoal();
                first = false;
            } else {
                shooter.highGoalReadjustPower();
            }
            sleep(1000);
            loader.loadAndUnload();
        }
        turret.setToReloadPosition();
        shooter.stopShooting();
    }

    public void scoreWobbleGoal(String side) {
        double targetXPosition;
        if (side.equals("red")) {
            targetXPosition = 36;
        } else {
            targetXPosition = -36;
        }

        //move to edge of wall facing left
        drivetrain.moveTo(targetXPosition, -63, 180);

        //drop goal
        wobbleGoalMover.placeGoal();
    }
}
