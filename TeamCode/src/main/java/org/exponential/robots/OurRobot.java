package org.exponential.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.ArcOdometry;
import org.exponential.mechanisms.CameraOpenCV;
import org.exponential.mechanisms.DriveTrainParametric;
import org.exponential.mechanisms.Drivetrain;
import org.exponential.mechanisms.IMU;
import org.exponential.mechanisms.ImprovedArcOdometry;
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

        loader = new Loader();
        loader.initialize(opMode);

        shooter = new Shooter();
        shooter.initialize(opMode);

        wobbleGoalMover = new WobbleGoalMover();
        wobbleGoalMover.initialize(opMode);

        imu = new IMU();
        imu.initialize(opMode);

        odometry = new ImprovedArcOdometry(imu);
        odometry.initialize(opMode);

        drivetrain = new DriveTrainParametric(odometry);
        drivetrain.initialize(opMode);

        turret = new Turret(drivetrain);
        turret.initialize(opMode);

        intake = new Intake();
        intake.initialize(opMode);
    }

    public void setUpServos() {
        intake.setServoPositions();
        wobbleGoalMover.raise();
        wobbleGoalMover.clamp();
    }

    public void savePositions() {
        odometry.savePosition();
        //turret.savePosition();
    }

    public void loadPositions() {
        odometry.loadPosition();
        //turret.loadPosition();
    }

    public void shootAtPowerShotTargets(String side) {
        double[] targetXPositions;
        if (side.equals("red")) {
            targetXPositions = new double[]{4, 11.5, 19};
        } else {
            targetXPositions = new double[]{-2, -9.5, -17};
        }

        for (double targetXPosition : targetXPositions) {
            shooter.shootAtPowerShot();  //going to assume that the initial move will be long enough to rev up the motor enough

            drivetrain.moveTo(targetXPosition, -6, 270, 2);
            drivetrain.performBrake();
            loader.loadAndUnload();
        }
        shooter.stopShooting();
    }

    public void shootPowerShotTargetsTurret(String side) {
        double turretTargetEncoders[];
        shooter.shootAtPowerShot();
        if (side.equals("red")) {
            drivetrain.moveTo(12, -6, 270);
            turretTargetEncoders = new double[]{turret.encCountAtAngleZero - 100,
                    turret.encCountAtAngleZero,
                    turret.encCountAtAngleZero + 100};
        } else {
            turretTargetEncoders = new double[]{};
        }
        for (double targetEncoderValue : turretTargetEncoders) {
            turret.turretMotor.setTargetPosition((int) targetEncoderValue);
            turret.turretMotor.setPower(1);
            opMode.sleep(500);
            loader.loadAndUnload();
        }
    }

    public void shootAtHighGoal(String side) {
        double goalXPosition;
        if (side.equals("red")) {
            goalXPosition = 36;
        } else {
            goalXPosition = -36;
        }

        double robotX = odometry.getxPos();
        double robotY = odometry.getyPos();

        //SET SHOOTER TO ITS INITIAL POWER
        shooter.shootAtHighGoal();

        //move to some position on field to start shooting
        //double targetAngle = Math.toDegrees(Math.atan2(-6 - robotY, goalXPosition - robotX));
        drivetrain.moveTo(goalXPosition, -6, 270, 1.75);
        drivetrain.performBrake();

        loadAndUnloadAllRings();
        /*boolean first = true;
        for (int i = 0; i < 3; i++) {
            if (first) {
                first = false;
            } else {
                shooter.readjustHighGoalPower();
                sleep(250);
            }
            loader.loadAndUnload();
        }*/
        //turret.pointToReloadPosition();
        shooter.stopShooting();
    }

    public void scoreWobbleGoal(String side) {
        wobbleGoalMover.raise();
        double targetXPosition;
        if (side.equals("red")) {
            targetXPosition = 36;
        } else {
            targetXPosition = -36;
        }

        //move to edge of wall facing left
        drivetrain.moveTo(targetXPosition, -61, 180);

        //drop goal
        //wobbleGoalMover.placeGoal();
        wobbleGoalMover.release();
    }

    public void loadAndUnloadAllRings() {
        for (int i = 0; i < 2; i++) {
            loader.loadAndUnload();
            opMode.sleep(210);
        }
        shooter.readjustHighGoalPower();
        loader.loadAndUnload();
    }

    public double headingRotation(double targetAngle) {
        double kP = .06;
        double rotationPower = 0;
        double tolerance = 5;
        if (Math.abs(targetAngle - IMU.normalize(odometry.getAngle())) > tolerance) {
            rotationPower = kP * (targetAngle - IMU.normalize(odometry.getAngle()));
        }
        return rotationPower;
    }

    //These methods are to shoot at different powers depending on distance
    public void shootThreeFromDistance(double motorPower) {
        for (int i = 0; i < 2; i++) {
            loader.loadAndUnload();
            opMode.sleep(100);
        }
        shooter.adjustedReadjustHighGoal(motorPower);
        loader.loadAndUnload();
    }

    public void shootAtHighGoalFromDistance (String side) {
        final double goalHeight = 0.91;
        double goalXPosition;
        double motorPower;

        if (side.equals("red")) {
            goalXPosition = 36;
        } else {
            goalXPosition = -36;
        }

        double robotX = odometry.getxPos();
        double robotY = odometry.getyPos();

        motorPower = distanceAdjustedPower(goalXPosition, goalHeight);

        shooter.adjustedHighGoal(motorPower);

        shootThreeFromDistance(motorPower);
        shooter.stopShooting();
    }

    public double distanceAdjustedPower (double targetX, double targetHeight) {
        //Constants defined by our robot design and the Earth
        final double gravity = -9.8;
        final double theta = Math.toRadians(33);
        final double radiusOfFlywheel = 0.1115;

        //Getting robot position
        double robotY = odometry.getxPos();
        double robotX = odometry.getyPos();

        //Finding distance from the goal
        double deltaZ = targetHeight - 0.24;
        double deltaX = 0.0254 * (Math.sqrt(Math.pow(targetX - robotX,2) + Math.pow(72 - robotY,2)));

        //Finding Launching velocity of the ring
        double velInitial = Math.sqrt((gravity * Math.pow(deltaX,2) / (2 * Math.pow(Math.cos(theta),2) * (deltaZ - deltaX*Math.tan(theta)))));
        //Converting launching velocity to angular velocity
        double omega = velInitial/radiusOfFlywheel;
        //Converting angular velocity into motor power
        double adjustedMotorPower = omega/(20 * Math.PI);
        return adjustedMotorPower;
    }
}
