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
import org.exponential.mechanisms.parametricEQ.CubicSpline;
import org.exponential.mechanisms.parametricEQ.State;
import org.exponential.superclasses.Robot;

import java.util.ArrayList;
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
            targetXPositions = new double[]{3, 10.5, 18.75};
        } else {
            targetXPositions = new double[]{-3, -10.5, -18.75};
        }

        for (double targetXPosition : targetXPositions) {
            shooter.shootAtPowerShot();  //going to assume that the initial move will be long enough to rev up the motor enough

            drivetrain.moveTo(targetXPosition, -6, 270, 2);
            drivetrain.performBrake();
            loader.loadAndUnload();
        }
        shooter.stopShooting();
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
        drivetrain.performBrake();
        for (int i = 0; i < 2; i++) {
            loader.loadAndUnload();
            opMode.sleep(150);
        }
        shooter.readjustHighGoalPower();
        loader.loadAndUnload();
        loader.resting();
        turret.pointToReloadPosition();
    }

    public double headingRotation(double targetAngle) {
        double kP = .013;
        double rotationPower = 0;
        double tolerance = .5;
        odometry.update();
        if (Math.abs(IMU.normalize(targetAngle - IMU.normalize(odometry.getAngle()))) > tolerance) {
            rotationPower = -kP * (IMU.normalize(targetAngle - IMU.normalize(odometry.getAngle())));
        }
        return rotationPower;
    }

    public void turnTurretFromCurrent (double xOffset, double yOffset) {
        odometry.update ();
        turret.setTarget (odometry.getxPos() + xOffset, odometry.getyPos() + yOffset);
        turret.readjustTurretAngle();
        turret.pointAtTarget();
        turret.readjustTurretAngle();
    }



    public void turretShootThree () {
        loadAndUnloadAllRings();
        opMode.sleep(100);
        turret.pointToReloadPosition();
    }

    public void turretShootOne () {
        drivetrain.performBrake();
        loader.loadAndUnload();
    }

    //These methods are to shoot at different powers depending on distance
    public void shootThreeFromDistance(double motorPower) {
        for (int i = 0; i < 2; i++) {
            loader.loadAndUnload();
            opMode.sleep(70);
        }
        shooter.adjustedReadjustHighGoal(motorPower);
        loader.loadAndUnload();
    }

    //Start at the wall and automaticall moves to shoot the first power shot
    public void shootPowerShotTeleOp() {
        shooter.shootAtPowerShot();
        double xPos = odometry.getxPos();
        double yPos = odometry.getyPos();
        ArrayList<CubicSpline.CubicSplinePoint> spline = new ArrayList<CubicSpline.CubicSplinePoint>();
        // starting on rightmost red tape, facing left
        State state = new State();
        state = new State();
        state.fieldX = xPos;
        state.fieldY = yPos;
        state.angle = 270;
        state.velX = 0;
        state.velY = 0;
        state.angleVel = 0;
        spline.add(new CubicSpline.CubicSplinePoint(state, 0));

        //Shoot left Power Shot
        state = new State();
        state.fieldX = xPos + 20;
        state.fieldY = yPos;
        state.angle = 270;
        state.velX = 3;
        state.velY = 0;
        state.angleVel = 5;
        spline.add(new CubicSpline.CubicSplinePoint(state, 3));
        ((DriveTrainParametric)(drivetrain)).moveAlongParametricEq(new CubicSpline(spline));

        drivetrain.turnTo(270,.5);
        loader.loadAndUnload();
    }

    //Used after shooting the first power shot to shoot the middle, then the right power shots.
    public void shootOtherPowerShot() {
        shooter.shootAtPowerShot();
        double xPos = odometry.getxPos();
        double yPos = odometry.getyPos();
        ArrayList<CubicSpline.CubicSplinePoint> spline = new ArrayList<CubicSpline.CubicSplinePoint>();
        // starting on rightmost red tape, facing left
        State state = new State();
        state = new State();
        state.fieldX = xPos;
        state.fieldY = yPos;
        state.angle = 270;
        state.velX = 3;
        state.velY = 0;
        state.angleVel = 1;
        spline.add(new CubicSpline.CubicSplinePoint(state, 0));

        //Shoot left Power Shot
        state = new State();
        state.fieldX = xPos + 7;
        state.fieldY = yPos;
        state.angle = 270;
        state.velX = 4;
        state.velY = .5;
        state.angleVel = 3;
        spline.add(new CubicSpline.CubicSplinePoint(state, 1.3));
        ((DriveTrainParametric)(drivetrain)).moveAlongParametricEq(new CubicSpline(spline));

        loader.loadAndUnload();
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
