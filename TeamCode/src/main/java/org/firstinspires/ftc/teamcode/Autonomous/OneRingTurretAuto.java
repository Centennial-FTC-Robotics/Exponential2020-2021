package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.CameraOpenCV;
import org.exponential.mechanisms.DriveTrainParametric;
import org.exponential.mechanisms.parametricEQ.CubicSpline;
import org.exponential.mechanisms.parametricEQ.State;
import org.exponential.robots.OurRobot;

import java.util.ArrayList;

@Autonomous(group="Autonomous", name="OneRingTurretAuto")
public class OneRingTurretAuto extends LinearOpMode {
    OurRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        CameraOpenCV camera = new CameraOpenCV(225, 250, 150, 150);
        robot = new OurRobot(camera);
        robot.initialize(this);
        //ourRobot.camera.setCameraBounds(300, 250, 150, 150);
        robot.camera.activate();

        waitForStart();

        int numRings = robot.camera.getNumberOfRings();
        robot.camera.deactivate();
        robot.setUpServos();
        robot.turret.pointToReloadPosition();

        robot.odometry.setPosition(48, -63, 180);

        ArrayList<CubicSpline.CubicSplinePoint> spline = new ArrayList<CubicSpline.CubicSplinePoint>();
        // starting on rightmost red tape, facing left
        State state = new State();
        state.fieldX = 48;
        state.fieldY = -63;
        state.angle = 180;
        state.velX = 0;
        state.velY = 0;
        state.angleVel = 0;
        spline.add(new CubicSpline.CubicSplinePoint(state, 0));


        // go to gap between rings and wall (going backwards, wobble goal is on left of robot)
        state = new State();
        state.fieldX = 56;
        state.fieldY = -24;
        state.angle = 270;
        state.velX = 6.8;
        state.velY = 13.37;
        state.angleVel = 0;
        spline.add(new CubicSpline.CubicSplinePoint(state, 1.2));
        // move to proper zone
        if (numRings == 1) { //zone B
            state = new State();
            state.fieldX = 48;
            state.fieldY = 6;
            state.angle = 270;
            state.velX = -6.8;
            state.velY = 13.37;
            state.angleVel = 0;
            spline.add(new CubicSpline.CubicSplinePoint(state, 2.75));

            state = new State();
            state.fieldX = 26;
            state.fieldY = 36;
            state.angle = 270;
            state.velX = 0;
            state.velY = 0;
            state.angleVel = 0;
            spline.add(new CubicSpline.CubicSplinePoint(state, 4.45));
        }

        ((DriveTrainParametric)(robot.drivetrain)).moveAlongParametricEq(new CubicSpline(spline));

        robot.drivetrain.performBrake();

        //start revving up shooter
        robot.shooter.shootAtHighGoal();  //TODO: change this if you want

        //place wobble goal
        robot.wobbleGoalMover.placeGoal();



        //TODO: POWER SHOTS HERE


        //TODO: SECOND WOBBLE GOAL HERE
        if (numRings == 1) {
            robot.drivetrain.moveToStraight(14, -44);
        }
        robot.drivetrain.performBrake();
        robot.wobbleGoalMover.pickupGoal();

        //MOVING TO ZONE
        if (numRings == 1) { //zone B
            double targetX = 22;
            double targetY = 36;
            robot.drivetrain.moveTo(targetX, targetY, 270);
        }
        robot.drivetrain.performBrake();
        robot.wobbleGoalMover.placeGoal();

        //park on line
        if (numRings == 1) {
            robot.drivetrain.moveTo(24, 12, 270);
        }
        robot.savePositions();
    }
}
