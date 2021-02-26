package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.mechanisms.CameraOpenCV;
import org.exponential.mechanisms.DriveTrainParametric;
import org.exponential.mechanisms.parametricEQ.CubicSpline;
import org.exponential.mechanisms.parametricEQ.State;
import org.exponential.robots.OurRobot;

import java.util.ArrayList;

@Autonomous(group="Autonomous", name="RedRightSpline")
public class RedRightSpline extends LinearOpMode {
    OurRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        CameraOpenCV camera = new CameraOpenCV(225, 250, 150, 150);
        robot = new OurRobot(camera);
        robot.initialize(this);
        //ourRobot.camera.setCameraBounds(300, 250, 150, 150);
        robot.camera.activate();
        waitForStart();
        Runnable myRunnable =
                new Runnable(){
                    public void run(){
                        while (true) {
                            robot.odometry.update();
                            sleep(100);
                        }
                    }
                };
        Thread thread = new Thread(myRunnable);
        //thread.start();

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
        if (numRings == 0) { //zone A
            state = new State();
            state.fieldX = 48;
            state.fieldY = 12;
            state.angle = 270;
            state.velX = 0;
            state.velY = 0;
            state.angleVel = 0;
            spline.add(new CubicSpline.CubicSplinePoint(state, 3.5));
        } else if (numRings == 1) { //zone B
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
        } else { //zone C
            // go to gap between rings and wall (going backwards, wobble goal is on left of robot)
            /*state = new State();
            state.fieldX = 48;
            state.fieldY = 10;
            state.angle = 270;
            state.velX = -6.8;
            state.velY = 13.37;
            state.angleVel = 0;
            spline.add(new CubicSpline.CubicSplinePoint(state, 2.9));*/

            // go to gap between rings and wall (going backwards, wobble goal is on left of robot)
            state = new State();
            state.fieldX = 48;
            state.fieldY = 52;
            state.angle = 270;
            state.velX = 0;
            state.velY = 0;
            state.angleVel = 0;
            spline.add(new CubicSpline.CubicSplinePoint(state, 4.15));

        }
        ((DriveTrainParametric)(robot.drivetrain)).moveAlongParametricEq(new CubicSpline(spline));
        
        robot.drivetrain.performBrake();

        //start revving up shooter
        robot.shooter.shootAtHighGoal();

        //place wobble goal
        robot.wobbleGoalMover.placeGoal();

        //shoot rings at power shot targets (also moves to the designated position on the field)
        //ourRobot.shootPowerShotTargets("red");

        if (numRings == 1) {
            robot.odometry.offsetXPos(3.5);
        } if (numRings == 4) {
            //robot.odometry.offsetXPos(-3.5);
        }
        // if it's 0 rings, go for power shots instead
        if (numRings == 0) {
            robot.shootAtPowerShotTargets("red");
        } else {
            robot.shootAtHighGoal("red");  //ends at 38, -6

        }
        if (numRings == 1) {
            //robot.odometry.offsetXPos(2);
            //pickup rings from starter stack
            robot.intake.outtake();  //should be intake, encoders are weird
            robot.shooter.shootAtHighGoal();

            robot.drivetrain.moveTo(36, -14, 270);

            sleep(1600);
            robot.intake.stop();
            //shoot the newly picked up rings
            robot.shootAtHighGoal("red");

            //robot.odometry.offsetXPos(-2);
        } else if (numRings == 4) {
            robot.shooter.shootAtHighGoal();

            /*//knock down starter stack
            robot.intake.intake(); //outtake rings to throw stack down
            robot.drivetrain.moveTo(36, -20, 270);
            robot.drivetrain.moveTo(36, -18, 270); */
            //pickup rings from starter stack
            robot.intake.outtake();  //should be intake, encoders are weird
            robot.drivetrain.moveTo(36, -15, 270);
            robot.drivetrain.performBrake();

            robot.drivetrain.moveTo(36, -17, 270);
            robot.drivetrain.performBrake();

            robot.drivetrain.moveTo(36, -19, 270);
            robot.drivetrain.performBrake();


            //robot.drivetrain.performBrake();
            //sleep(1100);
            // robot.intake.stop();
            //shoot the newly picked up rings
            robot.shootAtHighGoal("red");
            //pick up the rest of the rings
            robot.drivetrain.moveTo(36, -24, 270);
            //robot.drivetrain.performBrake();
            sleep(750);
            robot.shootAtHighGoal("red");
            robot.intake.stop();
        }

        //shootathighgoal moves robot to 36, -6

        //move to gap between half field and rings
        //robot.drivetrain.moveTo(11, -6, 270);
        //robot.drivetrain.moveTo(11.25, -24, 270);

        //robot.drivetrain.moveTo(11, -28, 270);

        //PICK UP SECOND GOAL
        if (numRings == 0) {
            //move to gap between half field and rings
            robot.drivetrain.moveTo(11, -6, 270);
            robot.drivetrain.moveTo(11, -47.5, 270);
        } else if (numRings == 1) {
            /*robot.odometry.offsetXPos(3);
            robot.odometry.offsetYPos(3);*/
            //robot.drivetrain.moveTo(11, -49, 270);
            robot.drivetrain.moveToStraight(14, -44);
        } else {
            robot.drivetrain.moveToStraight(15, -43);
            //robot.drivetrain.moveTo(11, -49, 270);

        }
        robot.drivetrain.performBrake();
        robot.wobbleGoalMover.pickupGoal();

        //move to the second gap
        //robot.drivetrain.moveTo(11, -24, 270);

        // move to proper zone
        if (numRings == 0) { //zone A
            robot.drivetrain.moveToStraight(44, 12, 270, 18);
        } else if (numRings == 1) { //zone B
            double targetX = 22;
            double targetY = 36;
            robot.drivetrain.moveTo(targetX, targetY, 270);
        } else { //zone C
            double targetX = 44;
            double targetY = 52;
            //double targetAngle = Math.atan2(targetY - robot.odometry.getyPos(), targetX - robot.odometry.getxPos());
            robot.drivetrain.moveToStraight(targetX, targetY, 270, 24);
        }
        robot.drivetrain.performBrake();
        robot.wobbleGoalMover.placeGoal();


        //park on line
        if (numRings == 1) {
            robot.drivetrain.moveTo(24, 12, 270);
        } else {
            robot.drivetrain.moveTo(40, 12, 270);
        }
        robot.savePositions();
    }

    public void sleep(int ms) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < ms) {
            robot.odometry.update();
        }
    }
}
