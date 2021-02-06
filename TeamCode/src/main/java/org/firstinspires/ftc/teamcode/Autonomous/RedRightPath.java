package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.CameraOpenCV;
import org.exponential.robots.OurRobot;

@Autonomous(group="Autonomous", name="RedRightPath")
public class RedRightPath extends LinearOpMode {
    OurRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        CameraOpenCV camera = new CameraOpenCV(250, 200, 150, 150);
        robot = new OurRobot(camera);
        robot.initialize(this);
        //ourRobot.camera.setCameraBounds(300, 250, 150, 150);
        robot.camera.activate();
        waitForStart();
        int numRings = robot.camera.getNumberOfRings();
        robot.camera.deactivate();
        robot.setUpServos();
        
        // starting on rightmost red tape, facing left
        robot.odometry.setPosition(48, -63, 180);

        // go to gap between rings and wall (going backwards, wobble goal is on left of robot)
        robot.drivetrain.moveTo(56, -24, 270);

        // move to proper zone
        if (numRings == 0) { //zone A
            robot.drivetrain.moveTo(44, 12, 270);
        } else if (numRings == 1) { //zone B
            robot.drivetrain.moveTo(22, 36, 270);
        } else { //zone C
            robot.drivetrain.moveTo(44, 56, 270);
        }
        robot.drivetrain.performBrake();

        //start revving up shooter
        robot.shooter.shootAtHighGoal();

        //place wobble goal
        robot.wobbleGoalMover.placeGoal();

        //shoot rings at power shot targets (also moves to the designated position on the field)
        //ourRobot.shootPowerShotTargets("red");
        robot.shootAtHighGoal("red");

        //move to gap between half field and rings
        robot.drivetrain.moveTo(11.25, -24, 270);

        //pick up second goal
        robot.drivetrain.moveTo(11.25, -50, 270);
        robot.drivetrain.performBrake();
        robot.wobbleGoalMover.pickupGoal();

        //move to the second gap
        robot.drivetrain.moveTo(11.25, -24, 270);

        // move to proper zone
        if (numRings == 0) { //zone A
            robot.drivetrain.moveTo(42, 12, 270);
        } else if (numRings == 1) { //zone B
            robot.drivetrain.moveTo(9, 36, 270);
        } else { //zone C
            robot.drivetrain.moveTo(39, 56, 270);
        }
        robot.drivetrain.performBrake();
        robot.wobbleGoalMover.placeGoal();


        //park on line
        robot.drivetrain.moveTo(30, 6, 270);
        robot.savePositions();
    }
}
