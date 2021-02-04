package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.CameraOpenCV;
import org.exponential.robots.OurRobot;

@Autonomous(group="Autonomous", name="RedRightPath")
public class RedRightPath extends LinearOpMode {
    OurRobot ourRobot;
    @Override
    public void runOpMode() throws InterruptedException {
        CameraOpenCV camera = new CameraOpenCV(250, 200, 150, 150);
        ourRobot = new OurRobot(camera);
        ourRobot.initialize(this);
        //ourRobot.camera.setCameraBounds(300, 250, 150, 150);
        ourRobot.camera.activate();
        waitForStart();
        int numRings = ourRobot.camera.getNumberOfRings();
        ourRobot.camera.deactivate();
        ourRobot.setUpServos();
        
        // starting on rightmost red tape, facing left
        ourRobot.odometry.setPosition(48, -63, 180);

        // go to gap between rings and wall (going backwards, wobble goal is on left of robot)
        ourRobot.drivetrain.moveTo(56, -24, 270);

        // move to proper zone
        if (numRings == 0) { //zone A
            ourRobot.drivetrain.moveTo(42, 12, 270);
        } else if (numRings == 1) { //zone B
            ourRobot.drivetrain.moveTo(20, 36, 270);
        } else { //zone C
            ourRobot.drivetrain.moveTo(42, 56, 270);
        }
        ourRobot.drivetrain.performBrake();

        ourRobot.wobbleGoalMover.placeGoal();

        //shoot rings at power shot targets (also moves to the designated position on the field)
        //ourRobot.shootPowerShotTargets("red");
        ourRobot.shootAtHighGoal("red");

        //move to gap between half field and rings
        ourRobot.drivetrain.moveTo(6.75, -24, 270);

        //pick up second goal
        ourRobot.drivetrain.moveTo(6.75, -48, 270);
        ourRobot.drivetrain.performBrake();
        ourRobot.wobbleGoalMover.pickupGoal();

        //move to the second gap
        ourRobot.drivetrain.moveTo(6.75, -24, 270);

        // move to proper zone
        if (numRings == 0) { //zone A
            ourRobot.drivetrain.moveTo(42, 12, 270);
        } else if (numRings == 1) { //zone B
            ourRobot.drivetrain.moveTo(9, 36, 270);
        } else { //zone C
            ourRobot.drivetrain.moveTo(39, 56, 270);
        }
        ourRobot.drivetrain.performBrake();
        ourRobot.wobbleGoalMover.placeGoal();


        //park on line
        ourRobot.drivetrain.moveTo(30, 6, 270);
        ourRobot.odometry.savePosition();
    }
}
