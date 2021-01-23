package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.robots.OurRobot;

@Autonomous(group="Autonomous", name="RedPath")
public class RedPath extends LinearOpMode {
    OurRobot ourRobot;
    @Override
    public void runOpMode() throws InterruptedException {
        ourRobot = new OurRobot();
        ourRobot.initialize(this);
        // ourRobot.camera.activate();
        waitForStart();
        int numRings = ourRobot.camera.getNumberOfRings();
        ourRobot.camera.deactivate();

        // starting on rightmost red tape, facing right
        ourRobot.odometry.setPosition(48, -63, 180);

        // go to gap between rings and wall (going backwards, wobble goal is on left of robot)
        ourRobot.drivetrain.moveTo(60, -24, 270);

        // move to proper zone
        if (numRings == 0) { //zone A
            ourRobot.drivetrain.moveTo(48, 12, 270);
        } else if (numRings == 1) { //zone B
            ourRobot.drivetrain.moveTo(24, 36, 270);
        } else { //zone C
            ourRobot.drivetrain.moveTo(48, 60, 270);
        }
        ourRobot.wobbleGoalMover.lower();
        ourRobot.wobbleGoalMover.release();
        ourRobot.wobbleGoalMover.raise();
        //move to gap between half field and rings
        ourRobot.drivetrain.moveTo(-12, -24, 270);

        //pick up second goal
        ourRobot.drivetrain.moveTo(24, -36, 0);
        ourRobot.wobbleGoalMover.lower();
        ourRobot.wobbleGoalMover.clamp();
        ourRobot.wobbleGoalMover.raise();

        //move to the same gap
        ourRobot.drivetrain.moveTo(-12, -24, 270);

        // move to proper zone
        if (numRings == 0) { //zone A
            ourRobot.drivetrain.moveTo(48, 12, 270);
        } else if (numRings == 1) { //zone B
            ourRobot.drivetrain.moveTo(24, 36, 270);
        } else { //zone C
            ourRobot.drivetrain.moveTo(48, 60, 270);
        }
        ourRobot.wobbleGoalMover.lower();
        ourRobot.wobbleGoalMover.release();
        ourRobot.wobbleGoalMover.raise();

        //move to some position on field to start shooting
        ourRobot.drivetrain.moveTo(12, 0, 90);
        
    }
}
