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
        CameraOpenCV camera = new CameraOpenCV(225, 250, 150, 150);
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
            robot.drivetrain.moveTo(48, 12, 270);
        } else if (numRings == 1) { //zone B
            robot.drivetrain.moveTo(26, 36, 270);
        } else { //zone C
            //split into two moves
            robot.drivetrain.moveTo(48, 10, 270);
            robot.drivetrain.moveTo(48, 52, 270);
        }
        robot.drivetrain.performBrake();

        //start revving up shooter
        robot.shooter.shootAtHighGoal();

        //place wobble goal
        robot.wobbleGoalMover.placeGoal();

        //shoot rings at power shot targets (also moves to the designated position on the field)
        //ourRobot.shootPowerShotTargets("red");
        robot.shootAtHighGoal("red");  //ends at 36, -6

        if (numRings == 1) {
            //pickup rings from starter stack
            robot.intake.outtake();  //should be intake, encoders are weird
            robot.shooter.shootAtHighGoal();

            robot.drivetrain.moveTo(36, -16, 270);

            sleep(1400);
            robot.intake.stop();
            //shoot the newly picked up rings
            robot.shootAtHighGoal("red");
        } else if (numRings == 4) {
            robot.shooter.shootAtHighGoal();

            //knock down starter stack
            robot.intake.intake(); //outtake rings to throw stack down
            robot.drivetrain.moveTo(36, -20, 270);
            robot.drivetrain.moveTo(36, -18, 270);
            //pickup rings from starter stack
            robot.intake.outtake();  //should be intake, encoders are weird

            robot.drivetrain.moveTo(36, -20, 270);
            robot.drivetrain.moveTo(36, -21.5, 270);
            robot.drivetrain.moveTo(36, -24, 270);

            sleep(1000);
            robot.intake.stop();
            //shoot the newly picked up rings
            robot.shootAtHighGoal("red");
        }

        //shootathighgoal moves robot to 36, -6

        //move to gap between half field and rings
        robot.drivetrain.moveTo(11., -6, 270);
        //robot.drivetrain.moveTo(11.25, -24, 270);

        //pick up second goal
        robot.drivetrain.moveTo(11., -50, 270);
        robot.drivetrain.performBrake();
        robot.wobbleGoalMover.pickupGoal();

        //move to the second gap
        robot.drivetrain.moveTo(11., -24, 270);

        // move to proper zone
        if (numRings == 0) { //zone A
            robot.drivetrain.moveTo(44, 12, 270);
        } else if (numRings == 1) { //zone B
            robot.drivetrain.moveTo(22, 36, 270);
        } else { //zone C
            robot.drivetrain.moveTo(44, 52, 270);
        }
        robot.drivetrain.performBrake();
        robot.wobbleGoalMover.placeGoal();


        //park on line
        robot.drivetrain.moveTo(40, 6, 270);
        robot.savePositions();
    }
}
