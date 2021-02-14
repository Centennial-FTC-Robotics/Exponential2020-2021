package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        thread.start();

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
            robot.drivetrain.moveTo(48, 6, 270);
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

            robot.drivetrain.moveTo(36, -16, 270);

            sleep(1400);
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
            robot.drivetrain.moveTo(36, -18, 270);
            robot.drivetrain.moveTo(36, -17, 270);

            robot.drivetrain.moveTo(36, -18, 270);
            robot.drivetrain.performBrake();
            sleep(1900);
            robot.intake.stop();
            //shoot the newly picked up rings
            robot.shootAtHighGoal("red");
        }

        //shootathighgoal moves robot to 36, -6

        //move to gap between half field and rings
        robot.drivetrain.moveTo(11, -6, 270);
        //robot.drivetrain.moveTo(11.25, -24, 270);

        robot.drivetrain.moveTo(11, -28, 270);

        //pick up second goal
        if (numRings == 0) {
            robot.drivetrain.moveTo(11, -49, 270);
        } else if (numRings == 1) {
            robot.odometry.offsetXPos(3);
            robot.odometry.offsetYPos(3);
            robot.drivetrain.moveTo(11, -49, 270);

        } else {
            robot.drivetrain.moveTo(11, -49, 270);

        }
        robot.drivetrain.performBrake();
        robot.wobbleGoalMover.pickupGoal();

        //move to the second gap
        robot.drivetrain.moveTo(11, -24, 270);

        // move to proper zone
        if (numRings == 0) { //zone A
            robot.drivetrain.moveTo(44, 12, 270);
        } else if (numRings == 1) { //zone B
            double targetX = 22;
            double targetY = 36;
            robot.drivetrain.moveTo(targetX, targetY, 270);
        } else { //zone C
            double targetX = 44;
            double targetY = 52;
            double targetAngle = Math.atan2(targetY - robot.odometry.getyPos(), targetX - robot.odometry.getxPos());
            robot.drivetrain.moveTo(targetX, targetY, targetAngle);
        }
        robot.drivetrain.performBrake();
        robot.wobbleGoalMover.placeGoal();


        //park on line
        if (numRings == 1) {
            robot.drivetrain.moveTo(24, 6, 270);
        } else {
            robot.drivetrain.moveTo(40, 6, 270);
        }
        robot.savePositions();
    }

    public void sleep(int ms) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < ms) {
            //robot.odometry.update();
        }
    }
}
