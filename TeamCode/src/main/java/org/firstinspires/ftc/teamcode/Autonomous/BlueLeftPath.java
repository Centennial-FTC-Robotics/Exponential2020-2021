package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.mechanisms.CameraOpenCV;
import org.exponential.robots.OurRobot;

@Autonomous(group="Autonomous", name="BlueLeftPath")
public class BlueLeftPath extends LinearOpMode {
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

        // starting on leftmost blue tape, facing left
        robot.odometry.setPosition(48, -63, 180);

        robot.savePositions();
    }

    public void sleep(int ms) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < ms) {
            robot.odometry.update();
        }
    }
}
