package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.CameraOpenCV;
import org.exponential.robots.OurRobot;

@Autonomous(group="Autonomous", name="RedLeftPath")
public class RedLeftPath extends LinearOpMode {
    OurRobot ourRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        CameraOpenCV camera = new CameraOpenCV(300, 1050, 150, 150);
        ourRobot = new OurRobot(camera);
        OurRobot robot = ourRobot;
        ourRobot.initialize(this);
        //ourRobot.camera.setCameraBounds(300, 250, 150, 150);
        ourRobot.camera.activate();
        waitForStart();

        int numRings = ourRobot.camera.getNumberOfRings();
        ourRobot.camera.deactivate();

        robot.shooter.shootAtHighGoal();
        sleep(1500);
        robot.loadAndUnloadAllRings();
        /*robot.loader.loadAndUnload();
        sleep(3000);
        robot.loader.loadAndUnload();
        sleep(3000);
        robot.loader.loadAndUnload();*/

    }
}
