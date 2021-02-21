package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.robots.OurRobot;

@Disabled
@Autonomous(group="Autonomous", name="SampleAutoPath")
public class SampleAutoPath extends LinearOpMode {
    OurRobot ourRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        ourRobot = new OurRobot();
        ourRobot.initialize(this);
        ourRobot.camera.activate();
        waitForStart();
        int numRings = ourRobot.camera.getNumberOfRings();
        ourRobot.camera.deactivate();

        //rest of path here
    }
}
