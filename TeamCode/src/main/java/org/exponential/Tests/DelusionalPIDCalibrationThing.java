package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.robots.OurRobot;

public class DelusionalPIDCalibrationThing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OurRobot expo = new OurRobot();
        expo.initialize(this);
        double p = 0.01;
        double i = 0.01;
        double d = 0;

        waitForStart();

        while(opModeIsActive()){

        }

    }
}
