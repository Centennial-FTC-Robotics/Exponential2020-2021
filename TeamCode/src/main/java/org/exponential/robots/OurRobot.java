package org.exponential.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Drivetrain;
import org.exponential.superclasses.Robot;

public class OurRobot implements Robot {
    public Drivetrain drivetrain;
    @Override
    public void initialize(LinearOpMode opMode) {
        drivetrain = new Drivetrain();
        drivetrain.initialize(opMode);
    }
}
