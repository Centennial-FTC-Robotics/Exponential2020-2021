package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Drivetrain;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="DrivetrainTester", group="Autonomous")
public class DrivetrainTester extends LinearOpMode implements UnitTester {
    Drivetrain drivetrain;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain();
        drivetrain.initialize(this);
        runTests();
    }

    @Override
    public void runTests() {

    }
}
