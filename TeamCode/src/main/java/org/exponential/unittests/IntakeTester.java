package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Intake;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="IntakeTester", group="Autonomous")
public class IntakeTester extends LinearOpMode implements UnitTester {
    Intake intake;
    @Override
    public void runTests() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake();
        intake.initialize();
        runTests();
    }
}
