package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Intake;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="IntakeTester", group="Autonomous")
public class IntakeTester extends UnitTester {
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake();
        intake.initialize(this);
        trackIndex(0, 5);
    }

    @Override
    public void runTest(int index) {

    }
}
