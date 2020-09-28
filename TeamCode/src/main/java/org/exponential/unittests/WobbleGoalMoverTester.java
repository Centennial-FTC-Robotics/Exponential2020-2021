package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.WobbleGoalMover;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="WobbleGoalMoverTester", group="Autonomous")
public class WobbleGoalMoverTester extends LinearOpMode implements UnitTester {
    WobbleGoalMover wobbleGoalMover;
    @Override
    public void runOpMode() throws InterruptedException {
        wobbleGoalMover = new WobbleGoalMover();
        wobbleGoalMover.initialize();
        runTests();
    }

    @Override
    public void runTests() {

    }
}
