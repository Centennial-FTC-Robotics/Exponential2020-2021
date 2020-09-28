package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.WobbleGoalPlacer;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="WobbleGoalPlacerTester", group="Autonomous")
public class WobbleGoalPlacerTester extends LinearOpMode implements UnitTester {
    WobbleGoalPlacer wobbleGoalPlacer;
    @Override
    public void runTests() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        wobbleGoalPlacer = new WobbleGoalPlacer();
        wobbleGoalPlacer.initialize();
        runTests();
    }
}
