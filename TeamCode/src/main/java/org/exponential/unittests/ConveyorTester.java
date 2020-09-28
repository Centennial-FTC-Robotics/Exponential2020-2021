package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Conveyor;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="ConveyorTester", group="Autonomous")
public class ConveyorTester extends LinearOpMode implements UnitTester {
    Conveyor conveyor;
    @Override
    public void runTests() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        conveyor = new Conveyor();
        conveyor.initialize();
        runTests();
    }
}
