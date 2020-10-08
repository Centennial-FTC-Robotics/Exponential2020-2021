package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Conveyor;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="ConveyorTester", group="Autonomous")
public class ConveyorTester extends UnitTester {
    Conveyor conveyor;
    @Override
    public void runOpMode() throws InterruptedException {
        conveyor = new Conveyor();
        conveyor.initialize(this);
        trackIndex(0, 5);
    }

    @Override
    public void runTest(int index) {

    }
}
