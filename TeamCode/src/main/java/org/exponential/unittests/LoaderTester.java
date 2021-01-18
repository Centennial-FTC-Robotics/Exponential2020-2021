package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.exponential.mechanisms.Loader;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="LoaderTester", group="Autonomous")
public class LoaderTester extends UnitTester {
    Loader loader;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        loader = new Loader();
        loader.initialize(this);
        trackIndex(0, 1);
    }

    @Override
    public void runTest(int index) {
        switch (index) {
            case 0:
                loadPositionTest();
                break;
            case 1:
                unloadPositionTest();
                break;
            default:
                break;
        }
    }

    public void loadPositionTest() {
        loader.load();
    }

    public void unloadPositionTest() {
        loader.unload();
    }
}
