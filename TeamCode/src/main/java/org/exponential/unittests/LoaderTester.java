package org.exponential.unittests;

import org.exponential.mechanisms.Conveyor;
import org.exponential.mechanisms.Loader;
import org.exponential.superclasses.UnitTester;

public class LoaderTester extends UnitTester {
    Loader loader;
    @Override
    public void runOpMode() throws InterruptedException {
        loader = new Loader();
        loader.initialize(this);
        trackIndex(0, 1);
    }

    @Override
    public void runTest(int index) {
        switch (index) {
            case 0:
                loadPositionTest();
            case 1:
                unloadPositionTest();
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
