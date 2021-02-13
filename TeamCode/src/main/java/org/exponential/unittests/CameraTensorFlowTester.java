package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.exponential.mechanisms.CameraTensorFlow;
import org.exponential.superclasses.UnitTester;

@Disabled
@Autonomous(name="CameraTensorFlowTester", group="Autonomous")
public class CameraTensorFlowTester extends UnitTester {
    CameraTensorFlow camera;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        camera = new CameraTensorFlow();

        camera.initialize(this);
        camera.activate();
        trackIndex(0, 1);
    }

    @Override
    public void runTest(int index) {
        switch (index) {
            case 0:
                activateCamera();
                break;
            case 1:
                deactivateCamera();
                break;
            default:
                break;
        }
    }
    public void activateCamera() {
        camera.activate();
    }

    public void deactivateCamera() {
        camera.deactivate();
    }
}
