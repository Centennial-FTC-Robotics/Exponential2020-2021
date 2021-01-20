package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.exponential.mechanisms.Camera;
import org.exponential.mechanisms.CameraOpenCVOld;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="CameraTester", group="Autonomous")
public class CameraTester extends UnitTester {
    Camera camera;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        camera = new Camera();
        camera.initialize(this);
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
