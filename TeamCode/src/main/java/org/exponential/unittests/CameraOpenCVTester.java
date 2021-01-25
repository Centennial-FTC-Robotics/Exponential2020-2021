package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.exponential.mechanisms.CameraOpenCV;
import org.exponential.mechanisms.CameraTensorFlow;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="CameraOpenCVTester", group="Autonomous")
public class CameraOpenCVTester extends UnitTester {
    CameraOpenCV camera;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        //camera = new CameraOpenCV();
        camera = new CameraOpenCV();

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
