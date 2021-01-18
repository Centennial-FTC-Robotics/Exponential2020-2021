package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Camera;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="CameraTester", group="Autonomous")
public class CameraTester extends UnitTester {
    Camera camera;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        camera = new Camera();
        camera.initialize(this);
        trackIndex(0, 5);
    }

    @Override
    public void runTest(int index) {

    }
}
