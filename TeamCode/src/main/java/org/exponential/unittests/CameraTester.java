package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Camera;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="CameraTester", group="Autonomous")
public class CameraTester extends LinearOpMode implements UnitTester {
    Camera camera;
    @Override
    public void runTests() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera();
        camera.initialize();
        runTests();
    }
}
