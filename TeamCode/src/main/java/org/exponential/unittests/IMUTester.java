package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.IMU;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="IMUTester", group="Autonomous")
public class IMUTester extends LinearOpMode implements UnitTester {
    IMU imu;
    @Override
    public void runTests() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMU();
        imu.initialize();
        runTests();
    }
}
