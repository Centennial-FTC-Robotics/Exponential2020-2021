package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.IMU;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="IMUTester", group="Autonomous")
public class IMUTester extends UnitTester {
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        imu = new IMU();
        imu.initialize(this);
        trackIndex(0, 5);
    }

    @Override
    public void runTest(int index) {

    }
}
