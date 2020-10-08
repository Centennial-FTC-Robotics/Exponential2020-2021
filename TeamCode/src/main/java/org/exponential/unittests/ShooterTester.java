package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Shooter;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="ShooterTester", group="Autonomous")
public class ShooterTester extends UnitTester {
    Shooter shooter;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter();
        shooter.initialize(this);
        trackIndex(0, 5);
    }

    @Override
    public void runTest(int index) {

    }
}
