package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Shooter;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="ShooterTester", group="Autonomous")
public class ShooterTester extends LinearOpMode implements UnitTester {
    Shooter shooter;
    @Override
    public void runTests() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter();
        shooter.initialize();
        runTests();
    }
}
