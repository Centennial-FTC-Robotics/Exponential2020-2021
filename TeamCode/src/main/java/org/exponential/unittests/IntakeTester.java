package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.mechanisms.Intake;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="IntakeTester", group="Autonomous")
public class IntakeTester extends UnitTester {
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake();
        intake.initialize(this);
        trackIndex(0, 1);
    }

    @Override
    public void runTest(int index) {
        switch (index) {
            case 0:
                autonomousIntakeTest();
            case 1:
                autonomousOuttakeTest();
            case 2:
                teleopIntakeTest();
            case 3:
                teleopOuttakeTest();
            default:
                break;
        }
    }

    public void autonomousIntakeTest() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        intake.intake();
        while (timer.seconds() < 3.0);
        intake.stop();
    }

    public void autonomousOuttakeTest() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        intake.outtake();
        while (timer.seconds() < 3.0);
        intake.stop();
    }

    public void teleopIntakeTest() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        intake.setPowerInput(-1);  // -1 because "down" should be
        while (timer.seconds() < 3.0);
        intake.stop();
    }

    public void teleopOuttakeTest() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        intake.setPowerInput(1);  // -1 because "down" should be
        while (timer.seconds() < 3.0);
        intake.stop();
    }
}
