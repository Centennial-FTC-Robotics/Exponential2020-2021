package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.WobbleGoalMover;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="WobbleGoalMoverTester", group="Autonomous")
public class WobbleGoalMoverTester extends UnitTester {
    WobbleGoalMover wobbleGoalMover;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        wobbleGoalMover = new WobbleGoalMover();
        wobbleGoalMover.initialize(this);
        trackIndex(0, 3);
    }

    @Override
    public void runTest(int index) {
        switch (index) {
            case 0:
                raise();
            case 1:
                lower();
            case 2:
                clamp();
            case 3:
                release();
            default:
                break;
        }
    }

    public void raise() {
        wobbleGoalMover.raise();
    }

    public void lower() {
        wobbleGoalMover.lower();
    }

    public void clamp() {
        wobbleGoalMover.clamp();
    }

    public void release() {
        wobbleGoalMover.release();
    }
}
