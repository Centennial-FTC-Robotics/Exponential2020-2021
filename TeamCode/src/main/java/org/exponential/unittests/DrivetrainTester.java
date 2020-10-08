package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Drivetrain;
import org.exponential.superclasses.UnitTester;

@Autonomous(name="DrivetrainTester", group="Autonomous")
public class DrivetrainTester extends UnitTester {
    Drivetrain drivetrain;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain();
        drivetrain.initialize(this);
        trackIndex(0, 0);
    }

    @Override
    public void runTest(int index) {
        switch (index) {
            case 0:
                regularStrafing();
                break;
            default:
                break;
        }
    }

    public void regularStrafing() {
        //TODO: add code when move method is made
    }
}
