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
        trackIndex(0, 3);
    }

    @Override
    public void runTest(int index) {
        switch (index) {
            case 0:
                regularStrafing();
            case 1:
                fieldCentricStrafing45Degrees();
            case 2:
                fieldCentricStrafing202Degrees();
            case 3:
                odometricTracking();
            default:
                break;
        }
    }

    public void regularStrafing() {

    }

    public void fieldCentricStrafing45Degrees() {

    }

    public void fieldCentricStrafing202Degrees() {

    }

    public void odometricTracking() {

    }
}
