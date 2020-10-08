package org.exponential.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.exponential.superclasses.UnitTester;

import java.util.ArrayList;

@Autonomous(name="AllUnitTests", group="Autonomous")
public class AllUnitTests implements UnitTester {
    @Override
    public void trackIndex() { //put this in a runOpMode() method, if there even is one
        ArrayList<UnitTester> testers = new ArrayList<UnitTester>();
        testers.add(new CameraTester());
        testers.add(new ConveyorTester());
        testers.add(new DrivetrainTester());
        testers.add(new IMUTester());
        testers.add(new IntakeTester());
        testers.add(new ShooterTester());
        testers.add(new WobbleGoalMoverTester());
        testers.add(new WobbleGoalPlacerTester());

        for (UnitTester tester: testers) {
            tester.trackIndex();
        }

    }
}
