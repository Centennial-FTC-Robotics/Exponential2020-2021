package org.exponential.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.mechanisms.Camera;
import org.exponential.mechanisms.Drivetrain;
import org.exponential.superclasses.Robot;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class OurRobot implements Robot {
    public Drivetrain drivetrain;
    public Camera camera;

    @Override
    public void initialize(LinearOpMode opMode) {
        List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Drivetrain();
        drivetrain.initialize(opMode);
        camera = new Camera();
        camera.initialize(opMode);
    }
}
