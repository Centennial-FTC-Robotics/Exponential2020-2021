package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.exponential.superclasses.Mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class Loader implements Mechanism {
    public static final double LOAD_POSITION = 1;
    public static final double UNLOAD_POSITION = 0;
    Servo loaderServo;
    @Override
    public void initialize(LinearOpMode opMode) {
        loaderServo = hardwareMap.servo.get("loaderServo");
        loaderServo.setPosition(UNLOAD_POSITION);
    }

    public void load() {
        loaderServo.setPosition(LOAD_POSITION);
    }

    public void unload() {
        loaderServo.setPosition(UNLOAD_POSITION);
    }
}
