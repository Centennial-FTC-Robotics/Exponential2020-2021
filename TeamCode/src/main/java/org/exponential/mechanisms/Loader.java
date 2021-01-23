package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.exponential.superclasses.Mechanism;

public class Loader implements Mechanism {
    public static final double LOAD_POSITION = 0;
    public static final double UNLOAD_POSITION = .5;
    Servo loaderServo;
    @Override
    public void initialize(LinearOpMode opMode) {
        loaderServo = opMode.hardwareMap.servo.get("loaderServo");
        loaderServo.setPosition(UNLOAD_POSITION);
    }

    public void load() {
        loaderServo.setPosition(LOAD_POSITION);
    }

    public void unload() {
        loaderServo.setPosition(UNLOAD_POSITION);
    }
}
