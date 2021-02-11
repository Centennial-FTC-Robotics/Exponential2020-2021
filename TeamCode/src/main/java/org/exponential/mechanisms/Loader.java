package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.superclasses.Mechanism;


public class Loader implements Mechanism {
    public static final double LOAD_POSITION = 0;
    public static final double UNLOAD_POSITION = .55;
    Servo loaderServo;
    LinearOpMode opMode;

    @Override
    public void initialize(LinearOpMode opMode) {
        this.opMode = opMode;
        loaderServo = opMode.hardwareMap.servo.get("loaderServo");
        loaderServo.setPosition(UNLOAD_POSITION);
    }

    public void load() {
        loaderServo.setPosition(LOAD_POSITION);
    }

    public void unload() {
        loaderServo.setPosition(UNLOAD_POSITION);
    }

    public void loadAndUnload() {
        load();
        opMode.sleep(100);
        unload();
    }

    public void loadAllRings() {
        for (int i = 0; i < 3; i++) {
            loadAndUnload();
            opMode.sleep(250);
        }
    }
}
