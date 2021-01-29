package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.exponential.superclasses.Mechanism;

import static org.exponential.utility.Utility.sleep;

public class WobbleGoalMover implements Mechanism {
    /*Servo leftMoverServo;
    Servo rightMoverServo;*/
    Servo raiseServo;
    Servo clampServo;

    public static final double RAISE_POSITION = .2;
    public static final double LOWER_POSITION = .62;
    public static final double CLAMP_POSITION = .7;
    public static final double RELEASE_POSITION = .4;
    @Override
    public void initialize(LinearOpMode opMode) {
        /*leftMoverServo = hardwareMap.servo.get("leftMoverServo");
        rightMoverServo = hardwareMap.servo.get("rightMoverServo");*/
        raiseServo = opMode.hardwareMap.servo.get("raiseServo");
        clampServo = opMode.hardwareMap.servo.get("clampServo");
    }

    //assuming the mover will be a clamp/arm system
    public void clamp() {
        clampServo.setPosition(CLAMP_POSITION);
    }

    public void release() {
        clampServo.setPosition(RELEASE_POSITION);
    }

    public void raise() {
        raiseServo.setPosition(RAISE_POSITION);
    }

    public void lower() {
        raiseServo.setPosition(LOWER_POSITION);
    }

    public void placeGoal() {
        lower();
        sleep(500);
        release();
        sleep(500);
        raise();
    }

    public void pickupGoal() {
        lower();
        sleep(500);
        clamp();
        sleep(500);
        raise();
    }
}
