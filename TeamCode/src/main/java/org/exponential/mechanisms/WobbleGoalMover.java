package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.exponential.superclasses.Mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class WobbleGoalMover implements Mechanism {
    Servo leftMoverServo;
    Servo rightMoverServo;
    Servo clampServo;

    @Override
    public void initialize(LinearOpMode opMode) {
        leftMoverServo = hardwareMap.servo.get("leftMoverServo");
        rightMoverServo = hardwareMap.servo.get("rightMoverServo");
        clampServo = hardwareMap.servo.get("clampServo");
    }

    //assuming the mover will be a clamp/arm system
    public void clamp() {
        clampServo.setPosition(1);
    }

    public void release() {
        clampServo.setPosition(0);
    }

    public void raise() {
        leftMoverServo.setPosition(0);
        rightMoverServo.setPosition(0);
    }

    public void lower() {
        leftMoverServo.setPosition(0);
        rightMoverServo.setPosition(0);
    }
}
