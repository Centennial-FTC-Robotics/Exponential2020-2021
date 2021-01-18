package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.exponential.superclasses.Mechanism;

import static java.lang.Thread.sleep;

public class Intake implements Mechanism {
    public static final double INTAKE_POWER = -.6;
    public static final double OUTTAKE_POWER = .6;

    public static final double INTAKE_FACTOR = .6;

    public static final double LEFT_SERVO_POSITION = 0;
    public static final double RIGHT_SERVO_POSITION = 0;
    DcMotorEx intakeMotor;
    Servo leftIntakeServo;
    public Servo rightIntakeServo;
    @Override
    public void initialize(LinearOpMode opMode) {
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftIntakeServo = opMode.hardwareMap.servo.get("leftIntakeServo");
        rightIntakeServo = opMode.hardwareMap.servo.get("rightIntakeServo");
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    // TODO: this will have to be fine tuned. (reverse direction so that -1 is intaking, the intake factor, etc.)
    // this method takes in the RAW GAMEPAD Y VALUE, all reversals/etc should be done here
    public void setPowerInput(double y) {
        intakeMotor.setPower(y * INTAKE_FACTOR);
    }

    // TODO: intake and outtake power will need to be changed for direction as well.
    public void intake() {
        setPower(INTAKE_POWER);
    }

    public void outtake() {
        setPower(OUTTAKE_POWER);
    }

    public void stop() {
        setPower(0);
    }

    public void setServoPositions() {
        // leftIntakeServo.setPosition(LEFT_SERVO_POSITION);
        rightIntakeServo.setPosition(RIGHT_SERVO_POSITION);
    }
}
