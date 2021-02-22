package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.exponential.superclasses.Mechanism;

import static java.lang.Thread.sleep;

public class Intake implements Mechanism {
    public static final double INTAKE_POWER = .9;
    public static final double OUTTAKE_POWER = -.9;
    public static final double CONVEYOR_INTAKE_POWER = 1;
    public static final double CONVEYOR_OUTTAKE_POWER = -1;
    public static final double SERVO_INTAKE_POWER = -1;
    public static final double SERVO_OUTTAKE_POWER = 1;
    
    public static final double INTAKE_FACTOR = .8;
    public static final double CONVEYOR_FACTOR = -.15;

    public static final double LEFT_SERVO_POSITION = .85;
    public static final double RIGHT_SERVO_POSITION = 0;
    public static final double INTAKE_RELEASE_POSITION = .4;
    public DcMotorEx intakeMotor;
    public DcMotorEx conveyorMotor;
    //public CRServo conveyorServo;
    public Servo rightIntakeServo;
    public Servo intakeReleaseServo;
    @Override
    public void initialize(LinearOpMode opMode) {
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        conveyorMotor = opMode.hardwareMap.get(DcMotorEx.class, "conveyorMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //conveyorServo = opMode.hardwareMap.get(CRServo.class,"conveyorServo");
        //rightIntakeServo = opMode.hardwareMap.servo.get("rightIntakeServo");
        intakeReleaseServo = opMode.hardwareMap.servo.get("intakeReleaseServo");
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    // TODO: this will have to be fine tuned. (reverse direction so that -1 is intaking, the intake factor, etc.)
    // this method takes in the RAW GAMEPAD Y VALUE, all reversals/etc should be done here
    public void setPowerInput(double y) {

        intakeMotor.setPower(y * INTAKE_FACTOR);
        conveyorMotor.setPower(y * CONVEYOR_FACTOR);

    }

    // TODO: intake and outtake power will need to be changed for direction as well.
    public void intake() {
        intakeMotor.setPower(INTAKE_POWER);
        conveyorMotor.setPower(CONVEYOR_INTAKE_POWER);
        //conveyorServo.setPower(SERVO_INTAKE_POWER);
    }

    public void outtake() {
        intakeMotor.setPower(OUTTAKE_POWER);
        conveyorMotor.setPower(CONVEYOR_OUTTAKE_POWER);
        //conveyorServo.setPower(SERVO_OUTTAKE_POWER);
    }

    public void stop() {
        intakeMotor.setPower(0);
        conveyorMotor.setPower(0);
        //conveyorServo.setPower(0);
    }

    public void setServoPositions() {
        //leftIntakeServo.setPosition(LEFT_SERVO_POSITION);
        //rightIntakeServo.setPosition(RIGHT_SERVO_POSITION);
        intakeReleaseServo.setPosition(INTAKE_RELEASE_POSITION);
    }
}
