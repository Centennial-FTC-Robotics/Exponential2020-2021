package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.exponential.superclasses.Mechanism;

public class Intake implements Mechanism {
    public static final double INTAKE_POWER = .5;
    public static final double OUTTAKE_POWER = .5;

    DcMotorEx intakeMotor;
    @Override
    public void initialize(LinearOpMode opMode) {
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void intakeRings() {
        setPower(INTAKE_POWER);
    }

    public void outtakeRings() {
        setPower(OUTTAKE_POWER);
    }
}
