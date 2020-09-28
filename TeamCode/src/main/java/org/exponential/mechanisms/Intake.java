package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.exponential.superclasses.Mechanism;

public class Intake implements Mechanism {
    DcMotorEx intakeMotor;
    @Override
    public void initialize(LinearOpMode opMode) {
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }
}
