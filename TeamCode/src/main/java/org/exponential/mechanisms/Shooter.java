package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.exponential.superclasses.Mechanism;

public class Shooter implements Mechanism {
    DcMotorEx shooterMotor;
    @Override
    public void initialize(LinearOpMode opMode) {
        shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void shoot() {
        setPower(.5);
    }
}
