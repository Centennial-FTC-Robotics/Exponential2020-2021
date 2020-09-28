package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.exponential.superclasses.Mechanism;

public class Conveyor implements Mechanism {
    DcMotorEx conveyorMotor;
    @Override
    public void initialize(LinearOpMode opMode) {
        conveyorMotor = opMode.hardwareMap.get(DcMotorEx.class, "conveyorMotor");
    }

    public void setPower(double power) {
        conveyorMotor.setPower(power);
    }
}
