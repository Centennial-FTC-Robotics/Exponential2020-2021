package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.exponential.superclasses.Mechanism;

import java.util.HashMap;

public class Drivetrain implements Mechanism {
    DcMotorEx frontLeft;
    DcMotorEx backLeft;
    DcMotorEx frontRight;
    DcMotorEx backRight;

    @Override
    public void initialize(LinearOpMode opMode) {
        frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPowerDriveMotors(HashMap<String, Double> powers) {
        frontLeft.setPower(powers.get("frontLeft"));
        backLeft.setPower(powers.get("backLeft"));
        frontRight.setPower(powers.get("frontRight"));
        backRight.setPower(powers.get("backRight"));
    }
}
