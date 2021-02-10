package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.exponential.superclasses.Mechanism;

public class Shooter implements Mechanism {

    DcMotorEx shooterMotor;
    @Override
    public void initialize(LinearOpMode opMode) {
        shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void shootAtPowerShot() { setPower(.43); }

    public void powerShotReadjustPower() { setPower(.37); }

    public void shootAtHighGoal() {setPower(.39); }

    public void highGoalReadjustPower() {
        setPower(.385);
    }


    public void stopShooting() {
        setPower(0);
    }
}
