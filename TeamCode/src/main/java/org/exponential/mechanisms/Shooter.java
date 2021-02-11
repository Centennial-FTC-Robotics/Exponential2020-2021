package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.exponential.superclasses.Mechanism;

public class Shooter implements Mechanism {

    DcMotorEx shooterMotor;

    public static final double NEW_I = 0;



    @Override
    public void initialize(LinearOpMode opMode) {
        shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
        //Getting old PID Values
        PIDCoefficients pidOrig = shooterMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        //Adding in New PID Values
        PIDCoefficients pidNew = new PIDCoefficients(pidOrig.p, NEW_I, pidOrig.d);
        shooterMotor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        */

        //------------------------------------------------------------------------------------------ testing out top and bottom.
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)shooterMotor.getController();

        // get the port number of our configured motor.
        int motorIndex = ((DcMotorEx)shooterMotor).getPortNumber();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients pidOrig = motorControllerEx.getPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients.
        PIDCoefficients pidNew = new PIDCoefficients(pidOrig.p, NEW_I, pidOrig.d);
        motorControllerEx.setPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDCoefficients pidModified = motorControllerEx.getPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void shootAtPowerShot() { setPower(.43); }

    public void powerShotReadjustPower() { setPower(.355); }

    public void shootAtHighGoal() {setPower(.38); }

    public void highGoalReadjustPower() {
        setPower(.385);
    }


    public void stopShooting() {
        setPower(0);
    }
}
