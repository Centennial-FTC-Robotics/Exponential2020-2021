package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.exponential.superclasses.Mechanism;

public class Shooter implements Mechanism {

    DcMotorEx shooterMotor;

    public static final double NEW_P = 22;
    public static final double NEW_I = 3.85;
    public static final double NEW_D = 3.55;


    @Override
    public void initialize(LinearOpMode opMode) {
        shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)shooterMotor.getController();

        // get the port number of our configured motor.
        int motorIndex = ((DcMotorEx)shooterMotor).getPortNumber();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients pidOrig = motorControllerEx.getPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients.
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        motorControllerEx.setPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void shootAtPowerShot() {
        setPower(.355);
    }

    public void shootAtHighGoal() {
        setPower(.368);
    }

    public void readjustHighGoalPower() {
        setPower(.338);
    }

    public void stopShooting() {
        setPower(0);
    }

    //These methods are used for adjusting for distance
    public void distancedHighGoal(double adjustment) { setPower(.368 + adjustment); }

    public void distancedReadjustHighGoal (double adjustment) {setPower ( .338 + adjustment); }

    public void distancedPowerShot (double adjustment) { setPower (.355 + adjustment); }

}
