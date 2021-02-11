package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.exponential.mechanisms.Shooter;
import org.exponential.robots.OurRobot;
@TeleOp
public class DelusionalPIDCalibrationThing extends LinearOpMode {
    public Shooter shooter;

    OurRobot expo = new OurRobot();
    DcMotorEx shooterMotor;
    final double NEW_P = .25;
    final double NEW_I = .1;
    final double NEW_D = .1;
    public void runOpMode() throws InterruptedException {
        expo.initialize(this);
        /*double p = 0.01;
        double i = 0.01;
        double d = 0;*/

        shooterMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"shooterMotor");

        //Getting old PID Values
        PIDCoefficients pidOrig = shooterMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        //Adding in New PID Values
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        shooterMotor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        //re-reading PID values to look for change
        PIDCoefficients pidModified = shooterMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        /* //bottom one ig rip------------------------------------------------------------------------------
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

         */

        waitForStart();
        while (opModeIsActive()) {

        telemetry.addData("oP", pidOrig.p);
        telemetry.addData("oI", pidOrig.i);
        telemetry.addData("oD", pidOrig.d);
        telemetry.addData("mP", pidModified.p);
        telemetry.addData("mI", pidModified.i);
        telemetry.addData("mD", pidModified.d);

        }
    }
}
