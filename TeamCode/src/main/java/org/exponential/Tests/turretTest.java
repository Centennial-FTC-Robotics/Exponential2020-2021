package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.exponential.robots.OurRobot;
@TeleOp
public class turretTest extends LinearOpMode {
    OurRobot expo = new OurRobot();
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        expo.initialize(this);
        expo.turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        expo.turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretPower(.2);
        expo.turret.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {
        turretPower(0);
        if(gamepad1.a) {
            turretPosition(1000);
            turretPower(1);
        }
        if(gamepad1.b) {
            turretPosition(0);
            turretPower(1);
        }
            telemetry.addData("turretPosition", expo.turret.turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }
    public void turretPower (double power) {
        expo.turret.turretMotor.setPower(power);
    }
    public void turretPosition (int position) {
        expo.turret.turretMotor.setTargetPosition(position);
    }
}
