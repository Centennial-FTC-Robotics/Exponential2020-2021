package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.exponential.robots.OurRobot;
@TeleOp(group = "Tests")
public class TurretEncTest extends LinearOpMode {
    OurRobot expo = new OurRobot();

    double testInput = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        expo.initialize(this);
        waitForStart();
        expo.turret.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()) {
            /*if (gamepad1.a) {
                expo.turret.loadPosition();
            }
            telemetry.addData("encoders: ", expo.turret.turretMotor.getCurrentPosition());
            telemetry.update();*/

            //Testing Hardware Code
            testInput = gamepad1.left_stick_x * .2;
            expo.turret.turretMotor.setPower(testInput);
        }
    }
}
