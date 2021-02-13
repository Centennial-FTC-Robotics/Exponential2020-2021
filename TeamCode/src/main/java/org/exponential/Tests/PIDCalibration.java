package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.robots.OurRobot;

@Disabled
@TeleOp(name = "PIDCalibration", group = "TeleOp")
public class PIDCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OurRobot ourRobot = new OurRobot();
        ourRobot.initialize(this);

        double p = .1;
        double i = .01;
        double d = 0;
        int index = 0;
        int inches = 12;
        while (opModeIsActive()) {
            if (gamepad1.dpad_down && index < 1) {
                index++;
                sleep(100);
            } else if (gamepad1.dpad_up && index < 1) {
                index++;
                sleep(100);
            }

            if (gamepad1.dpad_right) {
                if (index == 0) {
                    p += .01;
                } else if (index == 1) {
                    i += .001;
                }
                sleep(100);
            } else if (gamepad1.dpad_left) {
                if (index == 0) {
                    p -= .01;
                } else if (index == 1) {
                    i -= .001;
                }
                sleep(100);
            }

            if (gamepad1.left_stick_y > 0) {
                inches++;
                sleep(100);
            } else if (gamepad1.left_stick_y < 0) {
                inches--;
                sleep(100);
            }

            if (gamepad1.a) {
                ourRobot.drivetrain.moveRelative(0, inches);
                sleep(100);
            } else if (gamepad1.b) {
                ourRobot.drivetrain.moveRelative(inches, 0);
                sleep(100);
            }
            if (index == 0) {
                telemetry.addData("Changing", "P");
            } else if (index == 1) {
                telemetry.addData("Changing", "i");
            }
            telemetry.addData("p", p);
            telemetry.addData("i", i);
            telemetry.addData("d", d);
            telemetry.update();
        }
    }
}
