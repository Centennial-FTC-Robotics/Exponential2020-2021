package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.robots.OurRobot;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "BasicTeleOp", group = "TeleOp")
public class BasicTeleOp extends LinearOpMode {
    private OurRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        robot = new OurRobot();
        robot.initialize(this);
        robot.loadPositions();
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            if (gamepad1.right_trigger > 0) {
                robot.intake.setPowerInput(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                robot.intake.setPowerInput(-gamepad1.left_trigger);
            } else {
                robot.intake.setPowerInput(0);
            }
            robot.drivetrain.setPowerDriveMotors(getMotorPowers(x, y, gamepad1.right_stick_x));
        }
    }

    public HashMap<String, Double> getMotorPowers(double x, double y, double rotate) {
        HashMap<String, Double> powers = new HashMap<>();
        powers.put("frontLeft", -x + y + rotate);
        powers.put("backLeft", x + y + rotate);
        powers.put("frontRight", x + y - rotate);
        powers.put("backRight", -x + y - rotate);

        double maxPower = Math.max(Math.max(Math.max(Math.abs(powers.get("frontLeft")),
                Math.abs(powers.get("backLeft"))), Math.abs(powers.get("frontRight"))), Math.abs(powers.get("backRight")));
        if (maxPower > 1) {
            for (Map.Entry<String, Double> mapElement : powers.entrySet()) {
                powers.put(mapElement.getKey(), mapElement.getValue() / maxPower);
            }
        }
        return powers;
    }
}
