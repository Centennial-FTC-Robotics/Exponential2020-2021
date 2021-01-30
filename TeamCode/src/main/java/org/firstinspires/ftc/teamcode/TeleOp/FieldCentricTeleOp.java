package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.robots.OurRobot;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "FieldCentricTeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends LinearOpMode {
    private double initialAngle;
    private double currentAngle;

    private OurRobot ourRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        ourRobot = new OurRobot();
        ourRobot.initialize(this);
        // ourRobot.odometry.loadPosition();
        ourRobot.odometry.setPosition(0, 60, 90);
        initialAngle = ourRobot.odometry.getAngle();

        ourRobot.intake.setServoPositions();
        ourRobot.wobbleGoalMover.raise();
        //ourRobot.turret.setTarget(25, 25);

        while (opModeIsActive()) {
            ourRobot.odometry.update();
            currentAngle = ourRobot.odometry.getAngle();

            // uncomment to change to robot centric
            /*
            initialAngle = 0;
            currentAngle = 0;
             */
            double inputLeftX = -gamepad1.left_stick_x;
            double inputLeftY = -gamepad1.left_stick_y;
            double inputRightX = gamepad1.right_stick_x;
            double inputRightY = gamepad1.right_stick_y;
            double reductionFactor = .5;
            // halve values
            if (gamepad1.left_bumper) {
                inputLeftX *= reductionFactor;
                inputLeftY *= reductionFactor;
                inputRightX *= reductionFactor;
                inputRightY *= reductionFactor;
            }

            //John's targeting thing

            // ourRobot.turret.moveTurret();//TODO Eric pls fix this

            double theta = currentAngle - initialAngle;
            double rotatedX = inputLeftX * Math.cos(theta) - inputLeftY * Math.sin(theta);
            double rotatedY = inputLeftX * Math.sin(theta) + inputLeftY * Math.cos(theta);

            if (gamepad1.right_trigger > 0) {
                ourRobot.intake.setPowerInput(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                ourRobot.intake.setPowerInput(-gamepad1.left_trigger);
            } else {
                ourRobot.intake.setPowerInput(0);
            }


            if (gamepad1.a) {
                ourRobot.loader.load();
                ourRobot.shooter.speedBackUp();
                sleep(250);
                ourRobot.loader.unload();
            } else if (gamepad1.b) {
                ourRobot.loader.unload();
            }
            if (gamepad1.right_bumper) {
                ourRobot.shooter.shootAtHighGoal();
            }
            if (gamepad1.x) {
                ourRobot.loader.load();
                sleep (250);
                ourRobot.loader.unload();
            }
            if (gamepad1.x) {
                ourRobot.shootAtHighGoal("red");
            } else if (gamepad1.y) {
                ourRobot.shootPowerShotTargets("red");
            }

            if (gamepad1.right_bumper) {
                ourRobot.shooter.shootAtPowerShot();
            } else {
                ourRobot.shooter.stopShooting();
            }
            ourRobot.drivetrain.setPowerDriveMotors(getMotorPowers(rotatedX, rotatedY, inputRightX));
        }
    }

    public static double[] rotatePoint(double x, double y, double theta) {
        double newX = x * Math.cos(theta) - y * Math.sin(theta);
        double newY = x * Math.sin(theta) + y * Math.cos(theta);
        return new double[]{newX, newY};
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
