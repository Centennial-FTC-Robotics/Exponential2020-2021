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

    private OurRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        robot = new OurRobot();
        robot.initialize(this);
        //comment out this line for non transition (testing) purposes
        //robot.loadPositions();

        //comment out the following two lines for auto to teleop transitions
        robot.odometry.setPosition(0, 0, 90);
        robot.turret.setAngle(0);

        initialAngle = robot.odometry.getAngle() + 90;  // +90 so that the controls are field centric from the red drivers' perspective
        int bumperTest;

        robot.setUpServos();
        //ourRobot.turret.setTarget(25, 25);
        robot.turret.pointToReloadPosition();
        boolean raised = true;
        boolean clamped = true;
        while (opModeIsActive()) {
            robot.odometry.update();
            currentAngle = robot.odometry.getAngle();

            // move the turret to correct position
            robot.turret.readjustTurretAngle();

            // uncomment to change to robot centric
            /*initialAngle = 0;
            currentAngle = 0;*/

            double inputLeftX = -gamepad1.left_stick_x;
            double inputLeftY = -gamepad1.left_stick_y;
            double inputRightX = .3 * gamepad1.right_stick_x;
            double inputRightY = .3 * gamepad1.right_stick_y;
            double reductionFactor = .5;
            // halve values
            if (gamepad1.left_bumper) {
                inputLeftX *= reductionFactor;
                inputLeftY *= reductionFactor;
                inputRightX *= reductionFactor;
                inputRightY *= reductionFactor;
            }

            double theta = currentAngle - initialAngle;
            double rotatedX = inputLeftX * Math.cos(Math.toRadians(theta)) - inputLeftY * Math.sin(Math.toRadians(theta));
            double rotatedY = inputLeftX * Math.sin(Math.toRadians(theta)) + inputLeftY * Math.cos(Math.toRadians(theta));

            if (gamepad1.right_trigger > 0 || gamepad2.left_trigger > 0) {
                //robot.intake.setPowerInput(gamepad1.right_trigger);
                robot.intake.intake();
            } else if (gamepad1.left_trigger > 0 || gamepad2.right_trigger > 0) {
                //robot.intake.setPowerInput(-gamepad1.left_trigger);
                robot.intake.outtake();
            } else {
                //robot.intake.setPowerInput(0);
                robot.intake.stop();
            }


            /*if (gamepad1.dpad_up) {
                *//*ourRobot.loader.load();
                //ourRobot.shooter.speedBackUp();
                sleep(250);
                ourRobot.loader.unload();*//*
                robot.loader.loadAndUnload();
            } *//*else if (gamepad1.b) {
                ourRobot.loader.unload();
            }*//*
            if (gamepad1.a) {
                if (raised) {
                    robot.wobbleGoalMover.lower();
                } else {
                    robot.wobbleGoalMover.raise();
                }
                raised = !raised;
                sleep(250);
            }
            if (gamepad1.b) {
                if (clamped) {
                    robot.wobbleGoalMover.release();
                } else {
                    robot.wobbleGoalMover.clamp();
                }
                clamped = !clamped;
                sleep(250);
            }*/

            if (gamepad1.dpad_up) {
                robot.wobbleGoalMover.raise();
            } else if (gamepad1.dpad_down) {
                robot.wobbleGoalMover.lower();
            } else if (gamepad1.dpad_left) {
                robot.wobbleGoalMover.release();
            } else if (gamepad1.dpad_right) {
                robot.wobbleGoalMover.clamp();
            }


            if (gamepad1.x) { //towards back
                robot.drivetrain.turnTo(-90);  //(270)
            } else if (gamepad1.b) { //towards goals
                robot.drivetrain.turnTo(90);
            } else if (gamepad1.a) { //towards drivers
                robot.drivetrain.turnTo(0);
            } else if (gamepad1.y) { // away from drivers
                robot.drivetrain.turnTo(180);
            }
            /*if (gamepad1.dpad_down) {
                robot.shootAtHighGoal("red");
            } *//*else if (gamepad1.y) {
                ourRobot.shootPowerShotTargets("red");
            }*//*

            if (gamepad1.dpad_up) {
                robot.scoreWobbleGoal("red");
            }
*/
            if (gamepad2.b) {
                robot.shootAtHighGoal("red");
            }
            if (gamepad2.x) {
                robot.loader.loadAndUnload();
            }
            if (gamepad2.right_bumper) {
                robot.shooter.shootAtHighGoal();
            } else {
                robot.shooter.stopShooting();
            }
            if (gamepad1.left_bumper) {
                robot.turret.pointToReloadPosition();
            }

            if (gamepad2.dpad_up) {
                robot.turret.setTarget(36, 72); // aim at high goal
                robot.turret.pointAtTarget();
            } else if (gamepad2.dpad_left) {
                robot.turret.setTarget(2, 72); // left power shot
                robot.turret.pointAtTarget();

            } else if (gamepad2.dpad_down) {
                robot.turret.setTarget(9.5, 72); // middle power shot
                robot.turret.pointAtTarget();

            } else if (gamepad2.dpad_right) {
                robot.turret.setTarget(17, 72); // right power shot
                robot.turret.pointAtTarget();
            }

            /*if (gamepad1.right_bumper) {
                robot.shooter.shootAtPowerShot();
            } *//*else if (gamepad1.left_bumper) {
                ourRobot.shooter.shootAtHighGoal();
            }*//*
            if (gamepad1.right_bumper) {
                bumperTest = 1;
            } else {
                bumperTest = 0;
            }
            telemetry.addData("bumper",bumperTest);*/
            telemetry.update();
            robot.drivetrain.setPowerDriveMotors(getMotorPowers(rotatedX, rotatedY, inputRightX));
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
