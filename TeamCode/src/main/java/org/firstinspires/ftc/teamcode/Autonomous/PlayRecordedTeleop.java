package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.robots.OurRobot;
import org.exponential.utility.FileReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@Autonomous(group="Autonomous", name="PlayRecordedTeleop")
public class PlayRecordedTeleop extends LinearOpMode {
    OurRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OurRobot();
        robot.initialize(this);

        ArrayList<String> inputLines = FileReader.getLinesFromCSVFile("sampleTeleopRecord");


        waitForStart();
        robot.setUpServos();

        // starting on rightmost red tape, facing left
        robot.odometry.setPosition(48, -63, 180);
        double initialAngle = robot.odometry.getAngle() + 270;  // +270 so that the controls are field centric from the red drivers' perspective

        double currentAngle;


        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        for (String line: inputLines) {
            String[] input = line.split(",");
            double iterationTime = Double.parseDouble(input[0]);
            double inputLeftX = Double.parseDouble(input[1]);
            double inputLeftY = Double.parseDouble(input[2]);
            double inputRightX = Double.parseDouble(input[3]);
            double inputRightY = Double.parseDouble(input[4]);
            boolean a = Boolean.parseBoolean(input[5]);
            boolean b = Boolean.parseBoolean(input[6]);
            boolean x = Boolean.parseBoolean(input[7]);
            boolean y = Boolean.parseBoolean(input[8]);
            double left_trigger = Double.parseDouble(input[9]);
            double right_trigger = Double.parseDouble(input[10]);
            boolean left_bumper = Boolean.parseBoolean(input[11]);
            boolean right_bumper = Boolean.parseBoolean(input[12]);
            boolean dpad_right = Boolean.parseBoolean(input[13]);
            boolean dpad_up = Boolean.parseBoolean(input[14]);
            boolean dpad_left = Boolean.parseBoolean(input[15]);
            boolean dpad_down = Boolean.parseBoolean(input[16]);

            while (timer.milliseconds() < iterationTime) {
                robot.odometry.update();
                currentAngle = robot.odometry.getAngle();
            }
            robot.odometry.update();
            currentAngle = robot.odometry.getAngle();

            //input translating -> robot movements here
            double reductionFactor = .5;

            // halve values
            /*if (left_bumper) {
                inputLeftX *= reductionFactor;
                inputLeftY *= reductionFactor;
                inputRightX *= reductionFactor;
                inputRightY *= reductionFactor;
            }*/

            double theta = currentAngle - initialAngle;
            double rotatedX = inputLeftX * Math.cos(Math.toRadians(theta)) - inputLeftY * Math.sin(Math.toRadians(theta));
            double rotatedY = inputLeftX * Math.sin(Math.toRadians(theta)) + inputLeftY * Math.cos(Math.toRadians(theta));

            if (right_trigger > 0) {
                robot.intake.intake();
            } else if (left_trigger > 0) {
                //robot.intake.setPowerInput(-gamepad1.left_trigger);
                robot.intake.outtake();
            } else {
                //robot.intake.setPowerInput(0);
                robot.intake.stop();
            }

            if (dpad_up) {
                robot.wobbleGoalMover.pickupGoal();
            } else if (dpad_down) {
                robot.wobbleGoalMover.placeGoal();
            }

            //Triple shot
            if (a) {
                robot.loadAndUnloadAllRings();
            }
            if (x) {
                robot.shootAtPowerShotTargets("red");
            }
            if (b) {
                robot.shootAtHighGoal("red");
            }

            if (right_bumper) {
                robot.shooter.shootAtHighGoal();
            } else if (left_bumper) {
                robot.shooter.shootAtPowerShot();
            } else {
                robot.shooter.stopShooting();
            }

            robot.drivetrain.setPowerDriveMotors(getMotorPowers(rotatedX, rotatedY, inputRightX));
        }
        robot.odometry.savePosition();
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