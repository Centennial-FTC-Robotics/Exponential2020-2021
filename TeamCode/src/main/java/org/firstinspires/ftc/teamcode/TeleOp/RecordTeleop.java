package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.robots.OurRobot;
import org.exponential.utility.Log;

import java.util.HashMap;
import java.util.Map;

@Disabled
@TeleOp(name = "Recordteleop", group = "TeleOp")
public class RecordTeleop extends LinearOpMode {
    String side;
    private double initialAngle;
    private double currentAngle;

    double headingRotationPower = 0;

    private OurRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        Log log = new Log("sampleTeleopRecord", true);
        waitForStart();

        robot = new OurRobot();
        robot.initialize(this);

        if (side.equals("red")) {
            robot.odometry.setPosition(48, -63, 180);
            initialAngle = robot.odometry.getAngle() + 270;  // +270 so that the controls are field centric from the red drivers' perspective
        } else {
            robot.odometry.setPosition(-48, -63, 180);
            initialAngle = robot.odometry.getAngle() + 90;

        }

        robot.setUpServos();
        boolean raised = true;
        boolean clamped = true;
        boolean paused = false;

        log.start();
        while (opModeIsActive()) {
            robot.odometry.update();
            currentAngle = robot.odometry.getAngle();

            // uncomment to change to robot centric
            /*initialAngle = 0;
            currentAngle = 0;*/

            double inputLeftX = -gamepad1.left_stick_x;
            double inputLeftY = -gamepad1.left_stick_y;
            double inputRightX = .4 * gamepad1.right_stick_x;
            double inputRightY = .4 * gamepad1.right_stick_y;
            double reductionFactor = .5;

            //data added: leftX,leftY,rightX,rightY,a,b,x,y,leftTrig,rightTrig,leftBump,rightBump,dpadR,dpadU,dpadL,dpadD
            log.addData(inputLeftX);
            log.addData(inputLeftY);
            log.addData(inputRightX);
            log.addData(inputRightY);
            log.addData(gamepad1.a);
            log.addData(gamepad1.b);
            log.addData(gamepad1.x);
            log.addData(gamepad1.y);
            log.addData(gamepad1.left_trigger);
            log.addData(gamepad1.right_trigger);
            log.addData(gamepad1.left_bumper);
            log.addData(gamepad1.right_bumper);
            log.addData(gamepad1.dpad_right);
            log.addData(gamepad1.dpad_up);
            log.addData(gamepad1.dpad_left);
            log.addData(gamepad1.dpad_down);
            log.update();

            // halve values
            /*if (gamepad1.left_bumper) {  // commented out because of control conflict
                inputLeftX *= reductionFactor;
                inputLeftY *= reductionFactor;
                inputRightX *= reductionFactor;
                inputRightY *= reductionFactor;
            }*/

            double theta = currentAngle - initialAngle;
            double rotatedX = inputLeftX * Math.cos(Math.toRadians(theta)) - inputLeftY * Math.sin(Math.toRadians(theta));
            double rotatedY = inputLeftX * Math.sin(Math.toRadians(theta)) + inputLeftY * Math.cos(Math.toRadians(theta));

            if (gamepad1.right_trigger > 0 || gamepad2.left_stick_y < 0) {
                robot.intake.intake();
            } else if (gamepad1.left_trigger > 0 || gamepad2.left_stick_y >  0) {
                //robot.intake.setPowerInput(-gamepad1.left_trigger);
                robot.intake.outtake();
            } else {
                //robot.intake.setPowerInput(0);
                robot.intake.stop();
            }
            if (gamepad1.dpad_up) {
                robot.wobbleGoalMover.pickupGoal();
            } else if (gamepad1.dpad_down) {
                robot.wobbleGoalMover.placeGoal();
            }

            //Triple shot
            if (gamepad1.a) {
                robot.loadAndUnloadAllRings();
            }
            //single shot
            if(gamepad2.x) {
                robot.loader.loadAndUnload();
            }
            if (gamepad1.x) {
                robot.shootAtPowerShotTargets(side);
            }
            if (gamepad1.b) {
                robot.shootAtHighGoal(side);
            }
            /*if (gamepad1.right_bumper) {
                robot.scoreWobbleGoal(side);
            }*/
            if (gamepad1.right_bumper) {
                robot.shooter.shootAtHighGoal();
            } else if (gamepad1.left_bumper) {
                robot.shooter.shootAtPowerShot();
            } else {
                robot.shooter.stopShooting();
            }

            telemetry.update();

            robot.drivetrain.setPowerDriveMotors(getMotorPowers(rotatedX, rotatedY, inputRightX));
        }
        log.close();
        robot.odometry.savePosition();
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
