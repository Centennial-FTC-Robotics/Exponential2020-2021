package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.robots.OurRobot;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "TurretTeleOp", group = "TeleOp")
public class TurretTeleOp extends LinearOpMode {
    String side = "red";
    private double initialAngle;
    private double currentAngle;
    ElapsedTime wobbleToggleTimer = new ElapsedTime();
    boolean wobbleState = true;
    ElapsedTime turretTimer = new ElapsedTime();
    double shooterInaccuracy = 0;
    ElapsedTime shooterTimer = new ElapsedTime();
    double headingRotationPower = 0;
    double xPos;
    double yPos;
    //private String turretState = "something";

    private OurRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        robot = new OurRobot();
        robot.initialize(this);
        //comment out this line for non transition (testing) purposes
        //robot.loadPositions();


        //comment out the following line for auto to teleop transitions
        robot.odometry.setPosition(0, 0, 90);

        /*if (side.equals("red")) {
            initialAngle = robot.odometry.getAngle() + 270;  // +270 so that the controls are field centric from the red drivers' perspective
        } else {
            initialAngle = robot.odometry.getAngle() + 90;
        }*/
        robot.setUpServos();
        robot.turret.pointToReloadPosition();
        boolean raised = true;
        boolean clamped = true;
        while (opModeIsActive()) {
            robot.odometry.update();
            currentAngle = robot.odometry.getAngle();

            // uncomment to change to robot centric
            /*initialAngle = 0;
            currentAngle = 0;*/
            xPos = robot.odometry.getxPos();
            yPos = robot. odometry. getyPos();
            if (gamepad2.dpad_up) {
                //Aiming straight at high goal or power shot
                robot.turret.setTarget(xPos, yPos + 40);
                robot.turret.pointAtTarget();
            } else if (gamepad2.dpad_down){
                //Aiming from the middle to high goal
                robot.turret.setTarget(xPos + 18, yPos + 40);
                robot.turret.pointAtTarget();
            } else if (gamepad2.dpad_right) {
                //aim to the right power shot from middle
                robot.turret.setTarget(xPos + 7, yPos + 40);
                robot.turret.pointAtTarget();
            } else if (gamepad2.dpad_left) {
                //aim to the left power shot from middle
                robot.turret.setTarget(xPos - 7, yPos + 40);
                robot.turret.pointAtTarget();
            }else if (gamepad2.left_bumper) {
                robot.turret.pointToReloadPosition();
            }

            double inputLeftX = gamepad1.left_stick_x;
            double inputLeftY = gamepad1.left_stick_y;
            double inputRightX = .5 * gamepad1.right_stick_x;
            double inputRightY = .5 * gamepad1.right_stick_y;
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

            if (gamepad1.right_trigger > 0 || gamepad2.left_stick_y < 0) {
                robot.intake.intake();
            } else if (gamepad1.left_trigger > 0 || gamepad2.left_stick_y > 0) {
                //robot.intake.setPowerInput(-gamepad1.left_trigger);
                robot.intake.outtake();
            } else {
                //robot.intake.setPowerInput(0);
                robot.intake.stop();
            }

            if (gamepad1.dpad_up) {
                robot.wobbleGoalMover.raise();
            } else if (gamepad1.dpad_down) {
                robot.wobbleGoalMover.release();
                sleep(50);
                robot.wobbleGoalMover.lower();
            } else if (gamepad1.dpad_right) {
                if (clamped) {
                    robot.wobbleGoalMover.release();
                } else {
                    robot.wobbleGoalMover.clamp();
                }
                clamped = !clamped;
                sleep(250);
            }
            if (gamepad1.x) {
                headingRotationPower = robot.headingRotation(-90);
            } else if (gamepad1.y) {
                headingRotationPower = robot.headingRotation(180);
            } else if (gamepad1.a) {
                headingRotationPower = robot.headingRotation(0);
            } else if (gamepad1.b) {
                headingRotationPower = robot.headingRotation(90);
            } else {
                headingRotationPower = 0;
            }

            if (gamepad1.right_bumper) {
                rotatedX = 0;
                rotatedY = 0;
                inputRightX = 0;

                robot.shooter.shootAtHighGoal();
                sleep(1000);
                robot.loadAndUnloadAllRings();

            } else {
                robot.shooter.setPower(0);
            }

            /*if (gamepad2.b && !gamepad2.start) {
                //robot.shootAtHighGoal(side);
                robot.drivetrain.moveTo(44, 0, 270);
                robot.drivetrain.turnTo(270);

            }*/ //we already have another thing for b so I commented this out -John
            //Triple shot
            /*if (gamepad2.y) {
                robot.loadAndUnloadAllRings();
            }

            /* //Auto Power Shot
            } else if (gamepad2.x) {
                //targetX was 2
                robot.shooter.shootAtPowerShot();
                robot.turret.setTarget(16, 72); // right power shot
                robot.turret.pointAtTarget();
                //turretState = "left power shot";
                sleep(1500);
                robot.loader.loadAndUnload();
                sleep(200);

                robot.turret.setTarget(8, 72); // middle power shot
                robot.turret.pointAtTarget();
                //turretState = "middle power shot";
                sleep(800);
                robot.loader.loadAndUnload();
                sleep(800);

                robot.turret.setTarget(2, 72); // left power shot
                robot.turret.pointAtTarget();
                //turretState = "right power shot";
                sleep(500);
                robot.loader.loadAndUnload();
                sleep(200);


                robot.shooter.stopShooting();
                robot.turret.pointToReloadPosition();
            }*/
            //Reload position


            //single shot
            if(gamepad2.x) {
                robot.drivetrain.performBrake();
                robot.loader.loadAndUnload();
            }
            //triple shot
            if (gamepad2.y) {
                robot.drivetrain.performBrake();
                robot.turretShootThree();
            }
                //turretState = "reload";
            /*if (gamepad1.right_bumper) {
                robot.scoreWobbleGoal(side);
            }*/
            if (gamepad2.right_bumper) {
                robot.shooter.shootAtHighGoal();
            } else if (gamepad2.right_trigger > 0) {
                robot.shooter.shootAtPowerShot();
            } else if (gamepad2.left_trigger > 0){
                robot.shooter.stopShooting();
            }

/*
            if (gamepad2.dpad_up) {
                 // aim at high goal
                robot.drivetrain.moveTo(36, -6, 270);

            } else if (gamepad2.dpad_left) { //left power shot
                robot.drivetrain.moveTo(2, -6, 270);

            } else if (gamepad2.dpad_down) { //center power shot
                robot.drivetrain.moveTo(9.5, -6, 270);

            } else if (gamepad2.dpad_right) { //right power shot
                robot.drivetrain.moveTo(17, -6, 270);

            }*/
            telemetry.addData("bumper", gamepad2.right_bumper);
            telemetry.update();

                robot.drivetrain.setPowerDriveMotors(getMotorPowers(rotatedX, rotatedY, inputRightX + headingRotationPower));
            }
            //robot.odometry.savePosition();
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
