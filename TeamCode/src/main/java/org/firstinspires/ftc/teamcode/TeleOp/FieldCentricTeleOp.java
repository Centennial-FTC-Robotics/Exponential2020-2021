package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.robots.OurRobot;

import java.util.HashMap;
import java.util.Map;

@Disabled
@TeleOp(name = "FieldCentricTeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends LinearOpMode {
    String side;
    private double initialAngle;
    private double currentAngle;
    ElapsedTime wobbleToggleTimer = new ElapsedTime();
    boolean wobbleState = true;
    double shooterInaccuracy = 0;
    ElapsedTime shooterTimer = new ElapsedTime();
    double headingRotationPower = 0;

    private OurRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        robot = new OurRobot();
        robot.initialize(this);
        //comment out this line for non transition (testing) purposes
        robot.loadPositions();


        //comment out the following line for auto to teleop transitions
        //robot.odometry.setPosition(0, 0, 90);

        if (side.equals("red")) {
            initialAngle = robot.odometry.getAngle() + 270;  // +270 so that the controls are field centric from the red drivers' perspective
        } else {
            initialAngle = robot.odometry.getAngle() + 90;
        }
        robot.setUpServos();
        boolean raised = true;
        boolean clamped = true;
        robot.turret.pointToReloadPosition();

        while (opModeIsActive()) {
            robot.turret.readjustTurretAngle();
            robot.odometry.update();
            currentAngle = robot.odometry.getAngle();

            // uncomment to change to robot centric
            /*initialAngle = 0;
            currentAngle = 0;*/

            double inputLeftX = -gamepad1.left_stick_x;
            double inputLeftY = -gamepad1.left_stick_y;
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
            } else if (gamepad1.left_trigger > 0 || gamepad2.left_stick_y >  0) {
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
                    robot.wobbleGoalMover.standby();
                } else {
                    robot.wobbleGoalMover.clamp();
                }
                clamped = !clamped;
                sleep(250);
            }

            if (gamepad2.dpad_right) {
                robot.odometry.offsetXPos(-2);
                sleep(150);
            } else if (gamepad2.dpad_left) {
                robot.odometry.offsetXPos(2);
                sleep(150);
            } else if (gamepad2.dpad_up) {
                robot.odometry.offsetYPos(-1);
                sleep(150);
            } else if (gamepad2.dpad_down) {
                robot.odometry.offsetYPos(1);
                sleep(150);
            }

            /*if (gamepad1.x) { //towards back
                Runnable myRunnable = new Runnable(){
                            public void run(){
                                robot.drivetrain.turnTo(-90);  //(270)
                            }};
                Thread thread = new Thread(myRunnable);
                thread.start();
            } else if (gamepad1.b) { //towards goals
                Runnable myRunnable = new Runnable(){
                    public void run(){
                        robot.drivetrain.turnTo(90);  //(270)
                    }};
                Thread thread = new Thread(myRunnable);
                thread.start();
            } else if (gamepad1.a && !gamepad1.start) { //towards drivers
                Runnable myRunnable = new Runnable(){
                    public void run(){
                        robot.drivetrain.turnTo(0);  //(270)
                    }};
                Thread thread = new Thread(myRunnable);
                thread.start();
            } else if (gamepad1.y) { // away from drivers
                Runnable myRunnable = new Runnable(){
                    public void run(){
                        robot.drivetrain.turnTo(180);  //(270)
                    }};
                Thread thread = new Thread(myRunnable);
                thread.start();
            }*/

            if (gamepad1.x) {
                headingRotationPower = robot.headingRotation(-90);
            } else if (gamepad1.y) {
                headingRotationPower = robot.headingRotation (180);
            } else if (gamepad1.a) {
                headingRotationPower = robot.headingRotation(0);
            } else if (gamepad1.b) {
                headingRotationPower = robot.headingRotation (90);
            } else {
                headingRotationPower = 0;
            }
            /*if (gamepad2.b && !gamepad2.start) {
                //robot.shootAtHighGoal(side);
                robot.drivetrain.moveTo(44, 0, 270);
                robot.drivetrain.turnTo(270);

            }*/ //we already have another thing for b so I commented this out -John
            //Triple shot
            if (gamepad2.y) {
                robot.loadAndUnloadAllRings();
            }
            //single shot
            if(gamepad2.x) {
                robot.loader.loadAndUnload();
            }
            if (gamepad2.a) {
                robot.shootPowerShotTeleOp();
                robot.shootOtherPowerShot();
                robot.shootOtherPowerShot();
                /*robot.loader.loadAndUnload();
                sleep(100);
                robot.shooter.adjustedPowerShot();

                robot.drivetrain.moveTo(robot.odometry.getxPos() - 5, robot.odometry.getyPos(), 270);
                robot.drivetrain.performBrake();
                sleep(500);
                robot.loader.loadAndUnload();

                robot.drivetrain.moveTo(robot.odometry.getxPos() - 6.5, robot.odometry.getyPos(), 270);
                robot.drivetrain.performBrake();
                sleep(500);
                robot.loader.loadAndUnload();*/
            }
            /*if (gamepad2.b) {
                robot.shootAtHighGoal(side);
            }*/
            /*if (gamepad1.right_bumper) {
                robot.scoreWobbleGoal(side);
            }*/
            if (gamepad2.right_bumper) {
                robot.shooter.shootAtHighGoal();
            } else if (gamepad2.left_trigger > 0) {
                robot.shooter.shootAtPowerShot();
            } else {
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

            telemetry.update();

            robot.drivetrain.setPowerDriveMotors(getMotorPowers(rotatedX, rotatedY, inputRightX + headingRotationPower));
        }
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
