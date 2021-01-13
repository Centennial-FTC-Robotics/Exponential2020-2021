package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.exponential.mechanisms.Turret;
import org.exponential.robots.OurRobot;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "FieldCentricTeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends LinearOpMode {
    private double initialAngle;
    private double currentAngle;
    private int targetNumber = 1;//John added this
    private double [] goalXCoords = {85.3, 98.42, 111.54}; //Change these up please (I'll do it)
    private OurRobot ourRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        ourRobot = new OurRobot();
        ourRobot.initialize(this);

        initialAngle = getInitialAngle();

        while (opModeIsActive()) {
            currentAngle = getAngle();

            double inputX = gamepad1.left_stick_x;
            double inputY = -gamepad1.left_stick_y;

            //John's targeting thing
            if (gamepad2.a == true){
                targetNumber = targetNumber + 1;
            }
            ourRobot.turret.moveTurret(toggleTarget(targetNumber));//TODO Eric pls fix this
            //--------------

            double theta = currentAngle - initialAngle;
            double rotatedX = inputX * Math.cos(theta) - inputY * Math.sin(theta);
            double rotatedY = inputX * Math.sin(theta) + inputY * Math.cos(theta);

            ourRobot.drivetrain.setPowerDriveMotors(getMotorPowers(rotatedX, rotatedY, gamepad1.right_stick_x));
        }
    }

    public double getAngle() {
        return 90;
    }

    public double getInitialAngle() {
        return 90;
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

    //John's targeting method
    public double toggleTarget(double targetXCoords){
        targetXCoords = goalXCoords [targetNumber];
        return targetXCoords;
    }
}
