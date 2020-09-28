package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "BasicTeleOp", group = "TeleOp")
public class BasicTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;


        }
    }

    public HashMap<String, Double> getMotorPowers(double x, double y, double rotate) {
        HashMap<String, Double> powers = new HashMap<>();
        powers.put("frontLeft", -x + y + rotate);
        powers.put("backLeft", x + y + rotate);
        powers.put("frontRight", x + y - rotate);
        powers.put("backRight", -x + y - rotate);

        double maxPower = Math.max(Math.max(Math.max(powers.get("frontLeft"), powers.get("backLeft")), powers.get("frontRight")), powers.get("backRight"));
        if (maxPower > 1) {
            for (Map.Entry<String, Double> mapElement : powers.entrySet()) {
                powers.put(mapElement.getKey(), mapElement.getValue() / maxPower);
            }
        }
        return powers;
    }
}
