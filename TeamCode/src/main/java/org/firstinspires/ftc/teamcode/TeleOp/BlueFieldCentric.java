package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlueFieldCentric", group = "TeleOp")
public class BlueFieldCentric extends FieldCentricTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        side = "blue";
        super.runOpMode();
    }
}