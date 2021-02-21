package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RedFieldCentric", group = "TeleOp")
public class RedFieldCentric extends FieldCentricTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        side = "red";
        super.runOpMode();
    }
}
