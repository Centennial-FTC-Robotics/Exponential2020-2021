package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlueRecordTeleop", group = "TeleOp")
public class BlueRecordTeleop extends RecordTeleop {
    @Override
    public void runOpMode() throws InterruptedException {
        side = "blue";
        super.runOpMode();
    }
}