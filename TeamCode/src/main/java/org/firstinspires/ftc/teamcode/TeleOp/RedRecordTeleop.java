package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RedRecordTeleop", group = "TeleOp")
public class RedRecordTeleop extends RecordTeleop {
    @Override
    public void runOpMode() throws InterruptedException {
        side = "red";
        super.runOpMode();
    }
}
