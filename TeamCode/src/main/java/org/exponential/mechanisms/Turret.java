package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.exponential.superclasses.Mechanism;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Turret implements Mechanism, Runnable {
    public static final double ENC_PER_DEGREE = (537.6 * 2) / 360;
    public static final boolean POINT_AT_TARGET = true;
    public static final boolean RELOAD = false;


    DcMotorEx turretMotor;
    Drivetrain drivetrain;
    LinearOpMode opMode;
    double targetXValue;
    double targetYValue;
    public boolean currentCommand = POINT_AT_TARGET;
    public double currentAngle = 0;

    // if the turret was facing directly forwards, what could the encoder count be?
    double encCountAtAngleZero = 0;


    public void initialize(LinearOpMode opMode) {
        this.opMode = opMode;
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());

        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(.4);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // 59 in x, 236.22 in y.
    }

    public Turret(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }


    public void setTarget(double targetXValue, double targetYValue) {
        this.targetXValue = targetXValue;
        this.targetYValue = targetYValue;
    }

    public void pointAtTarget() {
        currentCommand = POINT_AT_TARGET;
        readjustTurretAngle();
    }


    public void reload() {
        currentCommand = RELOAD;
        readjustTurretAngle();
    }


    public void readjustTurretAngle() {
        // call this constantly in the teleop while loop
        if (currentCommand == RELOAD) {
            turretMotor.setTargetPosition((int) (encCountAtAngleZero));
        } else {
            double targetAngle = IMU.normalize(
                    Math.toDegrees(Math.atan2(targetYValue - drivetrain.positioning.yPos,
                            targetXValue - drivetrain.positioning.xPos)) - drivetrain.positioning.angle);
            if (targetAngle > 90) {
                targetAngle = 90;
            } else if (targetAngle < -90) {
                targetAngle = -90;
            }
            turretMotor.setTargetPosition((int) (encCountAtAngleZero + ENC_PER_DEGREE * targetAngle));
        }
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.4);
        currentAngle = (turretMotor.getCurrentPosition()-encCountAtAngleZero)/ENC_PER_DEGREE;
        opMode.telemetry.addData("turret enc", turretMotor.getCurrentPosition());
    }


    public void savePosition() {
        File file = AppUtil.getInstance().getSettingsFile("TurretPosition.txt");
        String contents = "" + currentAngle;
        ReadWriteFile.writeFile(file, contents);
    }

    public void loadPosition() {
        File file = AppUtil.getInstance().getSettingsFile("TurretPosition.txt");
        currentAngle = Double.parseDouble(ReadWriteFile.readFile(file));
        encCountAtAngleZero = turretMotor.getCurrentPosition() - currentAngle * ENC_PER_DEGREE;
    }

    @Override
    public void run() {
        while (opMode.opModeIsActive()) {
            readjustTurretAngle();
        }
    }
}