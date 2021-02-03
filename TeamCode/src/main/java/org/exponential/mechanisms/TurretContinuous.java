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

public class TurretContinuous implements Mechanism, Runnable {
    public static final double ENC_PER_DEGREE = (537.6 * 2) / 360;
    public static final boolean POINT_AT_TARGET = true;
    public static final boolean RELOAD = false;


    public DcMotorEx turretMotor;
    Drivetrain drivetrain;
    LinearOpMode opMode;
    double targetXValue;
    double targetYValue;
    public boolean currentCommand = POINT_AT_TARGET;
    public double currentAngle = 0;

    private int previousPos;
    // if the turret was facing directly forwards, what could the encoder count be?
    double encCountAtAngleZero = 0;


    public void initialize(LinearOpMode opMode) {
        this.opMode = opMode;
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(.5);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        previousPos = turretMotor.getCurrentPosition();
    }

    public TurretContinuous(Drivetrain drivetrain) {
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
        double targetAngle;
        if (currentCommand == RELOAD) {
            targetAngle = 0;
        } else {
            targetAngle = IMU.normalize(
                    Math.toDegrees(Math.atan2(targetYValue - drivetrain.positioning.yPos,
                            targetXValue - drivetrain.positioning.xPos)) - drivetrain.positioning.angle + 180);
        }
        previousPos = turretMotor.getCurrentPosition();
        turretMotor.setTargetPosition(previousPos
                + (int) (ENC_PER_DEGREE * IMU.normalize(targetAngle - currentAngle)));
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(.5);
        currentAngle += IMU.normalize((previousPos - encCountAtAngleZero) / ENC_PER_DEGREE - currentAngle);
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