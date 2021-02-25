package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.exponential.superclasses.Mechanism;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Turret implements Mechanism, Runnable {
    public static final double ENC_PER_DEGREE = (8192.0) / 360.0;
    public static final int POINT_AT_TARGET = 0;
    public static final int RELOAD = 1;
    public static final int POINT_AT_ANGLE = 2;
    public double SHOOTER_INNACURACY = 0;

    public DcMotorEx turretMotor;
    Drivetrain drivetrain;
    LinearOpMode opMode;
    double targetXValue;
    double targetYValue;
    double targetAngle;
    public int currentCommand = POINT_AT_TARGET;
    public double currentAngle = 0;

    // if the turret was facing directly forwards, what could the encoder count be?
    public double encCountAtAngleZero = 0;


    public void initialize(LinearOpMode opMode) {
        this.opMode = opMode;
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "turretMotor");
        /*
        PIDFCoefficients oldCoeffi = turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients newCoeffi = new PIDFCoefficients(oldCoeffi.p, oldCoeffi.i*1.5, oldCoeffi.d, oldCoeffi.f);
        turretMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, newCoeffi);
        */
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        //turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1); //set to 0 to not move

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx) turretMotor.getController();

        // get the port number of our configured motor.
        int motorIndex = ((DcMotorEx) turretMotor).getPortNumber();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients pidOrig = motorControllerEx.getPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_TO_POSITION);

        // change coefficients.
        PIDCoefficients pidNew = new PIDCoefficients(3 * pidOrig.p * 288.0/8192.0, 3 * pidOrig.i* 288.0/8192.0, 1 * pidOrig.d* 288.0/8192.0);
        motorControllerEx.setPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_TO_POSITION, pidNew);

        // re-read coefficients and verify change.
        PIDCoefficients pidModified = motorControllerEx.getPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_TO_POSITION);

        turretMotor.setTargetPositionTolerance(1);
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


    public void pointToReloadPosition() {
        currentCommand = RELOAD;
        readjustTurretAngle();
    }

    public void pointAtAngle() {
        currentCommand = POINT_AT_ANGLE;
        readjustTurretAngle();
    }

    public void readjustTurretAngle() {
        // call this constantly in the teleop while loop
        if (currentCommand == RELOAD) {
            turretMotor.setTargetPosition((int) (encCountAtAngleZero));
        } else if (currentCommand == POINT_AT_TARGET) {
            double targetAngle = IMU.normalize(
                    Math.toDegrees(Math.atan2(targetYValue - drivetrain.positioning.yPos,
                            targetXValue - drivetrain.positioning.xPos)) - drivetrain.positioning.angle + 180);
            // redundant, a relic of the past.........+...
            if (targetAngle > 180) {
                targetAngle = 180;
            } else if (targetAngle < -180) {
                targetAngle = -180;
            }
            turretMotor.setTargetPosition((int) (encCountAtAngleZero + ENC_PER_DEGREE * targetAngle));
            opMode.telemetry.addData("target angle", targetAngle);
        } else if (currentCommand == POINT_AT_ANGLE) {
            turretMotor.setTargetPosition((int) (encCountAtAngleZero +
                    ENC_PER_DEGREE * IMU.normalize(targetAngle + 180 - drivetrain.positioning.getAngle())));
        }
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);
        currentAngle = (turretMotor.getCurrentPosition() - encCountAtAngleZero) / ENC_PER_DEGREE;
        /*opMode.telemetry.addData("turret enc: ", turretMotor.getCurrentPosition());
        opMode.telemetry.addData("current turret angle (in terms of field): ", IMU.normalize(currentAngle + drivetrain.positioning.getAngle() + 180));
        opMode.telemetry.addData("current turret angle (relative to turret): ", currentAngle);
        opMode.telemetry.addData("delta x: ", targetXValue - drivetrain.positioning.getxPos());
        opMode.telemetry.addData("delta y: ", targetYValue - drivetrain.positioning.getyPos());*/
    }

    public void setAngle(double angle) {
        currentAngle = angle;
        encCountAtAngleZero = turretMotor.getCurrentPosition() - currentAngle * ENC_PER_DEGREE;
    }

    public void setTargetAngle(double angleDeg) {
        targetAngle = angleDeg;
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