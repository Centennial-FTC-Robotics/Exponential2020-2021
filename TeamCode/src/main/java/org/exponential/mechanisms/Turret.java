package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    private ElapsedTime timer = new ElapsedTime();

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
        //turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();
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

    private double angleArea = 0;
    private double previousTime = 0;

    public void readjustTurretAngle() {
        // call this constantly in the teleop while loop
        double targetAngleRelativeToRobot = 0;
        if (currentCommand == RELOAD) {
            targetAngleRelativeToRobot = 0;
        } else if (currentCommand == POINT_AT_TARGET) {
            targetAngleRelativeToRobot = IMU.normalize(
                    Math.toDegrees(Math.atan2(targetYValue - drivetrain.positioning.yPos,
                            targetXValue - drivetrain.positioning.xPos)) - drivetrain.positioning.angle + 180);
            if (targetAngleRelativeToRobot > 180) {
                targetAngleRelativeToRobot = 180;
            } else if (targetAngleRelativeToRobot < -180) {
                targetAngleRelativeToRobot = -180;
            }
        } else if (currentCommand == POINT_AT_ANGLE) {
            targetAngleRelativeToRobot = IMU.normalize(targetAngle + 180 - drivetrain.positioning.getAngle());
        }
        currentAngle = (turretMotor.getCurrentPosition() - encCountAtAngleZero) / ENC_PER_DEGREE;

        // PID Stuff
        double displacment = IMU.normalize(targetAngleRelativeToRobot) - IMU.normalize(currentAngle);
        double timeChange = -previousTime + (previousTime = timer.seconds());
        if (Math.abs(displacment) > 60) {
            angleArea = 0;
            // so far away that it shouldn't even have a pid working
            turretMotor.setPower(0.65 * Math.signum(displacment));
        } else {
            if (Math.signum(angleArea) != Math.signum(displacment)) {
                // overshoots, sets angle area to 0
                angleArea = 0;
            } else {
                // updates area
                angleArea += timeChange * angleArea;
            }

            double Kp = 0.1;
            double Ki = 0.008;
            if(Math.abs(displacment) > 1.0){
                turretMotor.setPower(Kp * displacment + Ki * angleArea);
            } else {
                turretMotor.setPower(0);
                angleArea = 0;
            }
        }
        opMode.telemetry.addData("displacement", displacment);
        opMode.telemetry.addData("angleArea",angleArea);
        opMode.telemetry.update();
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