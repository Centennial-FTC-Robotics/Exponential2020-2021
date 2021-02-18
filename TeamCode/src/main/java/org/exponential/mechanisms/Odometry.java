package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.exponential.superclasses.Mechanism;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Odometry implements Runnable, Mechanism {
    // so that the imu doesn't slow down the while loop too much
    public int iterationsPerIMURead = 100; // ratio of encoder reads to imu reads
    private int currentIterationsSinceIMURead = 0;

    public DcMotorEx forwardLeftEnc;
    public DcMotorEx forwardRightEnc;
    public DcMotorEx horizontalEnc;

    LinearOpMode opMode;

    IMU imu;

    // all in field centric
    double xPos;
    double yPos;
    double angle;
    double xVel;
    double yVel;
    double angleVel;

    public int lastLeftEncPos;
    public int lastRightEncPos;
    public int lastHoriEncPos;

    public double horiEncPerDegree = 33;
    public double vertLeftEncPerDegree = -137.342;
    public double vertRightEncPerDegree = 157.898;

    ElapsedTime updateTimer;

    /*public Odometry(IMU imu, DcMotorEx leftEncoder, DcMotorEx backEncoder, DcMotorEx rightEncoder) {
        forwardLeftEnc = leftEncoder;
        forwardRightEnc = rightEncoder;
        horizontalEnc = backEncoder;
        this.imu = imu;
    }*/
    public Odometry(IMU imu) {
        this.imu = imu;
    }

    // for testing
    int imuUpdates = 0;
    int encAngleUpdates = 0;
    @Override
    public void initialize(LinearOpMode opMode) {
        /*forwardLeftEnc = opMode.hardwareMap.get(DcMotorEx.class, "leftOdometry");
        forwardRightEnc = opMode.hardwareMap.get(DcMotorEx.class, "rightOdometry");
        horizontalEnc = opMode.hardwareMap.get(DcMotorEx.class, "backOdometry");*/
        forwardLeftEnc = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        horizontalEnc = opMode.hardwareMap.get(DcMotorEx.class, "conveyorMotor");
        forwardRightEnc = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");

        forwardLeftEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        forwardRightEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        forwardLeftEnc.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        forwardRightEnc.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        forwardRightEnc.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalEnc.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        this.opMode = opMode;
        updateTimer = new ElapsedTime();
    }

    public void setEncoders(DcMotorEx leftEncoder, DcMotorEx backEncoder, DcMotorEx rightEncoder) {
        forwardLeftEnc = leftEncoder;
        forwardRightEnc = rightEncoder;
        horizontalEnc = backEncoder;
    }

    @Override
    public void run() {
        while (opMode.opModeIsActive()) {
            update();
        }
    }

    public double[] toFieldCentric(double robotX, double robotY) {
        double angleRad = Math.toRadians(imu.angle);
        double centricX = robotX * Math.cos(angleRad - Math.PI / 2) - robotY * Math.sin(angleRad - Math.PI / 2);
        double centricY = robotY * Math.cos(angleRad - Math.PI / 2) + robotX * Math.sin(angleRad - Math.PI / 2);

        return new double[]{centricX, centricY};
    }

    public double[] toRobotCentric(double fieldX, double fieldY) {
        double angleRad = Math.toRadians(imu.angle);
        double robotX = fieldX * Math.cos(-angleRad + Math.PI / 2) - fieldY * Math.sin(-angleRad + Math.PI / 2);
        double robotY = fieldY * Math.cos(-angleRad + Math.PI / 2) + fieldX * Math.sin(-angleRad + Math.PI / 2);

        return new double[]{robotX, robotY};
    }

    public double encToInch(double encoders) {
        return 2 * Math.PI * 0.984252 / 8192 * encoders;
    }

    public void update() {
        double timeElapsed = updateTimer.seconds();
        updateTimer.reset();

        // opMode.telemetry.addData("ratio", horizontalEnc.getCurrentPosition()/angle);

        int leftEncChange = -(forwardLeftEnc.getCurrentPosition() - lastLeftEncPos);
        int rightEncChange = -(forwardRightEnc.getCurrentPosition() - lastRightEncPos);
        int horiEncChange = (horizontalEnc.getCurrentPosition() - lastHoriEncPos);

        // does not call getCurrentPosition a second time because you would not account for encoder
        // readings from the time between the two calls
        lastLeftEncPos -= leftEncChange;
        lastRightEncPos -= rightEncChange;
        lastHoriEncPos += horiEncChange;

        // updates angle
        double changeInAngle;
        if (currentIterationsSinceIMURead == iterationsPerIMURead) {
            currentIterationsSinceIMURead = 0;
            imu.update();
            changeInAngle = imu.angle - angle;
            opMode.telemetry.addData("Angle Read Type: ", "IMU");

            update(timeElapsed, changeInAngle, leftEncChange, rightEncChange, horiEncChange, true);
            imuUpdates++;

        } else {
            currentIterationsSinceIMURead++;
            changeInAngle = getAngleChange(leftEncChange, rightEncChange);

            opMode.telemetry.addData("Angle Read Type: ", "Encoder");
            update(timeElapsed, changeInAngle, leftEncChange, rightEncChange, horiEncChange, false);
            encAngleUpdates++;
        }
        opMode.telemetry.addData("imu and enc updates: ", imuUpdates+", "+encAngleUpdates);
    }

    public void update(double timeElapsed, double changeInAngle, int leftEncChange, int rightEncChange, int horiEncChange, boolean imuAngleUpdate) {

        // updates position, velocity, and angle according to how much time has elapsed
        // currently in robot centric
        double[] changeInPos = new double[]{
                encToInch(horiEncChange - horiEncPerDegree * changeInAngle),
                encToInch(weightedAverage(leftEncChange, rightEncChange))
        };

        // changes to field centric displacement vector
        changeInPos = toFieldCentric(changeInPos[0], changeInPos[1]);

        xPos += changeInPos[0];
        yPos += changeInPos[1];
        xVel = changeInPos[0] / timeElapsed;
        yVel = changeInPos[1] / timeElapsed;
        angle += changeInAngle;
        angleVel = changeInAngle / timeElapsed;

    }


    public void savePosition() {
        File file = AppUtil.getInstance().getSettingsFile("RobotPosition.txt");
        String contents = xPos + ":" + yPos + ":" + angle;
        ReadWriteFile.writeFile(file, contents);
    }

    public void loadPosition() {
        File file = AppUtil.getInstance().getSettingsFile("RobotPosition.txt");
        String[] contents = ReadWriteFile.readFile(file).trim().split(":");
        setPosition(Double.parseDouble(contents[0]), Double.parseDouble(contents[1]), Double.parseDouble(contents[2]));
    }

    public void setPosition(double x, double y, double angle) {
        this.xPos = x;
        this.yPos = y;
        this.angle = angle;
        imu.angle = angle;
    }

    public double weightedAverage(int leftEncChange, int rightEncChange) {
        return (vertRightEncPerDegree * leftEncChange - vertLeftEncPerDegree * rightEncChange) / (vertRightEncPerDegree - vertLeftEncPerDegree);
    }

    public double getAngleChange(int leftEncChange, int rightEncChange) {
        return (rightEncChange - leftEncChange) / (vertRightEncPerDegree - vertLeftEncPerDegree);
    }

    public void offsetXPos(double dx) {
        xPos = xPos + dx;
    }

    public void offsetYPos(double dy) {
        yPos = yPos + dy;
    }

    public double getxPos() {
        return xPos;
    }

    public double getyPos() {
        return yPos;
    }

    public double getAngle() {
        return angle;
    }

    public double getxVel() {
        return xVel;
    }

    public double getyVel() {
        return yVel;
    }

    public double getAngleVel() {
        return angleVel;
    }
}
