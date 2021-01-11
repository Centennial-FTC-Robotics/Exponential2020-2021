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
    DcMotorEx forwardLeftEnc;
    DcMotorEx forwardRightEnc;
    DcMotorEx horizontalEnc;

    LinearOpMode opMode;

    IMU imu;

    // all in field centric
    double xPos;
    double yPos;
    double angle;
    double xVel;
    double yVel;
    double angleVel;

    // encapsulated variables (yuhwan: why make these private and not everything else?)
    // eric: cause everything above are stuff you wanna access in drivetrain class, I didn't make an getters
    // or at least that was the intention until I had to specialize it, ur right
    int lastLeftEncPos;
    int lastRightEncPos;
    int lastHoriEncPos;

    double horiEncPerDegree = 0; //TODO: Find this value

    ElapsedTime updateTimer;

    public Odometry(IMU imu) {
        this.imu = imu;
    }

    @Override
    public void initialize(LinearOpMode opMode) {
        forwardLeftEnc = opMode.hardwareMap.get(DcMotorEx.class, "leftOdometry");
        forwardRightEnc = opMode.hardwareMap.get(DcMotorEx.class, "rightOdometry");
        horizontalEnc = opMode.hardwareMap.get(DcMotorEx.class, "backOdometry");

        forwardLeftEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        forwardRightEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEnc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        forwardLeftEnc.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        forwardRightEnc.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        forwardRightEnc.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalEnc.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        // TODO: make sure that the encoders are in the right directions, so maybe reverse directions if needed
        this.opMode = opMode;
        updateTimer = new ElapsedTime();
    }

    @Override
    public void run() {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive()) {
            update(timer.seconds());
            timer.reset();
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
        return 2*Math.PI*0.984252/8192 * encoders;
    }

    public void update(double timeElapsed) {
        updateTimer.reset();

        // updates position, velocity, and angle according to how much time has elapsed

        int leftEncChange = forwardLeftEnc.getCurrentPosition() - lastLeftEncPos;
        int rightEncChange = forwardRightEnc.getCurrentPosition() - lastRightEncPos;
        int horiEncChange = horizontalEnc.getCurrentPosition() - lastHoriEncPos;

        // does not call getCurrentPosition a second time because you would not account for encoder
        // readings from the time between the two calls
        lastLeftEncPos += leftEncChange;
        lastRightEncPos += rightEncChange;
        lastHoriEncPos += horiEncChange;

        // updates angle
        imu.update();
        double changeInAngle = imu.angle - angle;
        angleVel = changeInAngle / timeElapsed;

        // currently in robot centric
        double[] changeInPos = new double[]{
                encToInch(horiEncChange - horiEncPerDegree * changeInAngle),
                encToInch((leftEncChange + rightEncChange) / 2)
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

    public void update() {
        double timeElapsed = updateTimer.seconds();
        update(timeElapsed);
    }

    public void savePosition() {
        File file = AppUtil.getInstance().getSettingsFile("RobotPosition.txt");
        String contents = xPos + ":" + yPos + ":" + angle;
        ReadWriteFile.writeFile(file, contents);
    }

    public void loadPosition() {
        File file = AppUtil.getInstance().getSettingsFile("RobotPosition.txt");
        String[] contents = ReadWriteFile.readFile(file).trim().split(":");
        xPos = Double.parseDouble(contents[0]);
        yPos = Double.parseDouble(contents[1]);
        angle = Double.parseDouble(contents[2]);
    }

    public void setPosition(double x, double y, double angle){
        this.xPos = x;
        this.yPos = y;
        this.angle = angle;
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
