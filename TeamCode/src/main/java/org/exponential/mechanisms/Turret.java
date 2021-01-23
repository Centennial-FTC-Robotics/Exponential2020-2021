package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.exponential.superclasses.Mechanism;

public class Turret implements Mechanism {
    DcMotorEx turretMotor;
    Odometry positioning;
    double turretAngle;
    double shooterInaccuracy = 10;
    double atanAngle;
    double yGoal = 236.22;


    public void initialize(LinearOpMode opMode) {
    }

    public Turret (Odometry odometry, LinearOpMode opMode){
        positioning = odometry;
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // 59 in x, 236.22 in y.
    }
    public void moveTurret(double targetPosition){

        positioning.update();
        turretDegreeToEncoder(IMU.normalize(findingTurretAngle(targetPosition)));//TODO SET Motor to go to this position

    }
    //Motor Degrees to Encoders
    public double turretDegreeToEncoder (double degToEnc){
        degToEnc = degToEnc * 1;//TODO find this conversion rate
        return degToEnc;
    }

    public double turretEncoderToDegree (double encToDeg){
        encToDeg = encToDeg * 1;//whatever the number is FIND THIS OUT LATER
        return encToDeg;
    }

    //Finds the angle for the turret to shoot at in relation to the rest of the robot
    public double findingTurretAngle (double targetX){
        atanAngle = IMU.normalize(Math.toDegrees(Math.atan2(yGoal - positioning.getyPos(), targetX - positioning.getxPos())));
        double robotAngle = IMU.normalize(positioning.getAngle());
        turretAngle = atanAngle - robotAngle + shooterInaccuracy;
        return turretAngle;
    }
}
