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
    double xGoal = 236.22;
    double compensationTurning;
    int targetNumber = 0;
    double [] goalYCoords = {85.3, 98.42, 111.54};


    public void initialize(LinearOpMode opMode) {
    }

    public Turret (Odometry odometry, LinearOpMode opMode){
        positioning = odometry;
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // 59 in x, 236.22 in y.
    }
    public void moveTurret(int toggleInput, int previousTarget){
        positioning.update();
        turretMotor.setTargetPosition((int) Math.round(turretDegreeToEncoder(limitTurretAngle(IMU.normalize(findingTurretAngle(toggleGoal(toggleInput, previousTarget)))))));//TODO SET Motor to go to this position

    }
    //Motor Degrees to Encoders
    public double turretDegreeToEncoder (double degToEnc){
        degToEnc = degToEnc * (537.6 * 2)/360;//TODO find this conversion rate
        return degToEnc;
    }

    public double turretEncoderToDegree (double encToDeg){
        encToDeg = encToDeg * 360/(537.6 * 2);//whatever the number is FIND THIS OUT LATER
        return encToDeg;
    }

    //Finds the angle for the turret to shoot at in relation to the rest of the robot
    public double findingTurretAngle (double targetY){
        atanAngle = IMU.normalize(Math.toDegrees(Math.atan2(targetY- positioning.getyPos(), xGoal - positioning.getxPos())));
        double robotAngle = IMU.normalize(positioning.getAngle());
        turretAngle = atanAngle - robotAngle + shooterInaccuracy;
        return turretAngle;
    }

    public double limitTurretAngle (double unlimitedAngle){
        //angles are in the perspective of the motor. ccw is 160 deg | cw is 130 deg
        if (unlimitedAngle > 160) {
            compensationTurning = unlimitedAngle - 160;
            unlimitedAngle = 160;
        } else if (unlimitedAngle < -130){
            compensationTurning = unlimitedAngle + 130;
            unlimitedAngle = -130;
        }
        //TODO need to add rotate method here to actually compensate for the angle.
        return unlimitedAngle;
    }

    public double toggleGoal(int toggle, int targetNumber){
        double targetYCoords;

        //Toggling
        if (toggle == 1 && targetNumber < 4) {
            targetNumber = targetNumber + 1;
        } else if (toggle == 1 & targetNumber == 5) {
            targetNumber = 0;
        }

        targetYCoords = goalYCoords [targetNumber];
        return targetYCoords;
    }
}
