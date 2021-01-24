package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.exponential.superclasses.Mechanism;

public class Turret implements Mechanism {
    DcMotorEx turretMotor;
    Drivetrain drivetrain;
    double turretAngle;
    double shooterOffset = 10;
    double atanAngle;
    //double xGoal = 236.22;
    double targetYPosition = 72;
    int targetNumber = 0;
    //double [] goalYCoords = {85.3, 98.42, 111.54};

    double lastAngle = 0;

    public void initialize(LinearOpMode opMode) {
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(.2);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // 59 in x, 236.22 in y.
    }

    public Turret (Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    public void rotateTurretToTarget(int targetXValue){
        drivetrain.positioning.update();

        double angleToRotate = IMU.normalize(getAngleToRotate(targetXValue));
        double normalizedAngle = IMU.normalize(angleToRotate);
        //double clippedAngle = limitTurretAngle(normalizedAngle);
        double clippedAngleToRotate = Range.clip(angleToRotate, -130 - lastAngle, 160 - lastAngle);
        double angleToRotateDrivetrain = angleToRotate - clippedAngleToRotate;
        int encoderToRotate = turretDegreeToEncoder(clippedAngleToRotate);
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition() + encoderToRotate);//TODO SET Motor to go to this position
        lastAngle = lastAngle + clippedAngleToRotate;

    }
    //Motor Degrees to Encoders
    public int turretDegreeToEncoder (double degToEnc){
        degToEnc = degToEnc * (537.6 * 2)/360;//TODO find this conversion rate
        return (int) degToEnc;
    }

    public double turretEncoderToDegree (double encToDeg){
        encToDeg = encToDeg * 360/(537.6 * 2);//whatever the number is FIND THIS OUT LATER
        return encToDeg;
    }

    //Finds the number of degrees to rotate the turret
    public double getAngleToRotate(double targetXPosition){
        atanAngle = IMU.normalize(Math.toDegrees(Math.atan2(targetYPosition - drivetrain.positioning.getyPos(), targetXPosition - drivetrain.positioning.getxPos())));
        double robotAngle = IMU.normalize(drivetrain.positioning.getAngle());
        turretAngle = atanAngle - robotAngle + shooterOffset;
        return turretAngle;
    }

    /*public double limitTurretAngle (double unlimitedAngle){
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
    }*/

    /*public void setTargetXPosition(double x) {
        targetXPosition = x;
    }
*/
    /*public double toggleGoal(int targetNumber){
        double targetYCoords;

        //Toggling
        if (toggle == 1 && targetNumber < 4) {
            targetNumber = targetNumber + 1;
        } else if (toggle == 1 & targetNumber == 5) {
            targetNumber = 0;
        }

        targetYCoords = goalYCoords [targetNumber];
        return targetYCoords;
    }*/
}
