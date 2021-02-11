package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.exponential.superclasses.Mechanism;

import static org.exponential.utility.Utility.sleep;

public class Loader implements Mechanism {
    public static final double LOAD_POSITION = 0;
    public static final double UNLOAD_POSITION = .55;
    Servo loaderServo;
    int shot = 0;
    ElapsedTime shooterTimer = new ElapsedTime();


    @Override
    public void initialize(LinearOpMode opMode) {
        loaderServo = opMode.hardwareMap.servo.get("loaderServo");
        loaderServo.setPosition(UNLOAD_POSITION);
    }

    public void load() {
        loaderServo.setPosition(LOAD_POSITION);
    }

    public void unload() {
        loaderServo.setPosition(UNLOAD_POSITION);
    }

    public void loadAndUnloadTwo() {

        /*for (shot = 0; shot < 3; shot++) {
            shooterTimer.reset();
            if (shooterTimer.milliseconds() > 250) {
                load();
            } else if (shooterTimer.milliseconds() > 500) {
                unload();
            }
        }
        shot = 0;*/

        for (int i = 0; i < 2; i++) {
            load();
            sleep(80);
            unload();
            sleep(250);
        }
    }
        public void loadAndUnload() {
            load();
            sleep(80);
            unload();
            sleep(250);

        }
}
