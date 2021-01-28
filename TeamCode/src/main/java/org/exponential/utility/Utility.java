package org.exponential.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Utility {
    public static void sleep(int ms) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < ms);
    }
}
