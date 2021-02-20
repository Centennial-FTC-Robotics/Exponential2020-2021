package org.exponential.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.exponential.mechanisms.DriveTrainParametric;
import org.exponential.mechanisms.parametricEQ.CubicSpline;
import org.exponential.mechanisms.parametricEQ.State;
import org.exponential.superclasses.ExpoOpMode;

import java.util.ArrayList;

@Autonomous
public class SplineTest extends ExpoOpMode {

    public void run() throws InterruptedException {
        double time = 0;
        ArrayList<CubicSpline.CubicSplinePoint> spline = new ArrayList<CubicSpline.CubicSplinePoint>();
        State state = new State();
        state.fieldX = 0;
        state.fieldY = 0;
        state.angle = 0;
        state.velX = 0;
        state.velY = 0;
        state.angleVel = 0;


        spline.add(new CubicSpline.CubicSplinePoint(state,0));
        for(int i=0; i<10; i++){
            time+=2;
            state = new State();
            state.fieldX = 48;
            state.fieldY = 24;
            state.angle = 70;
            state.velX = 2;
            state.velY = 20;
            state.angleVel = 5;
            spline.add(new CubicSpline.CubicSplinePoint(state, time));

            time+=2;
            state = new State();
            state.fieldX = 60;
            state.fieldY = 96;
            state.angle = 180;
            state.velX = -10;
            state.velY = 0;
            state.angleVel = 5;
            spline.add(new CubicSpline.CubicSplinePoint(state, time));

            time+=2;
            state = new State();
            state.fieldX = 24;
            state.fieldY = 96;
            state.angle = 180;
            state.velX = -10;
            state.velY = -10;
            state.angleVel = 5;
            spline.add(new CubicSpline.CubicSplinePoint(state, time));

            time+=4;
            state = new State();
            state.fieldX = 0;
            state.fieldY = 0;
            state.angle = 0;
            state.velX = 0;
            state.velY = 0;
            state.angleVel = 0;

            spline.add(new CubicSpline.CubicSplinePoint(state, time));
        }
        ((DriveTrainParametric)(expo.drivetrain)).moveAlongParametricEq(new CubicSpline(spline));


    }
}
