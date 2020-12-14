package org.exponential.mechanisms.parametricEQ;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class CubicSpline extends ParametricEq {
    static class CubicSplinePoint {
        State state;
        double time;

        CubicSplinePoint(State state, double time) {
            this.state = state;
            this.time = time;
        }
    }

    static class Cubic /* polynomial */ {
        double a;
        double b;
        double c;
        double d;

        Cubic(double a, double b, double c, double d) {
            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
        }

        Cubic(double pos1, double vel1, double time1, double pos2, double vel2, double time2) {
            // after deriving the formula using 4 variable linear system
            a = (vel1 + vel2) / Math.pow((time1 - time2), 2) - 2 * (pos1 - pos2) / Math.pow((time1 - time2), 3);
            b = 1 / 2.0 * ((vel1 - vel2) / (time1 - time2) - 3 * (time1 + time2) * a);
            c = vel1 - 3 * time1 * time1 * a - 2 * time1 * b;
            d = pos1 - Math.pow(time1, 3) * a - Math.pow(time1, 2) * b - time1 * c;
        }

        public double valueAt(double sec) {
            return a * sec * sec * sec + b * sec * sec + c * sec + d;
        }

        public double velAt(double sec){
            return 3*a*sec*sec+2*b*sec+c;
        }
    }

    static class SplineSegment {
        Cubic x;
        Cubic y;
        Cubic angle;
        double startTime;
        double endTime;

        SplineSegment(CubicSplinePoint start, CubicSplinePoint end) {
            startTime = start.time;
            endTime = end.time;
            x = new Cubic(start.state.fieldX, start.state.velX, start.time, end.state.fieldX, end.state.velX, end.time);
            y = new Cubic(start.state.fieldY, start.state.velY, start.time, end.state.fieldY, end.state.velY, end.time);
            angle = new Cubic(start.state.angle, start.state.angleVel, start.time, end.state.angle, end.state.angleVel, end.time);
        }

        public State getStateAtTime(double t){
            State theState = new State();

            theState.fieldX=x.valueAt(t);
            theState.fieldY=y.valueAt(t);
            theState.angle=angle.valueAt(t);
            theState.velX=x.velAt(t);
            theState.velY=y.velAt(t);
            theState.angleVel=angle.velAt(t);
            return theState;

        }

    }

    LinkedList<SplineSegment> segments = new LinkedList<SplineSegment>();
    CubicSplinePoint end;
    SplineSegment current;
    boolean endedPath = false;

    CubicSpline(ArrayList<CubicSplinePoint> states) {
        // assumes the points are ordered from most recent to latest
        for(int i=0; i<states.size()-1; i++){
            segments.push(new SplineSegment(states.get(i), states.get(i+1)));
        }
        current = segments.pop();
        end = states.get(states.size()-1);
    }

    @Override
    public State getStateAtTime(double t) {
        if(t<=end.time){
            while(current.endTime<t){
                current=segments.pop();
            }
            return current.getStateAtTime(t);
        } else {
            endedPath=true;
            return end.state;
        }
    }

    @Override
    public boolean moveOn(double currentX, double currentY, double currentAngle, double velX, double velY, double velAngle) {
        if(endedPath){
            return Math.sqrt(Math.pow(end.state.fieldX - currentX, 2) + Math.pow(end.state.fieldY - currentY, 2)) < 0.5;
        }
        return false;
    }
}
