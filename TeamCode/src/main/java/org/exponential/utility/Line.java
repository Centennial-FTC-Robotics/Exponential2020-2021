package org.exponential.utility;

public class Line {
    private Coordinate endpoint1;
    private Coordinate endpoint2;
    private double x0;
    private double x1;
    private double y0;
    private double y1;
    private double t1;
    private double t2;

    public Line(Coordinate coord1, Coordinate coord2) {
        endpoint1 = coord1;
        endpoint2 = coord2;
        x0 = coord1.x;
        y0 = coord1.y;
        x1 = coord2.x;
        y1 = coord2.y;
    }

    // for determining whether or not a line intersects a circle
    public boolean intersectsCircle(double h, double k, double r) {
        double a = Math.pow((x1 - x0), 2) + Math.pow((y1 - y0), 2);
        double b = 2 * ((x0 - h) * (x1 - x0) + (y0 - k) * (y1 - y0));
        double c = Math.pow(x0, 2) - 2*h*x0 + Math.pow(h, 2) + Math.pow(y0, 2) - 2*k*y0 + Math.pow(k, 2) - Math.pow(r, 2);
        double discrim = b*b - 4*a*c;
        if (discrim <= 0) {
            return false;
        } else {
            t1 = (-b - Math.sqrt(discrim)) / (2*a);
            t1 = Math.max(0, Math.min(1, t1)); //clamp [0, 1]
            t2 = (-b + Math.sqrt(discrim)) / (2*a);
            t2 = Math.max(0, Math.min(1, t1));
            return true;
        }
    }

    // for getting the intersection point when you know the intersection to exist
    public Coordinate[] intersectionPoints() {
        return new Coordinate[] {new Coordinate(x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1),
                new Coordinate(x0 + (x1 - x0) * t2, y0 + (y1 - y0) * t2)};
    }
}
