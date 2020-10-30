package org.exponential.utility;

import java.util.ArrayList;

public class Path {
    ArrayList<Coordinate> coordinates;
    public Path(ArrayList<Coordinate> coordinates) {
        this.coordinates = coordinates;
    }

    public ArrayList<Coordinate> getCoordinates() {
        return coordinates;
    }
    public ArrayList<Line> getLines() {
        ArrayList<Line> lines = new ArrayList<Line>();
        for (int i = 0; i < coordinates.size() - 1; i++) {
            lines.add(new Line(coordinates.get(i), coordinates.get(i + 1)));
        }
        return lines;
    }
}
