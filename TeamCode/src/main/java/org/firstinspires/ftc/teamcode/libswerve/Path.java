package org.firstinspires.ftc.teamcode.libswerve;

import java.util.LinkedList;

public class Path {
    protected LinkedList<Point> points = new LinkedList<>();
    private double threshold;

    /**
     * Create a new spline between two poses <b>(splines heading)</b>
     * @param threshold Threshold in inches
     */
    public Path(Pose2d p1, Pose2d p2, double threshold) {
        this.threshold = threshold;


    }
}