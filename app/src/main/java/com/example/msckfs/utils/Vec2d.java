package com.example.msckfs.utils;

import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;

public class Vec2d extends Vecd {
    private static final int vecLength = 2;
    public Vec2d() {
        super(vecLength);
    }

    public Vec2d(double x, double y) {
        super(vecLength, x,y);
    }

    public Vec2d(Mat mat) {
        super(vecLength, mat);
    }
}
