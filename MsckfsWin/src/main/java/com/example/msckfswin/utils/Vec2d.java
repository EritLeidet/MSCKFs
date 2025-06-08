package com.example.msckfswin.utils;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

public abstract class Vec2d {

    private static final int vecLength = 2;
    public static MatOfDouble create() {
        return new MatOfDouble(0d,0d);
    }

    public static MatOfDouble create(double... data) {
        return Vecd.create(vecLength, data);
    }

    public static boolean isVec2d(Mat vec) {
        return Vecd.isVecd(vecLength, vec);
    }
}
