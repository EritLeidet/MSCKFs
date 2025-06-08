package com.example.msckfswin.utils;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

// TODO
public abstract class Vec4d {

    private static final int vecLength = 4;
    public static MatOfDouble create() {
        return new MatOfDouble(0d,0d,0d,0d);
    }

    public static MatOfDouble create(double... data) {
        return Vecd.create(vecLength, data);
    }

    public static boolean isVec4d(Mat vec) {
        return Vecd.isVecd(vecLength, vec);
    }
}
