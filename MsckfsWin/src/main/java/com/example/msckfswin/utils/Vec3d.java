package com.example.msckfswin.utils;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;


// TODO: maybe only use asserts, and remove all usages of create

public class Vec3d {

    private Vec3d() {}

    private static final int vecLength = 3;
    public static MatOfDouble create() {
        return new MatOfDouble(0d,0d,0d);
    }

    public static MatOfDouble create(double... data) {
        return Vecd.create(vecLength, data);
    }

    public static boolean isVec3d(Mat vec) {
        return Vecd.isVecd(vecLength, vec);
    }
}
