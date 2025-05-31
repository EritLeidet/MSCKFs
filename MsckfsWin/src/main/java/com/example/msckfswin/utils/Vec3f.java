package com.example.msckfswin.utils;

import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;

public abstract class Vec3f {


    private static final int vecLength = 3;
    public static MatOfFloat create() {
        return new MatOfFloat(0f,0f,0f);
    }

    public static MatOfFloat create(float... data) {
        return Vecf.create(vecLength, data);
    }

    public static boolean isVec3f(Mat vec) {
        return Vecf.isVecf(vecLength, vec);
    }
}