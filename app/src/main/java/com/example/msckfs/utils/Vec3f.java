package com.example.msckfs.utils;

import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;

public class Vec3f extends MatOfFloat {

    public Vec3f() {
        super(0f,0f,0f);
    }

    public Vec3f(float x, float y, float z) {
        super(x,y,z);
    }

    public float get(int index) {
        if (index > 2 || index < 0 ) throw new IllegalArgumentException("Vec3f access index must be between 0 and 2");
        return super.at(float.class, index, 0).getV();
    }
}
