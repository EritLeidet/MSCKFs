package com.example.msckfs.utils;

import static org.opencv.core.CvType.CV_32F;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;

public class Matx33f extends Mat {
    public Matx33f() {
        super(3,3,CV_32F, new Scalar(0));
    }

    public Matx33f(float[] data) {
        super(); // TODO: nötig?
        if (data.length != 9) throw new IllegalArgumentException("For Matx33f initialization data array size must be 9.");
        this.put(0,0, data);
    }
}
