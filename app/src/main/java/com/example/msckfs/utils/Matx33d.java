package com.example.msckfs.utils;

import static org.opencv.core.CvType.CV_32F;
import static org.opencv.core.CvType.CV_64F;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;

public class Matx33d extends Mat {

    public Matx33d() {
        super(3,3,CV_64F, new Scalar(0));
    }

    public Matx33d(double[] data) {
        super(); // TODO: nötig?
        if (data.length != 9) throw new IllegalArgumentException("For Matx33f initialization data array size must be 9.");
        this.put(0,0, data);
    }
}
