package com.example.msckfswin.utils;

import static org.opencv.core.CvType.CV_32F;
import static org.opencv.core.CvType.CV_64F;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfFloat;

public abstract class Vecf extends MatOfFloat {
    public static MatOfFloat create(int vecLength, float... data) {
        if (data.length != vecLength) throw new IllegalArgumentException("Data array has incorrect number of elements.");
        return new MatOfFloat(data);
    }

    public static boolean isVecf(int vecLength, Mat vec) {
        return vec.cols() == 1 && vec.rows() == vecLength && vec.channels() == 1 && vec.type() == CV_32F;
    }


    public static double get(Mat vec, int index) {
        return vec.at(float.class, index, 0).getV();
    }


}
