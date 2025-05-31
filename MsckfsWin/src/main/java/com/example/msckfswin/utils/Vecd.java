package com.example.msckfswin.utils;

import static org.opencv.core.CvType.CV_64F;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;


// TODO: should I refactor it to work like Matx33d?
public abstract class Vecd {



    public static MatOfDouble create(int vecLength, double... data) {
        if (data.length != vecLength) throw new IllegalArgumentException("Data array has incorrect number of elements.");
        return new MatOfDouble(data);
    }

    public static boolean isVecd(int vecLength, Mat vec) {
        return vec.cols() == 1 && vec.rows() == vecLength && vec.channels() == 1 && vec.type() == CV_64F;
    }


    public static double get(Mat vec, int index) {
        return vec.at(double.class, index, 0).getV();
    }

}
