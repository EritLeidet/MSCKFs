package com.example.msckfswin.utils;

import org.opencv.core.Mat;

public abstract class Matx {

    public static Mat create(int numRows, int numCols, int type, double[] data) {
        if (data.length != numRows * numCols) throw new IllegalArgumentException("Data array has incorrect number of elements.");
        Mat mat = new Mat(numRows,numCols,type);
        mat.put(0,0,data);
        return mat;
    }


    public static boolean isMatx( Mat mat, int numRows, int numCols, int type) {
        return mat.channels() == 1 && mat.rows() == numRows && mat.cols() == numCols && mat.type() == type;
    }

    public static double getD(Mat mat, int row, int col) {
        return mat.at(double.class, row, col).getV();
    }

    public static void setD(Mat mat, int row, int col, double val) {
        mat.at(double.class, row, col).setV(val);
    }


}
