package com.example.msckfs.utils;

import static org.opencv.core.CvType.CV_64F;

import org.opencv.core.Mat;

public abstract class Matx {

    /**
     *
     * @param numRows
     * @param numCols
     * @param type
     * @param data
     * @return
     */
    public static Mat create(int numRows, int numCols, int type, double[] data) {
        if (data.length != numRows * numCols) throw new IllegalArgumentException("Data array has incorrect number of elements.");
        Mat mat = new Mat(numRows,numCols,type);
        mat.put(0,0,data);
        return mat;
    }


    public static boolean isMatx( Mat mat, int numRows, int numCols, int type) {
        return mat.channels() == 1 && mat.rows() == numRows && mat.cols() == numCols && mat.type() == type;
    }


}
