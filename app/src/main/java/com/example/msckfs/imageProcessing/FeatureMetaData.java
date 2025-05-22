package com.example.msckfs.imageProcessing;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.opencv.core.Point;

public class FeatureMetaData {

    public final int id; // TODO: here 'int' like in Python, but FeatureIDType (unsigned long long) in C++
                        // TODO: document, what code from Python and what from C++
    public final float response;
    public final int lifetime;

    public final Point cam0_point;
    public FeatureMetaData() {
        ArrayRealVector
    }
}
