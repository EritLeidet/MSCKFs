package com.example.msckfs.imageProcessing;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.opencv.core.Point;

public class FeatureMetaData {

    public int id;
    public final float response;
    public int lifetime;
    public final Point cam0_point;
    public FeatureMetaData(float response, Point cam0Point) {
        this.response = response;
        this.cam0_point = cam0Point;

        // TODO: do these init vals matter ?
        this.id = -1;
        this.lifetime = -1;
    }

    public void setId(int id) {this.id = id;}

    public void setLifetime(int lifetime) {this.lifetime = lifetime;}


}
