package com.example.msckfswin.imageProcessing;

import org.opencv.core.Point;

public class FeatureMetaData {

    public int id;
    public float response;
    public int lifetime;
    public final Point cam0_point;
    public FeatureMetaData( Point cam0Point) {
        this.cam0_point = cam0Point;

        // TODO: do these init vals matter ?
        this.id = -1;
        this.lifetime = -1;
    }

    @Override
    public String toString() {
        return "FeatureMetaData{" +
                "id=" + id +
                ", response=" + response +
                ", lifetime=" + lifetime +
                ", cam0_point=" + cam0_point +
                '}';
    }
}
