package com.example.msckfswin.imageProcessing;

import org.opencv.core.Mat;

public class ImageMessage {

    public final long timestamp; // unix time
    public final Mat image;

    public ImageMessage(long timestamp, Mat image) {
        this.timestamp = timestamp;
        this.image = image;
    }
}
