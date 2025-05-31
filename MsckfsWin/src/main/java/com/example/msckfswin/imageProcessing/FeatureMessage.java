package com.example.msckfswin.imageProcessing;

import java.util.List;

public class FeatureMessage {

    public final long timestamp; // unix time
    public final List<FeatureMeasurement> features;

    public FeatureMessage(long timestamp, List<FeatureMeasurement> features) {
        this.features = features;
        this.timestamp = timestamp;

    }
}
