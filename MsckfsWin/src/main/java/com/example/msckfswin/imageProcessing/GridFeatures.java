package com.example.msckfswin.imageProcessing;

import com.example.msckfswin.config.ProcessorConfig;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class GridFeatures extends HashMap<Integer, List<FeatureMetaData>> {

    // TODO: can I just put in constructor?
    public void init(ProcessorConfig config) {
        for (int code = 0; code < config.gridRow * config.gridCol; code++) {
            this.put(code, new ArrayList<>());
        }
    }
}
