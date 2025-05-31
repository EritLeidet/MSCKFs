package com.example.msckfswin;
import com.example.msckfswin.imageProcessing.FeatureMessage;
import com.example.msckfswin.imageProcessing.FeatureMetaData;
import com.example.msckfswin.imageProcessing.ImageMessage;
import com.example.msckfswin.imageProcessing.ImageProcessor;
import com.example.msckfswin.imageProcessing.ProcessorConfig;
import com.example.msckfswin.subPub.CallbackSubscriber;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.SubmissionPublisher;
import java.util.function.Consumer;

/**
 * Instantiator for VIO algorithm
 */
public class Vio {

    //TODO: where determine the buffer size?


    public Vio() {

    }

    public static void tempFeatureCallback(FeatureMessage featureMessage) {
        System.out.print(featureMessage.features);

    }

    public static void main(String[] args) {
        System.loadLibrary( Core.NATIVE_LIBRARY_NAME );

        ProcessorConfig config = ProcessorConfig.getEuRoCconfig();

        // Msckf filter = new Msckf();

        // initialize publishers
        SubmissionPublisher<FeatureMessage> featurePublisher = new SubmissionPublisher<>();
        SubmissionPublisher<ImageMessage> imagePublisher = new SubmissionPublisher<>();

        // initialize subscribers
        Consumer<FeatureMessage> featureCallback = Vio::tempFeatureCallback;
        CallbackSubscriber<FeatureMessage> featureSubscriber = new CallbackSubscriber<>(featureCallback);

        // subscribe subscribers to publishers
        featurePublisher.subscribe(featureSubscriber);

        ImageProcessor imageProcessor = new ImageProcessor(config, featurePublisher);

        // TODO: parse EuRoC ground truth



    }
}
