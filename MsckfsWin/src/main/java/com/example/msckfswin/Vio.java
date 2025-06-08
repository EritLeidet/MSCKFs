package com.example.msckfswin;
import static org.opencv.imgcodecs.Imgcodecs.imread;

import static java.nio.file.Files.readAllLines;

import com.example.msckfswin.imageProcessing.FeatureMessage;
import com.example.msckfswin.imageProcessing.ImageMessage;
import com.example.msckfswin.imageProcessing.ImageProcessor;
import com.example.msckfswin.config.ProcessorConfig;
import com.example.msckfswin.subPub.CallbackSubscriber;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.SubmissionPublisher;
import java.util.function.Consumer;
import java.util.stream.Collectors;

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

    public static void main(String[] args) throws IOException {
        System.loadLibrary( Core.NATIVE_LIBRARY_NAME );

        ProcessorConfig config = ProcessorConfig.getEuRoCconfig();

        // Msckf filter = new Msckf();

        // initialize publishers
        SubmissionPublisher<FeatureMessage> featurePublisher = new SubmissionPublisher<>();
        SubmissionPublisher<ImageMessage> imagePublisher = new SubmissionPublisher<>();

        // give publishers to VIO modules
        ImageProcessor imageProcessor = new ImageProcessor(config, featurePublisher);

        // initialize subscribers
        Consumer<FeatureMessage> featureCallback = Vio::tempFeatureCallback;
        Consumer<ImageMessage> imageCallback = imageProcessor::imageCallback;
        CallbackSubscriber<FeatureMessage> featureSubscriber = new CallbackSubscriber<>(featureCallback);
        CallbackSubscriber<ImageMessage> imageSubscriber = new CallbackSubscriber<>(imageCallback);

        // subscribe subscribers to publishers
        featurePublisher.subscribe(featureSubscriber);
        imagePublisher.subscribe(imageSubscriber);


        // publish in loop
        // TODO: parse EuRoC ground truth
        // TODO: https://stackoverflow.com/questions/50050640/how-load-all-imagens-from-directory-and-read-using-function-imread-opencv
        Mat img = Imgcodecs.imread("C:\\Users\\Jessica\\Downloads\\MH_01_easy\\mav0\\cam0\\data\\1403636579763555584.png");
        List<List<String>> imgMetaData = Files.readAllLines(Paths.get("C:\\Users\\Jessica\\Downloads\\MH_01_easy\\mav0\\cam0\\data.csv"))
                .stream()
                .map(line -> Arrays.asList(line.split(",")))
                .collect(Collectors.toList());
        ImageMessage msg = new ImageMessage(Long.parseLong(imgMetaData.get(1).get(0)), img);
        imagePublisher.submit(msg);

        // close publishers
        imagePublisher.close();
        // TODO: when to close the feature publishers?




    }
}
