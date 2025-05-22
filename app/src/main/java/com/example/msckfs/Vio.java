package com.example.msckfs;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Flow;
import java.util.concurrent.SubmissionPublisher;
import java.util.function.Consumer;

/**
 * Instantiator for VIO algorithm
 */
public class Vio {

    //TODO: where determine the buffer size?

    static List<Integer> list;

    public Vio() {
        list = new ArrayList<>();
    }
    public static void main(String[] args) {
        Mat mat = new MatOfFloat(1,2,3);
        System.out.println(mat);


        /**
        SubmissionPublisher<Integer> pub = new SubmissionPublisher<Integer>();

        Consumer<Mat> ImageConsumer = new Consumer<Mat>() {
            @Override
            public void accept(Mat mat) {

            }
        };
         **/


    }
}
