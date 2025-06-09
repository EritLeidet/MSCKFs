package com.example.msckfswin;

import static org.opencv.core.CvType.CV_64F;

import com.example.msckfswin.imuProcessing.ImuState;


import org.apache.commons.math3.linear.RealMatrix;
import org.opencv.core.Mat;

import java.util.Map;

public class StateServer {


    public  ImuState imuState;

    public Map<Integer,CamState> camStates;

    // State covariance matrix
    public RealMatrix stateCov; // TODO:  Mat(21,21,CV_64F);
    public RealMatrix continuousNoiseCov; // TODO:  Mat(12,12,CV_64F);



    /*
    //TODO: should anything in the state server have volatile keyword?
    public RealMatrix stateCov, continuousNoiseCov;
    public RealVector gravity;


    public StateServer() {
        //TODO: put in MSCKF class? bc. why hardcode matrix size but not content?
        stateCov = new Array2DRowRealMatrix(21,21);
        continuousNoiseCov = new Array2DRowRealMatrix(12,12);

    }

     */

}
