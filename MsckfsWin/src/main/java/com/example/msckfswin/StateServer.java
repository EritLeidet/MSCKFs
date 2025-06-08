package com.example.msckfswin;

import static org.opencv.core.CvType.CV_64F;

import com.example.msckfswin.imuProcessing.ImuState;


import org.opencv.core.Mat;

import java.util.Map;

public class StateServer {


    public  ImuState imuState;

    public Map<Integer,CamState> camStates;

    // State covariance matrix
    Mat stateCov = new Mat(21,21,CV_64F);
    Mat continuousNoiseCov = new Mat(12,12,CV_64F);



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
