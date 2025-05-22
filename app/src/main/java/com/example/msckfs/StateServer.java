package com.example.msckfs;

import com.example.msckfs.imuProcessing.ImuState;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.Map;

public class StateServer {
    //TODO: should anything in the state server have volatile keyword?
    public RealMatrix stateCov, continuousNoiseCov;
    public RealVector gravity;
    public ImuState imuState;
    public Map<> camStates;

    public StateServer() {
        //TODO: put in MSCKF class? bc. why hardcode matrix size but not content?
        stateCov = new Array2DRowRealMatrix(21,21);
        continuousNoiseCov = new Array2DRowRealMatrix(12,12);

    }

}
