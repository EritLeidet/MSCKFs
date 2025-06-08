package com.example.msckfswin.utils;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * 3d rigid transform.
 */
public class Isometry3d {
    public final RealMatrix R;
    public final RealVector t;
    public Isometry3d(RealMatrix r, RealMatrix t) {
        assert(r.getRowDimension() == r.getColumnDimension() && r.getRowDimension() == 3); // TODO:assumptions
        assert(t.getColumnDimension() == 1 && t.getRowDimension() == 3);

        this.R = r;
        this.t = t;
    }

    public RealMatrix matrix() {
        RealMatrix m = MatrixUtils.createRealIdentityMatrix(4);
        m.setSubMatrix(this.R.getData(), 0,0);
        m.setSubMatrix(this.t.getData(), 0,3);
        return m;
    }

    public Isometry3d inverse() {
        return new Isometry3d(this.R.transpose(), this.R.transpose().scalarMultiply(-1).multiply(this.t));
    }

    public Isometry3d mul(Isometry3d T1) {
        RealMatrix newR = this.R.multiply(T1.R);
        RealMatrix newT = this.R.multiply(T1.t).add(this.t);
        return new Isometry3d(R,t);
    }
}
