package com.company;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Ray {
    private Vector3D p;
    private Vector3D v;

    public Vector3D getP(){ return p; }
    public void setP(Vector3D o){ this.p = o;}

    public Vector3D getV(){ return v;}
    public void setV(Vector3D v){ this.v = v;}

    public Ray(Vector3D p, Vector3D v)
    {
        this.p = p;
        this.v = v;
    }
    public String toString()
    {
        return("Point: " + this.p + "\n Vector: " + this.v);
    }
}
