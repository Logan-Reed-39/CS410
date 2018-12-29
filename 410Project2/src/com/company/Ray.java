package com.company;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Ray {
    private Vector3D p;
    private Vector3D v;
    private double bestT = 9999;
    private Sphere bestSphere;
    private Model bestModel;
    private Vector3D bestPoint;
    private String bestFace;
    private Vector3D bestModelPoint;
    private double beta = 0.0;
    private double gamma = 0.0;

    public void setBeta(double b) { this.beta = b; }
    public double getBeta(){ return beta;}

    public void setGamma(double b) { this.gamma = b; }
    public double getGamma(){ return gamma;}

    public double getBestT(){return bestT;}
    public void setBestT(double e){this.bestT = e; }
    public Sphere getBestSphere(){return bestSphere;}
    public void setBestSphere(Sphere e){this.bestSphere = e;}
    public Model getBestModel(){return bestModel;}
    public void setBestModel(Model e){this.bestModel = e;}
    public Vector3D getBestPoint(){return this.bestPoint;}
    public void setBestPoint(Vector3D e){ this.bestPoint = e;}
    public String getBestFace(){return this.bestFace;}
    public void setBestFace(String e){ this.bestFace = e;}
    public Vector3D getBestModelPoint(){return this.bestModelPoint;}
    public void setBestModelPoint(Vector3D e){ this.bestModelPoint = e;}



    public Vector3D getP(){ return p; }
    public void setP(Vector3D o){ this.p = o;}

    public Vector3D getV(){ return v;}
    public void setV(Vector3D v){ this.v = v;}

    public Ray(Vector3D p, Vector3D v)
    {
        this.bestPoint = new Vector3D(0,0,0);
        this.p = p;
        this.v = v;
    }
    public String toString()
    {
        return("Point: " + this.p + "\n Vector: " + this.v);
    }
}
