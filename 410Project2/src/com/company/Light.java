package com.company;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Light {

    private Vector3D lightCoordinates;
    private Vector3D lightColor;
    private Vector3D lightVector;
    private Vector3D locationL;
    private boolean W1;

    public boolean isW1() {
        return W1;
    }

    public void setW1(boolean w1) {
        W1 = w1;
    }

    public Vector3D getLightCoordinates() { return this.lightCoordinates;}
    public void setLightCoordinates(Vector3D e) { this.lightCoordinates = e;}

    public Vector3D getLightColor() { return this.lightColor;}
    public void setLightColor(Vector3D e) { this.lightColor = e;}

    public Vector3D getLightVector() { return this.lightVector;}
    public void setLightVector(Vector3D e) { this.lightVector = e;}

    public Vector3D getLocationL() { return this.locationL;}
    public void setLocationL(Vector3D e) { this.locationL = e;}

    public Light()
    {

    }
    public String toString()
    {
        String s = "Light coordinates: \n";
        for (double d: locationL.toArray())
        {
            s += d + " ";
        }
        s += "\n Light Colors: \n";
        for(double d: lightColor.toArray())
        {
            s+= d + " ";
        }
        return s;
    }


}
