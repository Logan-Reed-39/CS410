package com.company;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Sphere {
    private Vector3D centerCoordinates;
    private double sphereRadius = 0.0;
    private double[] ambientCoeff = new double[3];
    private double[] diffuseCoeff = new double[3];
    private double[] specularCoeff = new double[3];
    private Vector3D attenuationCoeff;
    private Vector3D opacityConstants;
    private double eta;
    private double phongConstant = 16;

    public Vector3D getCenterCoordinates() { return centerCoordinates; }
    public void setCenterCoordinates(Vector3D centerCoordinates) { this.centerCoordinates = centerCoordinates; }
    public double getSphereRadius() { return sphereRadius; }
    public void setSphereRadius(double sphereRadius) { this.sphereRadius = sphereRadius; }
    public double getEta() { return eta; }
    public void setEta(double eta) { this.eta = eta; }
    public double[] getAmbientCoeff() { return ambientCoeff; }
    public void setAmbientCoeff(double[] ambientCoeff) { this.ambientCoeff = ambientCoeff; }
    public double[] getDiffuseCoeff() { return diffuseCoeff; }
    public void setDiffuseCoeff(double[] diffuseCoeff) { this.diffuseCoeff = diffuseCoeff; }
    public double[] getSpecularCoeff() { return specularCoeff; }
    public void setSpecularCoeff(double[] specularCoeff) { this.specularCoeff = specularCoeff; }
    public Vector3D getAttenuationCoeff() { return attenuationCoeff; }
    public void setAttenuationCoeff(Vector3D attenuationCoeff) { this.attenuationCoeff = attenuationCoeff; }
    public Vector3D getOpacityConstants() { return opacityConstants; }
    public void setOpacityConstants(Vector3D opacityConstants) { this.opacityConstants = opacityConstants; }
    public double getPhongConstant() { return phongConstant; }
    public void setPhongConstant(double phongConstant) { this.phongConstant = phongConstant; }

    public Sphere(){

    }
    public Vector3D refract_tray(Vector3D W, Vector3D pt, Vector3D N, double eta1, double eta2)
    {
        double etar = eta1/ eta2;
        double a = -1 * etar;
        double wn = W.dotProduct(N);
        double radsq = Math.pow(etar,2) * (Math.pow(wn, 2) - 1) + 1;
        if (radsq < 0.0)
        {
            return null;
        }
        else
        {
            double b = (etar * wn) - Math.sqrt(radsq);
            Vector3D T = W.scalarMultiply(a).add(N.scalarMultiply(b));
            return T;
        }
    }

    public Ray refract_exit(Vector3D W, Vector3D pt, double eta_inside)
    {
        Vector3D T1 = refract_tray(W, pt, (pt.subtract(this.centerCoordinates)).normalize(), 1.0, eta_inside);
        if(T1.getX() + T1.getY() + T1.getZ() == 0.0)
        {
            return null;
        }
        else
        {
//            exit = pt + 2 * np.dot((self.C - pt),T1) * T1
            Vector3D exit = pt.add(T1.scalarMultiply(this.centerCoordinates.subtract(pt).dotProduct(T1)).scalarMultiply(2));
            Vector3D Nin = this.centerCoordinates.subtract(exit).normalize();
            Vector3D T2 = refract_tray(T1.scalarMultiply(-1), exit, Nin, eta_inside, 1.0);
            Ray refR = new Ray(exit, T2);
            return refR;
        }

    }
}
