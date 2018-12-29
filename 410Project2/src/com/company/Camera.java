package com.company;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Camera {
    private Vector3D EV;
    private Vector3D LV;
    private Vector3D UP;
    private Vector3D UV;
    private Vector3D WV;


    public Vector3D getWV() {
        return WV;
    }

    public void setWV(Vector3D WV) {
        this.WV = WV;
    }

    private Vector3D VV;

    public Vector3D getEV() {
        return EV;
    }

    public void setEV(Vector3D EV) {
        this.EV = EV;
    }

    public Vector3D getLV() {
        return LV;
    }

    public void setLV(Vector3D LV) {
        this.LV = LV;
    }

    public Vector3D getUP() {
        return UP;
    }

    public void setUP(Vector3D UP) {
        this.UP = UP;
    }

    public Vector3D getUV() {
        return UV;
    }

    public void setUV(Vector3D UV) {
        this.UV = UV;
    }

    public Vector3D getVV() {
        return VV;
    }

    public void setVV(Vector3D VV) {
        this.VV = VV;
    }

}
