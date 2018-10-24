package com.company;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Triangle {
    private Vector3D A;
    private Vector3D B;
    private Vector3D C;

    public Triangle(Vector3D A, Vector3D B, Vector3D C)
    {
        this.A = A;
        this.B = B;
        this.C = C;
    }
}
