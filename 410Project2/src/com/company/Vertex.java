package com.company;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import java.util.ArrayList;

public class Vertex {
    private ArrayList<Integer> faces = new ArrayList<>();
    private Vector3D point;
    private int index;

    public Vector3D getPoint() { return point; }

    public void setPoint(Vector3D point) { this.point = point; }

    public int getIndex() { return index; }

    public void setIndex(int index) { this.index = index; }

    public ArrayList<Integer> getFaces() { return faces; }

    public void setFaces(ArrayList<Integer> faces) { this.faces = faces; }

    public void addToFaces(int faceCount) { faces.add(faceCount);}

    public Vertex(Vector3D point, int count)
    {
        this.point = point;
        this.index = count;
    }
    public Vertex(Vector3D point)
    {
        this.point = point;
    }
}
