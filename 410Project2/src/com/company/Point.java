package com.company;

public class Point {
    private double A;
    private double B;
    private double C;

    public double getA() { return A; }
    public void setA(double a) { A = a; }
    public double getB() { return B; }
    public void setB(double b) { B = b; }
    public double getC() { return C; }
    public void setC(double c) { C = c; }

    public Point(double A, double B, double C){
        this.A = A;
        this.B = B;
        this.C = C;

    }

    public String toString(){
        return "( " + this.getA() + ", " + this.getB() + ", " + this.getC() + ")";
    }
}
