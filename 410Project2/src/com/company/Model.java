package com.company;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.util.Precision;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;


public class Model {


    private static DecimalFormat df2 = new DecimalFormat(".##");
    private String identify;
    private Vector3D rotation;
    private double theta;
    private double scale;
    private double[] translation = new double[3];
    private Vector3D KaMaterial;
    private Vector3D KdMaterial;
    private String ModelObjName;
    private ArrayList<String> vertices = new ArrayList<>();
    private ArrayList<Vector3D> vertexNormals = new ArrayList<>();
    private ArrayList<String> faces = new ArrayList<>();
    private List<Double> points;
    private ArrayList<String> modelFileContents = new ArrayList<>();
    private RealMatrix pixels;
    private ArrayList<Vector3D> transformedPoints = new ArrayList<>();

    public ArrayList<Vector3D> getTransformedPoints() { return transformedPoints; }
    public void setTransformedPoints(ArrayList<Vector3D> transformedPoints) { this.transformedPoints = transformedPoints; }

    public String getIdentify(){ return identify;}
    public Vector3D getRotation() { return rotation; }
    public double getTheta() { return theta; }
    public double getScale() { return scale; }
    public double[] getTranslation() { return translation; }
    public Vector3D getKaMaterial() { return KaMaterial; }
    public Vector3D getKdMaterial() { return KdMaterial; }
    public String getModelObjName() { return ModelObjName; }

    public ArrayList<String> getFaces() { return faces;}
    public ArrayList<String> getModelFileContents() { return modelFileContents; }


    public void setRotation(Vector3D rotate1) { this.rotation = rotate1; }
    public void setTheta(double theta1) { this.theta = theta1; }
    public void setScale(double scale1) { this.scale = scale1; }
    public void setTranslation(double[] translation1) { this.translation = translation1; }
    public void setKaMaterial(Vector3D translation1) { this.KaMaterial = translation1; }
    public void setKdMaterial(Vector3D translation1) { this.KdMaterial= translation1; }
    public void setModelObjName(String modelObj1) { this.ModelObjName = modelObj1; }
    public void setVertices(ArrayList<String> e) { this.vertices = e; }




    public Model()
    {

    }

    public String toString()
    {
        String s = "Identify: " + this.getIdentify();
        s += "\nRotation: " + (this.getRotation());
        s += "\nTheta: " + this.getTheta();
        s += "\nScale: " + this.getScale();
        s += "\nTranslation: " + Arrays.toString(this.getTranslation());
        s += "\nModel Name: " + this.getModelObjName();
        s+= "\n";
        return s;
    }

    public ArrayList<Vector3D> getCoordinates()
    {
        int size = 0;
        File objFile = new File(ModelObjName);
        try {
            Scanner input = new Scanner(new FileReader(objFile));
            while (input.hasNext()) {
                String line = input.nextLine();
                if (line.startsWith("v ")) {
                    String line2 = line.substring(2);
                    vertices.add(line2);
                    size++;
                }
                else if (line.startsWith("f "))
                {
                    faces.add(line);
                    modelFileContents.add(line);
                }
                else if(line.startsWith("mtllib"))
                {
                    readMaterial(line);
                }
                else{
                    modelFileContents.add(line);
                }
            }
            input.close();
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
        this.setVertices(vertices);
        ArrayList<Vector3D> points = new ArrayList<Vector3D>();
        for(String s: vertices)
        {
            String[] line = s.split((" "));
            double d1 = Double.parseDouble(line[0]);
            double d2 = Double.parseDouble(line[1]);
            double d3 = Double.parseDouble(line[2]);
            Vector3D v = new Vector3D(d1, d2, d3);
            points.add(v);
        }

        return points;
    }
    public void readMaterial(String materialLine)
    {
        String[] parts = materialLine.split(" ");
        File materialFile = new File(parts[1]);
        try {
            Scanner input = new Scanner(materialFile);
            while(input.hasNext())
            {
                String line = input.nextLine();
                if(line.startsWith("Ka"))
                {
                    line = line.substring(3);
                    String[] lineParts = line.split(" ");
                    double[] material = new double[3];
                    material[0] = Double.parseDouble(lineParts[0]);
                    material[1] = Double.parseDouble(lineParts[1]);
                    material[2] = Double.parseDouble(lineParts[2]);
                    this.setKaMaterial(new Vector3D(material));
                }
                else if(line.startsWith("Kd"))
                {
                    line = line.substring(3);
                    String[] lineParts = line.split(" ");
                    double[] material = new double[3];
                    material[0] = Double.parseDouble(lineParts[0]);
                    material[1] = Double.parseDouble(lineParts[1]);
                    material[2] = Double.parseDouble(lineParts[2]);
                    this.setKdMaterial(new Vector3D(material));
                }
            }
            input.close();
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }


    }

    public RealMatrix transform(ArrayList<Vector3D> p)
    {
        RealMatrix rotate = rotateObj(p, this.rotation, this.getTheta());

        RealMatrix scale = scaleObj(rotate, this.getScale());

        RealMatrix translate = translateObj(scale, this.getTranslation());

        double[][] numbers = translate.getData();
        for(int i = 0; i < numbers.length; i++)
        {
            for(int j = 0; j < numbers[i].length; j++)
            {
                numbers[i][j] = Precision.round(numbers[i][j], 4);
            }
        }
        RealMatrix final_one = MatrixUtils.createRealMatrix(numbers);
        return final_one.transpose();
    }

    public RealMatrix rotateObj(ArrayList<Vector3D> points, Vector3D rotation, double theta){

        Vector3D Wv = rotation.normalize();
        Vector3D Mv  = pickAxis(Wv);
        Vector3D Uv = Wv.crossProduct(Mv);
        Uv = Uv.scalarMultiply(1/Uv.getNorm());
        Vector3D V =  Wv.crossProduct(Uv);
        double[] first_row = new double[]{Uv.getX(), Uv.getY(), Uv.getZ(), 0};
        double[] second_row = new double[]{V.getX(), V.getY(), V.getZ(), 0};
        double[] third_row = new double[]{Wv.getX(), Wv.getY(), Wv.getZ(), 0};
        double[] lastRow = new double[]{0,0,0,1};
        double[][] rows = new double[4][4];
        rows[0] = first_row;
        rows[1] = second_row;
        rows[2] = third_row;
        rows[3] = lastRow;

        RealMatrix RM = MatrixUtils.createRealMatrix(rows);

        RealMatrix RMt = RM.transpose();
        double[][] r_z = new double[][]{{Math.cos(Math.toRadians(theta)), -Math.sin(Math.toRadians(theta)), 0,0},
                {Math.sin(Math.toRadians(theta)), Math.cos(Math.toRadians(theta)), 0,0}, {0,0,1,0}, {0,0,0,1}};
        RealMatrix RMz = MatrixUtils.createRealMatrix(r_z);

        RealMatrix RT = RMt.multiply(RMz).multiply(RM);

        RealMatrix pointsMatrix = arrayListToMatrixRotate(points);

        return RT.multiply(pointsMatrix.transpose());


    }
    public RealMatrix scaleObj(RealMatrix m, double scale) {
        RealMatrix p = MatrixUtils.createRealMatrix(m.getRowDimension(), m.getColumnDimension());
        for(int i = 0; i < m.getColumnDimension(); i++)
        {
            double[] row = m.getColumn(i);
            row[0] *= scale;
            row[1] *= scale;
            row[2] *= scale;
            p.setColumn(i, row);

        }
        return p;
    }
    public RealMatrix translateObj(RealMatrix m, double[] translation){

        RealMatrix p = MatrixUtils.createRealMatrix(m.getRowDimension(), m.getColumnDimension());
        for(int i = 0; i < m.getRowDimension() - 1; i++)
        {
            double[] row = m.getRow(i);
            for(int j = 0; j < row.length; j++)
            {
                row[j] += translation[i];
            }
            p.setRow(i, row);

        }

        return p;


    }
    public Vector3D pickAxis(Vector3D e)
    {
        double[] points = e.toArray();
        double min = points[0];
        int index = 0;
        int i = 0;
        while(i < 3)
        {
            if (points[i] < min)
            {
                index = i;
            }
            i++;
        }
        points[index] = 1.0;
        Vector3D m = new Vector3D(points);
        m = m.scalarMultiply(1/m.getNorm());
        return m;
    }

    public RealMatrix arrayListToMatrixRotate(ArrayList<Vector3D> e)
    {
        double[][] points = new double[e.size()][4];
        for(int i = 0; i < e.size(); i++)
        {
            double[] vector = new double[]{e.get(i).getX(), e.get(i).getY(), e.get(i).getZ(), 1.0};
            points[i] = vector;
        }
        RealMatrix m = MatrixUtils.createRealMatrix(points);

        return m;
    }
    public ArrayList<Vector3D> transformVertexNormals(ArrayList<String> faces, ArrayList<Vector3D> vertices)
    {
        ArrayList<Vector3D> vertexNormals2 = new ArrayList<>();
        for(String s: faces)
        {
            ArrayList<Vector3D> triangle = getTriangleVectors(faces, vertices);
            Vector3D V1 = triangle.get(0).subtract(triangle.get(1));
            Vector3D V2 = triangle.get(1).subtract(triangle.get(2));
            Vector3D newNormal = V1.crossProduct(V2);
            vertexNormals2.add(newNormal);
        }
        return vertexNormals2;
    }

    public ArrayList<Vector3D> getSingleTriangleVectors(String face, ArrayList<Vector3D> vertices)
    {
        ArrayList<Vector3D> triangle = new ArrayList<>();
        ArrayList<Integer> indexes = new ArrayList<>();
        String[] smallerParts;
        face = face.substring(2);
        String[] parts = face.split("\\s+");
        for (String o : parts) {
            smallerParts = o.split("/");
            int i = Integer.parseInt(smallerParts[0]);
            indexes.add(i);
        }
        Vector3D A = vertices.get(indexes.get(0) - 1);
        Vector3D B = vertices.get(indexes.get(1) - 1);
        Vector3D C = vertices.get(indexes.get(2) - 1);
        triangle.add(A);
        triangle.add(B);
        triangle.add(C);
        return triangle;
    }
    public ArrayList<Vector3D> getTriangleVectors(ArrayList<String> faces, ArrayList<Vector3D> vertices)
    {
        ArrayList<Vector3D> triangle = new ArrayList<>();
        for (String s : faces) {
            ArrayList<Integer> indexes = new ArrayList<>();
            String[] smallerParts;
            s = s.substring(2);
            String[] parts = s.split("\\s+");
            for (String o : parts) {
                smallerParts = o.split("/");
                int i = Integer.parseInt(smallerParts[0]);
                indexes.add(i);
            }
            Vector3D A = vertices.get(indexes.get(0) - 1);
            Vector3D B = vertices.get(indexes.get(1) - 1);
            Vector3D C = vertices.get(indexes.get(2) - 1);
            triangle.add(A);
            triangle.add(B);
            triangle.add(C);
        }
        return triangle;
    }

}
