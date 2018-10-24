package com.company;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.io.*;
import java.util.ArrayList;
import java.util.Scanner;

public class Raytracer {
    private Scanner input;
    ArrayList<String> fileData = new ArrayList<>();


    private double[] eyes = new double[3];
    private double[] lookAt = new double[3];
    private double[] upV = new double[3];
    private double focalLength;
    private double[] bounds = new double[4];
    private double[] resolution = new double[2];
    private double width;
    private double height;
    private double sTval = 0.0;
    private String face = "";
    private Vector3D ambient;
    private ArrayList<String> lightSources = new ArrayList<>();
    private ArrayList<Vector3D> vertexNormals = new ArrayList<>();
    private ArrayList<Point> points = new ArrayList<>();
    private ArrayList<Ray> rays = new ArrayList<>();
    private ArrayList<Model> objects = new ArrayList<>();
    private ArrayList<Light> lights = new ArrayList<>();
    private ArrayList<Vector3D>  transformedPoints = new ArrayList<>();
    private ArrayList<Vector3D> finalColoredPoints = new ArrayList<>();
    private String fileToWrite;
    private Vector3D[][] finalVectors;
    private boolean W1;


    public ArrayList<Vector3D> getTransformedPoints() { return transformedPoints; }

    public double getFocalLength() { return focalLength; }
    public double[] getBounds() { return bounds; }
    public double getWidth() { return width;}
    public double getHeight() { return height;}
    public void setFileToWrite(String s){ this.fileToWrite = s;}
    public Vector3D getAmbient() { return this.ambient;}
    public void setAmbient(Vector3D e){ this.ambient = e;}


    public static void main(String[] args) {
        Raytracer trace = new Raytracer(args[0], args[1]);

    }

    public Raytracer(String driverFile, String modelFile)
    {
        File driver = new File(driverFile);
        readInFile(driver);
        setFileToWrite(modelFile);
        cameraSetup();
    }
    private void readInFile(File textFile)
    {
        try {
            input = new Scanner(textFile);
            while (input.hasNextLine())
            {
                String line = input.nextLine();
                fileData.add(line);
            }
            input.close();


        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
        for(String s: fileData)
        {
            if(s.startsWith("eye"))
            {
                String eyeData = s.substring(4);
                String[] eyeArray = eyeData.split("\\s+");
                for(int i = 0; i <= 2; i++)
                {
                    eyes[i] = Double.parseDouble(eyeArray[i]);
                }

            }
            if(s.startsWith("look"))
            {
                String lookData = s.substring(5);
                String[] lookArray = lookData.split("\\s+");
                for(int i = 0; i <= 2; i++)
                {
                    lookAt[i] = Double.parseDouble(lookArray[i]);
                }
            }
            if(s.startsWith("up"))
            {
                String UpData = s.substring(3);
                String[] UpArray = UpData.split("\\s+");
                for(int i = 0; i <= 2; i++)
                {
                    upV[i] = Double.parseDouble(UpArray[i]);
                }
            }
            if(s.startsWith("d"))
            {
                String distData = s.substring(1);
                focalLength = Double.parseDouble(distData);
            }
            if(s.startsWith("bounds"))
            {
                String boundsData = s.substring(7);
                String[] boundsArray = boundsData.split("\\s+");
                for(int i = 0; i < 4; i++)
                {
                    bounds[i] = Double.parseDouble(boundsArray[i]);
                }
            }
            if(s.startsWith("res"))
            {
                String resData = s.substring(4);
                String[] resArray = resData.split("\\s+");
                for(int i = 0; i < 2; i++)
                {
                    resolution[i] = Double.parseDouble(resArray[i]);
                }
                this.width = resolution[0];
                this.height = resolution[1];
                this.finalVectors = new Vector3D[(int)resolution[0]][(int)resolution[1]];
            }
            if(s.startsWith("ambient"))
            {
                String ambData = s.substring(8);
                String[] ambArray = ambData.split("\\s+");
                double[] ambientD = new double[]{Double.parseDouble(ambArray[0]), Double.parseDouble(ambArray[1]),
                            Double.parseDouble(ambArray[2])};
                Vector3D ambVector = new Vector3D(ambientD);
                this.setAmbient(ambVector);

            }
            if(s.startsWith("light"))
            {
                lightSources.add(s);
                Light l = new Light();
                String[] parts = s.split("\\s+");
                double[] location = new double[]{Double.parseDouble(parts[1]), Double.parseDouble(parts[2]),
                        Double.parseDouble(parts[3]), Double.parseDouble(parts[4])};
                if(location[3] == 0)
                {
                    l.setW1(false);
                }
                if(location[3] == 1)
                {
                    l.setW1(true);
                }
                Vector3D lightVector = new Vector3D(location[0], location[1], location[2]);

                l.setLocationL(lightVector);
                double[] color = new double[]{Double.parseDouble(parts[5]), Double.parseDouble(parts[6]), Double.parseDouble(parts[7])};
                l.setLightColor(new Vector3D(color));
                lights.add(l);
            }
            if(s.startsWith("model"))
            {
                Model obj = new Model();
                String sNew = s.substring(6);
                // Split the model line by whitespace
                String[] modelParts = sNew.split("\\s+");
                double[] d = new double[3];
                for(int i = 0; i <=2; i++)
                {
                    d[i] = Double.parseDouble(modelParts[i]);
                }
                Vector3D rotate = new Vector3D(d);
                // Set Rotation Vector
                obj.setRotation(rotate);
                // Set Theta
                obj.setTheta(Double.parseDouble(modelParts[3]));
                // Set Scale
                obj.setScale(Double.parseDouble(modelParts[4]));
                double[] e = new double[3];
                e[0] = Double.parseDouble(modelParts[5]);
                e[1] = Double.parseDouble(modelParts[6]);
                e[2] = Double.parseDouble(modelParts[7]);
                // Set translation
                obj.setTranslation(e);
                obj.setModelObjName(modelParts[8]);
                // Get points of model object
                ArrayList<Vector3D> modelPoints = obj.getCoordinates();
                // Transform the model object
                RealMatrix final1 = obj.transform(modelPoints);
                for(int i = 0; i < final1.getRowDimension(); i++)
                {
                    double[] a = final1.getRow(i);
                    Vector3D p = new Vector3D(a[0], a[1], a[2]);
                    this.transformedPoints.add(p);
                }
                obj.setTransformedPoints(this.transformedPoints);
                objects.add(obj);
//                writeOBJFile(final1, obj);
            }

        }

    }
    public void cameraSetup()
    {
        Vector3D EV = new Vector3D(eyes);
        Vector3D LV = new Vector3D(lookAt);
        Vector3D UP = new Vector3D(upV);
        Vector3D WV = EV.subtract(LV);
        WV = WV.normalize();
        Vector3D UV = UP.crossProduct(WV);
        UV = UV.normalize();
        Vector3D VV = WV.crossProduct(UV);
        Model obj = objects.get(0);
        for(int i = 0; i <= (int)this.getWidth()-1; i++)
        {
            for(int j = (int)this.getHeight()-1; j >=0; j--)
            {
                Ray r = pixelRay(j,i, EV, WV, UV, VV);
                rayCast2(r, obj);
                Vector3D pixel = lightPixel(obj, this.sTval, this.face, r);
                finalVectors[i][j] = pixel;
            }
        }
        writePPMFile(finalVectors);

    }
    public void writePPMFile(Vector3D[][] finalVectors)
    {
        System.out.println("Writing the file");
        try
        {
            File imageFile = new File(this.fileToWrite);
            BufferedWriter writer = new BufferedWriter(new FileWriter(imageFile));
            writer.write("P3" + "\n");
            writer.write((int)resolution[0] + " " + (int)resolution[1] + " " + "255" + "\n");
            for(int i = finalVectors.length-1; i >= 0; i--)
            {
                for(int j = 0; j < finalVectors[i].length; j++)
                {
                    String p = (int)finalVectors[i][j].getX() + " " + (int)finalVectors[i][j].getY() + " " + (int)finalVectors[i][j].getZ() + " \n";
                    writer.write(p);
                }
            }
        writer.close();
        } catch(IOException e)
        {
            e.printStackTrace();
        }

    }

    public Ray pixelRay(double i, double j, Vector3D EV, Vector3D WV, Vector3D UV, Vector3D VV)
    {
        double px = ((i/(this.getWidth()-1)) * (this.getBounds()[2] - this.getBounds()[0]) + this.getBounds()[0]);
        double py = ((j/(this.getHeight() - 1)) * (this.getBounds()[3] - this.getBounds()[1]) + this.getBounds()[1]);
        Vector3D pixpt = EV.add(WV.scalarMultiply(this.getFocalLength()*-1)).add(UV.scalarMultiply(px)).add(VV.scalarMultiply(py));
        Vector3D D = pixpt.subtract(EV); D = D.normalize();
        Ray r = new Ray(pixpt, D);
        return r;

    }
    public void rayCast2(Ray r, Model obj) {
        Vector3D totalLight = new Vector3D(0, 0, 0);
        // All the faces for this object
        ArrayList<String> faces = obj.getFaces();
        // Arbitrary large T-value to start when getting min
        double sTval = 9999;
        // Arbitrary blank string for face associated with minimum t
        String face = "";
        // For each face for the object
        // Get the ray location and direction vectors
        Vector3D LV = r.getP();
        Vector3D Dv = r.getV();
        for (String s : faces) {
            // Get the triangle vectors associated with that face
            ArrayList<Vector3D> triangles = obj.getSingleTriangleVectors(s, obj.getTransformedPoints());
            double Ax = triangles.get(0).getX();
            double Ay = triangles.get(0).getY();
            double Az = triangles.get(0).getZ();

            double Bx = triangles.get(1).getX();
            double By = triangles.get(1).getY();
            double Bz = triangles.get(1).getZ();

            double Cx = triangles.get(2).getX();
            double Cy = triangles.get(2).getY();
            double Cz = triangles.get(2).getZ();

            double Lx = LV.getX();
            double Ly = LV.getY();
            double Lz = LV.getZ();

            double Dx = Dv.getX();
            double Dy = Dv.getY();
            double Dz = Dv.getZ();

            double AxBx = Ax - Bx;
            double AyBy = Ay - By;
            double AzBz = Az - Bz;

            double AxCx = Ax - Cx;
            double AyCy = Ay - Cy;
            double AzCz = Az - Cz;

            double AxLx = Ax - Lx;
            double AyLy = Ay - Ly;
            double AzLz = Az - Lz;

            double deterDenom = ((((AzCz * Dy) - (AyCy * Dz)) * AxBx) - (((AzCz * Dx) - (AxCx * Dz)) * AyBy) + (((AyCy * Dx) - (AxCx * Dy)) * AzBz));
            double betaTop = ((((AzCz * Dy) - (AyCy * Dz)) * AxLx) - (((AzCz * Dx) - (AxCx * Dz)) * AyLy) + (((AyCy * Dx) - (AxCx * Dy)) * AzLz));
            double beta = betaTop / deterDenom;
            if (beta >= 0) {
                double gammaTop = ((((AzLz * Dy) - (AyLy * Dz)) * AxBx) - (((AzLz * Dx) - (AxLx * Dz)) * AyBy) + (((AyLy * Dx) - (AxLx * Dy)) * AzBz));
                double gamma = gammaTop / deterDenom;
                if ((gamma >= 0) && (beta + gamma <= 1)) {
                    double T_Top = ((((AyLy * AzCz) - (AyCy * AzLz)) * AxBx) - (((AxLx * AzCz) - (AxCx * AzLz)) * AyBy) + (((AxLx * AyCy) - (AxCx * AyLy)) * AzBz));
                    double t = T_Top / deterDenom;
                    if (((t) > 0) && ((t) < sTval)) {
                        sTval = t;
                        face = s;
                    }
                }

            }
        }
        this.sTval = sTval;
        this.face = face;
    }
    public Vector3D lightPixel(Model obj, double sTval, String face, Ray r){
        Vector3D totalLight = new Vector3D(0,0,0);
        if ((sTval != 9999) && (!face.equals(""))) {
            // Get the ambient light by multiplying the Ka values in the material file by the ambient values in the driver file
            Vector3D ambientLight = new Vector3D(
                    obj.getKaMaterial().getX() * this.getAmbient().getX(),
                    obj.getKaMaterial().getY() * this.getAmbient().getY(),
                    obj.getKaMaterial().getZ() * this.getAmbient().getZ());

            for (Light light : lights) {
                // Get location of light source, given in driver file
                Vector3D L = light.getLocationL();
                // Point of intersection between ray and surface (pixpt + directionOfRay * t)
                Vector3D POI = r.getP().add(r.getV().scalarMultiply(sTval));
                // If w = 1 for light
                if (light.isW1()) {
                    L = L.subtract(POI);
                }
                L = L.normalize();

                // Get the face associated with the closest t-value
                ArrayList<Vector3D> minFace = obj.getSingleTriangleVectors(face, obj.getTransformedPoints());
                // Get the three vertices of the triangle
                Vector3D Av = minFace.get(0);
                Vector3D Bv = minFace.get(1);
                Vector3D Cv = minFace.get(2);

                Vector3D AV_BV = Av.subtract(Bv);
                Vector3D AV_CV = Av.subtract(Cv);
                // Calculate surface normal
                Vector3D N = AV_BV.crossProduct(AV_CV);
                N = N.normalize();
                // Check if N * V (ray direction) > 0
                if((N.dotProduct(r.getV())) > 0)
                {
                    N = N.scalarMultiply(-1);
                }
                // Check if N * (L) is positive
                if((N.dotProduct(L)) > 0)
                {
                    // Compute cosine theta
                    double cosineTheta = N.dotProduct(L);
                    // Build vector for Kd by multiplying every Kd value in the material file by the color of the light in the
                    // driver file
                    Vector3D KDB = new Vector3D(
                            obj.getKdMaterial().getX() * light.getLightColor().getX(),
                            obj.getKdMaterial().getY() * light.getLightColor().getY(),
                            obj.getKdMaterial().getZ() * light.getLightColor().getZ());

                    KDB = KDB.scalarMultiply(cosineTheta);


                    Vector3D e = KDB.add(ambientLight);

                    double ambX = e.getX();
                    double ambY = e.getY();
                    double ambZ = e.getZ();
                    if(ambX > 1){ ambX = 1;}
                    if(ambY > 1){ ambY = 1;}
                    if(ambZ > 1){ ambZ = 1;}
                    ambX *= 255;
                    ambY *= 255;
                    ambZ *= 255;
                    Vector3D finalLight = new Vector3D(ambX, ambY, ambZ);
                    totalLight = totalLight.add(finalLight);

                }
            }
        }
        return totalLight;

    }

    public void writeOBJFile(RealMatrix m, Model obj)
    {
        String objName =  obj.getModelObjName();
        objName = objName.substring(0, objName.lastIndexOf('.'));
        ArrayList<String> points = new ArrayList<>();
        for(int i = 0; i < m.getRowDimension(); i++)
        {
            double[] d = m.getRow(i);
            double d1 = d[0];
            double d2 = d[1];
            double d3 = d[2];
            String s = Double.toString(d1) + " " + Double.toString(d2) + " " + Double.toString(d3);
            points.add(s);
        }
        ArrayList<String> contents = obj.getModelFileContents();
        try
        {
            int i = 0;
            File objFile;
            do {
                objFile = new File(objName + "_mw" + String.format("%02d", i) + ".obj" );
                i++;
            } while(objFile.exists());
            BufferedWriter writer = new BufferedWriter(new FileWriter(objFile));
            for (String s : contents) {
                if (s.startsWith("#") || (s.startsWith("mtllib"))) {
                    writer.write(s + "\n");
                }
            }
            for (String s : points) {
                writer.write("v " + s + "\n");
            }
            for (String s : contents) {
                if (s.startsWith("f ")) {
                    writer.write(s + "\n");
                }
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
}
