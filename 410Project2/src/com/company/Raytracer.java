package com.company;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
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
    private int recusrionLevel;
    private double width;
    private double height;
    private ArrayList<Vertex> vertices = new ArrayList<>();

    private Vector3D ambient;
    private ArrayList<String> lightSources = new ArrayList<>();

    private ArrayList<Model> objects = new ArrayList<>();
    private ArrayList<Sphere> spheres = new ArrayList<>();
    private Camera camera = new Camera();

    private ArrayList<Light> lights = new ArrayList<>();
    private String fileToWrite;
    private Vector3D[][] finalVectors;


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
            if(s.startsWith("recursionLevel"))
            {
                String recData = s.substring(15);
                this.recusrionLevel = Integer.parseInt(recData);
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
            if(s.startsWith("sphere"))
            {
                Sphere p = new Sphere();
                String sNew = s.substring(7);
                String[] sphereParts = sNew.split("\\s+");
                double[] d = new double[3];
                for(int i = 0; i <= 2; i++)
                {
                    d[i] = Double.parseDouble(sphereParts[i]);
                }
                p.setCenterCoordinates(new Vector3D(d));
                p.setSphereRadius(Double.parseDouble(sphereParts[3]));
                double[] e = new double[3];
                e[0] = Double.parseDouble(sphereParts[4]);
                e[1] = Double.parseDouble(sphereParts[5]);
                e[2] = Double.parseDouble(sphereParts[6]);
                p.setAmbientCoeff(e);
                double[] f = new double[3];
                f[0] = Double.parseDouble(sphereParts[7]);
                f[1] = Double.parseDouble(sphereParts[8]);
                f[2] = Double.parseDouble(sphereParts[9]);
                p.setDiffuseCoeff(f);
                double[] g = new double[3];
                g[0] = Double.parseDouble(sphereParts[10]);
                g[1] = Double.parseDouble(sphereParts[11]);
                g[2] = Double.parseDouble(sphereParts[12]);
                p.setSpecularCoeff(g);
                double[] h = new double[3];
                h[0] = Double.parseDouble(sphereParts[13]);
                h[1] = Double.parseDouble(sphereParts[14]);
                h[2] = Double.parseDouble(sphereParts[15]);
                p.setAttenuationCoeff(new Vector3D(h));
                double[] i = new double[3];
                i[0] = Double.parseDouble(sphereParts[16]);
                i[1] = Double.parseDouble(sphereParts[17]);
                i[2] = Double.parseDouble(sphereParts[18]);
                p.setOpacityConstants(new Vector3D(i));
                p.setEta(Double.parseDouble(sphereParts[19]));

                spheres.add(p);

            }
            if(s.startsWith("model"))
            {
                Model obj = new Model();
                ArrayList<Vector3D> transformedPoints2 = new ArrayList<>();
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
                obj.setSoothing(modelParts[8]);
                obj.setModelObjName(modelParts[9]);
                // Get points of model object
                ArrayList<Vector3D> modelPoints = obj.getCoordinates();
                // Transform the model object
                RealMatrix final1 = obj.transform(modelPoints);
                for(int i = 0; i < final1.getRowDimension(); i++)
                {
                    double[] a = final1.getRow(i);
                    Vector3D p = new Vector3D(a[0], a[1], a[2]);
                    transformedPoints2.add(p);
                }
                obj.setTransformedPoints(transformedPoints2);
                int vertexCount = 0;
                for(Vector3D e1: modelPoints)
                {
                    Vertex i = new Vertex(e1, vertexCount);
                    int faceCount = 0;
                    for(String face: obj.getFaces())
                    {
                        ArrayList<Integer> indexes = new ArrayList<>();
                        String[] smallerParts;
                        face = face.substring(2);
                        String[] parts = face.split("\\s+");
                        for (String o : parts) {
                            smallerParts = o.split("/");
                            int p = Integer.parseInt(smallerParts[0]);
                            indexes.add(p);
                        }
                        int f1 = (indexes.get(0) - 1);
                        int f2 = (indexes.get(1) - 1);
                        int f3 = (indexes.get(2) - 1);
                        if(vertexCount == f1 || vertexCount == f2 || vertexCount == f3)
                        {
                            i.addToFaces(faceCount);
                        }
                        faceCount++;
                    }
                    vertexCount++;
                    vertices.add(i);

                }
                objects.add(obj);

            }

        }

    }
    public void cameraSetup() {
        Camera c = new Camera();
        c.setEV(new Vector3D(eyes));
        c.setLV(new Vector3D(lookAt));
        c.setUP(new Vector3D(upV));
        Vector3D WV = c.getEV().subtract(c.getLV());
        WV = WV.normalize();
        c.setWV(WV);
        Vector3D UV = c.getUP().crossProduct(WV);
        UV = UV.normalize();
        c.setUV(UV);
        c.setVV(WV.crossProduct(UV));
        this.camera = c;
        castColor();
    }
    public void castColor()
    {
        for(int i = 0; i <= (int)this.getWidth()-1; i++)
        {
            for(int j = (int)this.getHeight()-1; j >=0; j--)
            {
                Ray r = pixelRay(j, i, this.camera);
                Vector3D rgb = new Vector3D(0,0,0);
                Vector3D pixel = convertColor(rayCast(r, objects, spheres, rgb, new Vector3D(1,1,1), this.recusrionLevel));
                finalVectors[i][j] = pixel;

            }
        }
        writePPMFile(finalVectors);
    }
    public Vector3D rayCast(Ray r, ArrayList<Model> objects, ArrayList<Sphere> spheres, Vector3D accum, Vector3D refatt, int recurse)
    {
        Vector3D pixel;
        Vector3D snrm;
        if(objects.size() != 0) {
            for (Model obj : objects) {
                modelRayCast(r, obj);
            }
        }
        if(spheres.size() != 0) {
            for (Sphere p : spheres) {
                sphereRayCast(r, p);
            }
        }
        if (r.getBestModel() != null) {
            pixel = ModelLightPixel(r, accum, refatt);
            ArrayList<Vector3D> minFace = r.getBestModel().getSingleTriangleVectors(r.getBestFace(), r.getBestModel().getTransformedPoints());
            // Get the three vertices of the triangle
            Vector3D Av = minFace.get(0);
            Vector3D Bv = minFace.get(1);
            Vector3D Cv = minFace.get(2);

            Vector3D AV_BV = Av.subtract(Bv);
            Vector3D AV_CV = Av.subtract(Cv);
            // Calculate surface normal
            snrm = AV_BV.crossProduct(AV_CV);

            if(r.getBestModel().getSoothing().equals("smooth"))
            {
                snrm = smoothNormal(r, snrm);
            }
            snrm = snrm.normalize();

        }
        else if(r.getBestSphere() != null)
        {
            pixel = SphereLightPixel(r, accum, refatt);
            snrm = r.getBestPoint().subtract(r.getBestSphere().getCenterCoordinates());
            snrm = snrm.normalize();
        }
        else
        {
            return accum;
        }
        accum = accum.add(new Vector3D(refatt.getX() * pixel.getX(), refatt.getY() * pixel.getY(), refatt.getZ() * pixel.getZ()));
        if (recurse > 0) {
            Vector3D Uinv = r.getV().scalarMultiply(-1);
//            refR = make_unit((2 * np.dot(N, Uinv) * N) - Uinv)
            Vector3D refR = (snrm.scalarMultiply((snrm.dotProduct(Uinv))).scalarMultiply(2)).subtract(Uinv);
            refR = refR.normalize();

            // Now multiply calculated reflection by reflection coefficients of object
            if(r.getBestSphere() != null)
            {
                Sphere p = r.getBestSphere();
//                mat.kr * refatt
                Vector3D newRefatt = new Vector3D(p.getAttenuationCoeff().getX() * refatt.getX(),
                        p.getAttenuationCoeff().getY() * refatt.getY(),
                        p.getAttenuationCoeff().getZ() * refatt.getZ());

                if(p.getOpacityConstants().getX() + p.getOpacityConstants().getY() + p.getOpacityConstants().getZ() < 3.0)
                {
                    Vector3D thru = new Vector3D(0,0,0);
                    Ray fraR = p.refract_exit(r.getV().scalarMultiply(-1), r.getBestPoint(), p.getEta());
                    if (fraR != null)
                    {
                        accum = accum.add(new Vector3D(refatt.getX() * (1.0 - p.getOpacityConstants().getX()) * thru.getX(),
                                refatt.getY() * (1.0 - p.getOpacityConstants().getY()) * thru.getY(),
                                refatt.getZ() * (1.0 - p.getOpacityConstants().getZ()) * thru.getZ()));
                        return rayCast(fraR,objects,spheres,accum,newRefatt, recurse-1);
//                        Vector3D newAccum = rayCast(fraR, objects, spheres, thru, newRefatt, recurse - 1);
//                        accum = accum.add(new Vector3D(refatt.getX() * (1.0 - p.getOpacityConstants().getX()) * newAccum.getX(),
//                                refatt.getY() * (1.0 - p.getOpacityConstants().getY()) * newAccum.getY(),
//                                refatt.getZ() * (1.0 - p.getOpacityConstants().getZ()) * newAccum.getZ()));
                    }
                }

//              ray_trace(Ray(ray.best_pt, refR), accum, mat.kr * refatt, (level - 1))
                return rayCast(new Ray(r.getBestPoint(), refR), objects, spheres, accum, newRefatt, recurse - 1);
            }
            else if(r.getBestModel() != null) {
//                mat.kr * refatt
                Vector3D newRefatt = new Vector3D(r.getBestModel().getAttentuationCoef().getX() * refatt.getX(),
                        r.getBestModel().getAttentuationCoef().getY() * refatt.getY(),
                        r.getBestModel().getAttentuationCoef().getZ() * refatt.getZ());

//              ray_trace(Ray(ray.best_pt, refR), accum, mat.kr * refatt, (level - 1))
                return rayCast(new Ray(r.getBestModelPoint(), refR), objects, spheres, accum, newRefatt, recurse - 1);
            }
        }
        return accum;
    }
    public Ray pixelRay(double i, double j, Camera c)
    {
        double px = ((i/(this.getWidth()-1)) * (this.getBounds()[2] - this.getBounds()[0]) + this.getBounds()[0]);
        double py = ((j/(this.getHeight() - 1)) * (this.getBounds()[3] - this.getBounds()[1]) + this.getBounds()[1]);
        Vector3D pixpt = c.getEV().add(c.getWV().scalarMultiply(this.getFocalLength()*-1)).add(c.getUV().scalarMultiply(px)).add(c.getVV().scalarMultiply(py));
        Vector3D D = pixpt.subtract(c.getEV()); D = D.normalize();
        Ray r = new Ray(pixpt, D);
        return r;

    }

    public boolean sphereRayCast(Ray ray, Sphere sph)
    {
        double r = sph.getSphereRadius();
        Vector3D Cv = sph.getCenterCoordinates();
        Vector3D Lv = ray.getP();
        Vector3D Uv = ray.getV();
        Vector3D Tv = Cv.subtract(Lv);
        double v = Tv.dotProduct(Uv);
        double csq = Tv.dotProduct(Tv);
        double disc = Math.pow(r,2) - (csq - Math.pow(v,2));
        if(disc < 0){ return false; }
        double d = Math.sqrt(disc);
        double tPrime = v - d;
        if(tPrime < ray.getBestT() && tPrime > 0.000001) {
            Vector3D pt = Lv.add(Uv.scalarMultiply(tPrime));
            ray.setBestT(tPrime);
            ray.setBestSphere(sph);
            ray.setBestPoint(pt);
            ray.setBestModel(null);
            return true;
        }
        return false;
    }
    public void modelRayCast(Ray ray, Model obj) {
        // All the faces for this object
        ArrayList<String> faces = obj.getFaces();

        // Get the ray location and direction vectors
        Vector3D LV = ray.getP();
        Vector3D Dv = ray.getV();
        // For each modelFace for the object
        for (String s : faces) {
            // Get the triangle vectors associated with that modelFace
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
                    if ((t > 0.000001) && (t < ray.getBestT())) {

                        ray.setBestT(t);
                        ray.setBestFace(s);
                        ray.setBestSphere(null);
                        ray.setBestModel(obj);
                        ray.setBeta(beta);
                        ray.setGamma(gamma);
                        // Point of intersection between ray and surface (pixpt + directionOfRay * t)
                        ray.setBestModelPoint(ray.getP().add(ray.getV().scalarMultiply(t)));
                    }
                }

            }
        }
    }
    public boolean hasShadow(Ray r, ArrayList<Model> objects, ArrayList<Sphere> spheres)
    {
        Vector3D LV = r.getP();
        Vector3D Dv = r.getV();
        for(Model obj: objects)
        {
            ArrayList<String> faces = obj.getFaces();
            for(String s : faces)
            {
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
                        if (((t) > 0.00000001)) {
                            return true;
                        }
                    }

                }
            }
        }
        for(Sphere p: spheres)
        {
            double d = p.getSphereRadius();
            Vector3D Cv = p.getCenterCoordinates();
            Vector3D Tv = Cv.subtract(LV);
            double v = Tv.dotProduct(Dv);
            double csq = Tv.dotProduct(Tv);
            double disc = Math.pow(d,2) - (csq - Math.pow(v,2));
            if(disc < 0){ return false; }
            double dS = Math.sqrt(disc);
            double tPrime = v - dS;
            if(tPrime > 0.00000001) {
                return true;
            }
        }
        return false;
    }
    public Vector3D SphereLightPixel(Ray r, Vector3D accum, Vector3D refatt){
            Vector3D snrm = r.getBestPoint().subtract(r.getBestSphere().getCenterCoordinates());
            snrm = snrm.normalize();
            Vector3D ambientLight = new Vector3D(
                    this.getAmbient().getX() * r.getBestSphere().getAmbientCoeff()[0],
                    this.getAmbient().getY() * r.getBestSphere().getAmbientCoeff()[1],
                    this.getAmbient().getZ() * r.getBestSphere().getAmbientCoeff()[2]);
            for (Light light : lights) {
                Vector3D ptL = light.getLocationL();
                Vector3D toL = ptL.subtract(r.getBestPoint());
                toL = toL.normalize();
                Ray rayP = new Ray(r.getBestPoint(), toL);
                if (hasShadow(rayP, this.objects, this.spheres)) {
                    continue;
                }
                if (snrm.dotProduct(toL) > 0.0) {
                    ambientLight = ambientLight.add(new Vector3D(
                            r.getBestSphere().getDiffuseCoeff()[0] * light.getLightColor().getX(),
                            r.getBestSphere().getDiffuseCoeff()[1] * light.getLightColor().getY(),
                            r.getBestSphere().getDiffuseCoeff()[2] * light.getLightColor().getZ()).scalarMultiply(snrm.dotProduct(toL)));
                    Vector3D toC = r.getP().subtract(r.getBestPoint());
                    toC = toC.normalize();
                    Vector3D spR = ((snrm.scalarMultiply(snrm.dotProduct(toL)).scalarMultiply(2)).subtract(toL));
                    double cdR = toC.dotProduct(spR);
                    if (cdR > 0.0) {
                        ambientLight = ambientLight.add(new Vector3D(
                                r.getBestSphere().getSpecularCoeff()[0] * light.getLightColor().getX(),
                                r.getBestSphere().getSpecularCoeff()[1] * light.getLightColor().getY(),
                                r.getBestSphere().getSpecularCoeff()[2] * light.getLightColor().getZ()).scalarMultiply(Math.pow(cdR, 16)));
                    }
                }
            }
            return accum.add(new Vector3D(refatt.getX() * r.getBestSphere().getOpacityConstants().getX() * ambientLight.getX(),
                    refatt.getY() * r.getBestSphere().getOpacityConstants().getY() * ambientLight.getY(),
                    refatt.getZ() * r.getBestSphere().getOpacityConstants().getZ() * ambientLight.getZ()));
    }

    public Vector3D smoothNormal(Ray r, Vector3D hitNormal)
    {
        Model obj = r.getBestModel();
        // Get the modelFace associated with the closest t-value
        ArrayList<Vector3D> minFace = obj.getSingleTriangleVectors(r.getBestFace(), r.getBestModel().getTransformedPoints());
        // Get the three vertices of the triangle
        Vector3D Av = minFace.get(0);
        Vector3D Bv = minFace.get(1);
        Vector3D Cv = minFace.get(2);

        ArrayList<Vector3D> validNormalsA = findValidNormals(r, obj, Av, hitNormal);
        Vector3D ANormalAverage = calculateVectorAverage(validNormalsA);

        ArrayList<Vector3D> validNormalsB = findValidNormals(r, obj, Bv, hitNormal);
        Vector3D BNormalAverage = calculateVectorAverage(validNormalsB);

        ArrayList<Vector3D> validNormalsC = findValidNormals(r, obj, Cv, hitNormal);
        Vector3D CNormalAverage = calculateVectorAverage(validNormalsC);

        Vector3D FinalNormal = ANormalAverage.scalarMultiply((1-r.getBeta()-r.getGamma())).add(BNormalAverage.scalarMultiply(r.getBeta()))
                .add(CNormalAverage.scalarMultiply(r.getGamma()));
        return FinalNormal.normalize();
    }
    public ArrayList<Vector3D> findValidNormals(Ray r, Model obj, Vector3D Av, Vector3D hitNormal)
    {
        ArrayList<Vector3D> validNormalsA = new ArrayList<>();
        // Get index of vertex A of the triangle to look up adjoining faces
        int indexA = obj.getTransformedPoints().indexOf(Av);
        // For each face that is connected to point A
        for(int A: vertices.get(indexA).getFaces())
        {
            // Get the face of that vertex
            String face = r.getBestModel().getFaces().get(A);
            // Get the triangle points of that face
            ArrayList<Vector3D> facePoints = r.getBestModel().getSingleTriangleVectors(face, obj.getTransformedPoints());
            // Calculate true normal of that face
            Vector3D trueNormal = calculateTrueNormal(facePoints);
            if(hitNormal.dotProduct(trueNormal) > 0.92387953251)
            {
                validNormalsA.add(trueNormal);
            }
        }
        return validNormalsA;
    }
    public Vector3D calculateVectorAverage(ArrayList<Vector3D> A)
    {
        int Asize= A.size();
        double Ax = 0;
        double Ay = 0;
        double Az = 0;
        for (Vector3D x : A)
        {
            Ax += x.getX();
            Ay += x.getY();
            Az += x.getZ();
        }
        Ax = Ax / Asize;
        Ay = Ay / Asize;
        Az = Az / Asize;
        double[] Adouble = new double[]{Ax, Ay, Az};
        return new Vector3D(Adouble);
    }
    public Vector3D calculateTrueNormal(ArrayList<Vector3D> minFace)
    {
        Vector3D A = minFace.get(0);
        Vector3D B = minFace.get(1);
        Vector3D C = minFace.get(2);
        Vector3D AV_BV = A.subtract(B);
        Vector3D AV_CV = A.subtract(C);
        // Calculate surface normal
        Vector3D N = AV_BV.crossProduct(AV_CV);
        N = N.normalize();
        return N;
    }

    public Vector3D ModelLightPixel(Ray r, Vector3D accum, Vector3D refatt){
        Vector3D totalLight = new Vector3D(0,0,0);
        if ((r.getBestT() != 9999) && (!r.getBestFace().equals(""))) {
            // Get the ambient light by multiplying the Ka values in the material file by the ambient values in the driver file
            Vector3D ambientLight = new Vector3D(
                    r.getBestModel().getKaMaterial().getX() * this.getAmbient().getX(),
                    r.getBestModel().getKaMaterial().getY() * this.getAmbient().getY(),
                    r.getBestModel().getKaMaterial().getZ() * this.getAmbient().getZ());
            for (Light light : lights) {
                // Get location of light source, given in driver file
                Vector3D L = light.getLocationL();
                // Point of intersection between ray and surface (pixpt + directionOfRay * t)
                Vector3D POI = r.getP().add(r.getV().scalarMultiply(r.getBestT()));
                // If w = 1 for light
                if (light.isW1()) {
                    L = L.subtract(POI);
                }
                L = L.normalize();
                Ray rayP = new Ray(POI, L);
                totalLight = totalLight.add(ambientLight);
                if(hasShadow(rayP, this.objects, this.spheres))
                {
                    continue;
                }

                // Get the modelFace associated with the closest t-value
                ArrayList<Vector3D> minFace = r.getBestModel().getSingleTriangleVectors(r.getBestFace(), r.getBestModel().getTransformedPoints());
                // Get the three vertices of the triangle
                Vector3D Av = minFace.get(0);
                Vector3D Bv = minFace.get(1);
                Vector3D Cv = minFace.get(2);

                Vector3D AV_BV = Av.subtract(Bv);
                Vector3D AV_CV = Av.subtract(Cv);
                // Calculate surface normal
                Vector3D N = AV_BV.crossProduct(AV_CV);
                N = N.normalize();
                if(r.getBestModel().getSoothing().equals("smooth"))
                {
                    N = smoothNormal(r, N);
                }
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
                            r.getBestModel().getKdMaterial().getX() * light.getLightColor().getX(),
                            r.getBestModel().getKdMaterial().getY() * light.getLightColor().getY(),
                            r.getBestModel().getKdMaterial().getZ() * light.getLightColor().getZ());

                    KDB = KDB.scalarMultiply(cosineTheta);

                    // Specular Reflection Calculation
                    // V = Ray Location - Point of Intersection
                    Vector3D toCamera = r.getP().subtract(POI); toCamera = toCamera.normalize();
                    // (N * (N dot L) * 2) - L
                    Vector3D spR = ((N.scalarMultiply(N.dotProduct(L)).scalarMultiply(2)).subtract(L));
                    double CDR = toCamera.dotProduct(spR);
                    Vector3D e = new Vector3D(0,0,0);
                    if (CDR > 0.0)
                    {
                        Vector3D colorPhong = new Vector3D(
                                r.getBestModel().getKsMaterial().getX()* light.getLightColor().getX(),
                                r.getBestModel().getKsMaterial().getY() * light.getLightColor().getY(),
                                r.getBestModel().getKsMaterial().getZ() * light.getLightColor().getZ())
                                    .scalarMultiply(Math.pow(CDR, r.getBestModel().getPhongConstant()));
                        e = e.add(colorPhong);
                    }

                    e = e.add(KDB);
                    totalLight = totalLight.add(e);
                }
            }
        }
        return totalLight;
    }
    public void writePPMFile(Vector3D[][] finalVectors)
    {
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
    public Vector3D convertColor(Vector3D original)
    {
        double origX = original.getX();
        double origY = original.getY();
        double origZ = original.getZ();
        if(origX > 1){ origX = 1;}
        if(origY > 1){ origY = 1;}
        if(origZ > 1){ origZ = 1;}
        origX *= 255;
        origY *= 255;
        origZ *= 255;
        Vector3D finalAmbient = new Vector3D(origX,origY, origZ);
        return finalAmbient;
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
