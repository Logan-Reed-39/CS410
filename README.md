# CS410 Ray Tracer

This is my implementation of a ray tracer that has many features to produce interesting and complex PPM images at various desired resolutions. Some features of this program include the ability to render multiple .OBJ files and multiple spheres, using recursion to implement reflections and refraction, and rendering smooth or sharp objects depending on the driver file. I imported the apache commons math linear algebra library to use throughout this program, (and a copy of the library is included in this repository).

This program compiles with the following command:             javac -cp commons-math3-3.6.1/*:. *.java

This program runs with the following command:                 java -cp commons-math3-3.6.1/*:. Raytracer <driverFile> <PPM_File>

The arguments to run are a driver file and an output file to write the image to. The driver file has the following format (as an example) :


eye 3 1 30                
look 0 0 0                
up 0 1 0                   
d 5                        
bounds -2 -2 2 2           
res 256 256                                 
recursionLevel 3                   
ambient 0.2 0.2 0.2                 
light 0 10 20 0 1 1 1                        
light 50 -30 30 1 1 1 1                       
model 0 1 0.0 20 2 0.0 0 0.0 smooth cow.obj                  
sphere 0 -.5 15 3 0 0 1 0 0 1 0 0 0 0.2 0.2 0.2 0.5 0.5 0.5 2.42              


Here is a brief explanation of the contents of the driver file:

**eye**: The location of the camera in x, y, and z coordinates

**look**: The location of the look at point in x, y, and z coordinates

**up**: The "up" direction of the camera. In this case the y axis is the "up" direction.

**d**: The focal length, in other words the distance from the focal point (the camera) to the image plane. 

**bounds**: The min and max values of the bounded image rectangle in the image plane in the horizontal and vertical directions respectively. 

**res**: The desired resolution of the final created image, usually for debugging a low resolution is used (i.e., 256 x 256) for quick output

**recursionLevel**: The amount of desired reflection represented by how many "bounces" a ray will take before calculating the pixel color.

**ambient**: The amount of red, green, and blue ambient light present in the scene

**light**: There can 1 or more light sources in the scene, the first 4 numbers are the light's x,y,z, and w locations followed by the amount of red, green, and blue light it is emmitting in a range from 0 (no color) to 1 (all color).

**model**: Specific model object specifications with transformations. There can be 0 or more models in a scene. The first three numbers represent which axis to rotate the model and by how many degrees (the fourth number). The next number is how much to scale the object. The next three numbers are how much to translate the object along the x,y, and z axis. Next, either >sharp> or >smooth> is inserted to indicate whether the object should render with the triangles (no smoothing) , or if the coloring should use the average of the surface normals for each triangle as the point to color (smoothing). Finally, the name of the .obj file tells the program which object to use, all .obj files in this repository are valid objects to render within this program.

**sphere**: There can be 0 or more spheres in the scene. The first three numbers represent the sphere's x,y, and z location. The fourth value is the radius of the sphere. All the next values are triplets representing the color of the sphere with specific material properties. The first triplet is the red, green, and blue ambient coefficients, the next triplet is diffuse, and the next is specular. The next triplet is the amount of reflection of the sphere, in other words how much reflection is shown on that sphere (0 is no reflection 1 is complete reflection). The next triplet is the amount of refraction for that sphere, in other words how much light is allowed to pass through it (0 means all light passes through, 1 means no light). The last number is the index of refraction for the material, in other words how much the light bends when passing through the sphere. 1.0 typically represents air and 1.5 represents glass.


All of the source files for this program are located in src/com/company. I hope you are able to make some pretty cool images with this program like I have, and it was a great project for me to learn the combined power of Java and linear algebra! 




