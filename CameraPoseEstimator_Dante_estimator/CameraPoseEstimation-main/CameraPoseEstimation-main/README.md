#  Camera Pose Estimation -- Computer Vision Final Project
## Acknowledgment
This work has been devoleped by Simone Cremasco and Edoardo Fiorini for the course of Computer Vision at the University of Verona.
## Problem Statment
This work implements a pipeline to estimate the camera pose automatically. Starting from a reference image of which we know 2D-3D correspondences and internal/external parameters and given a new image with only known internal parameters, we are able to compute the external parameters of the new image. 
The main points of the pipeline are the following one:
- Load the reference and new image known informations
- Extract the [SIFT](https://www.vlfeat.org/overview/sift.html#tut.sift.match) points on the reference image, in this way it is trained to recognize a certain set of 2D points for which the 3D correspondence is known 
- Extract the SIFT points on the new image and put them in correspondence with the points of the reference image. As all points on reference image have 3D correspondence, also the new image will have same 3D correspondence for matched 2D points.
- Filter the possible outliers matching with RANSAC algorithm
- Apply the method to estimate the camera pose(Fiore, POSIT, Lowe and Anisotropic Orthogonal Procustes Analysis), taken as input the 2D-3D correspondeces of the new image along with internal params of new camera.
 
## Dataset 
The camera pose estimation has been tested on two differend dataset:
- Ca vignal dataset
- Dante dataset, which is available on [3DFlow](https://www.3dflow.net/it/3df-zephyr-vetrina-di-ricostruzioni/) website

## Folder structure
The project is structured as follow:
- **classes**: contains the definition of what we call the **reference model** and an enumaration to identifiy the estimator method.
- **functions**: contains all utilities methods
- **models**: contains some example models
- **cav** and **dante**: contain some pictures and data used for testing.

Demo scripts are in the main folder.

## Reference Model
In order to come to an estimation for new cameras we need a reference model.
You can find some models already built in **models** folder. Models have been generated with the script called **create_model**. This is an example script to create models, but anyone can build how it prefers.
The script has to create an object of class **ReferenceModel** which must have the reference image, parameters matrices (K, R, T), all the correspondences between 2D and 3D points and the sift descriptors of those 2D points.
The script **create_model** is not general purpose. It is working with some given data extracted from Zephyr. More in section [Code Explanation](#code-explanation).

## Working Pipeline
The entire pipeline is executed inside a function called **pose_estimator**.
This function requires:
- model, the reference model object
- checkImageFile, test image from where estimate
- method, one in MethodName enumaration (except for Lowe, which is currently not working properly)
- testK, internal params matrix of the test image

The method will use sift to match descriptors of new image with the model one.
Then use ransac to estimate the best model that fit with 2D-3D correspondences.
With inliers produced by ransac the exterior parameteres are then computed and all inliers are plotted on new image along with the reprojection using the estimated matrix.
It will return estimated rotation and translation matrices.

## Code explanation
### pose_estimator.m
First **vl_sift** is used to find descriptors on new image.
Then those descriptors are matched with the reference ones.
As the reference model have already all the points with their 3D correspondencies, once 2D-2D correspondences are computed between the two image, we immediately have also the 3D correspondences on the test image.

Once we have those we use ransac to find a model that best fits as much points as possible. 
A desired threshold is set as half the points. If we have enough inlier we proceed we the pose estimation with them, calling **compute_exterior** which will use the desired algorithm to perform the estimation.

### create_model.m
This script for now works with *Ca Vignal* building, of which we had available a structure containing points and camera parameters inside **cav/imgInfo.mat** referring to image **cav/cav.jpg**. 
Also it works with *Dante statue* with points extracted from Zephyr, where the dataset is taken from https://www.3dflow.net/it/3df-zephyr-vetrina-di-ricostruzioni/ . 
Extracting the right points from this is not straightforward. Inside **dante** folder there's a file called **Visibility.txt**, from that we have to cut out only the part interested for a specific picture and create a file with its id like **VisibilityRef1013.txt**. 
Once that's ready, in create_model you can set ```env='dante';``` and ```imageIndex='1013';```. In this way the model for that reference will be created inside **models** folder with a name like **refDescriptorsDante1013.mat**.

What happens in the code is: after getting the 2D-3D correspondences as described before, it has to find image descriptors using sift. A first selection from those points is performed using matlab function **dsearchn** which for each descriptors, gives the closest 2D point of our list, then it keeps the closer ones.   
Then we use our function **getRefDescriptors** to associate each descriptor to the closest point and get them in the right order.

## Useful methods
### plotOnImage
This method plots 2D points on image, and uses the given camera parameters to project the given 3D points on the image.

### plotCameraPose
Given rotation and translation matrices plot the camera position in 3D along with the frame orientation.

## Demos and results
The usage can be seen in scripts **cav_estimator** and **dante_estimator**.

Here we collect the best results with different poses and lighting conditions. All the camera poses estimation are given by the Fiore's algorithm. 

Regarding Ca Vignal dataset we have this reference image:

![Test Image 1](https://github.com/creminem94/ComputerVisionProject/blob/main/cav/cav.jpg)

These are the projections, where in red real 2D points and  in blue circles 2D projection points:

![Test Image 2](https://github.com/creminem94/ComputerVisionProject/blob/main/cav/result/cav_projection.png)

This is the camera pose estimation with respect to the cloud of points:

![Test Image 3](https://github.com/creminem94/ComputerVisionProject/blob/main/cav/result/cav_pose.png)

Regarding Dante dataset we have 3 reference images.

The first one is:

![Test Image 4](https://github.com/creminem94/ComputerVisionProject/blob/main/dante/test/1020/_SAM1020.JPG)

These are the projections, where in red real 2D points and  in blue circles 2D projection points:

![Test Image 5](https://github.com/creminem94/ComputerVisionProject/blob/main/dante/test/1020/result/dante_projection.png)

This is the camera pose estimation with respect to the cloud of points

![Test Image 6](https://github.com/creminem94/ComputerVisionProject/blob/main/dante/test/1020/result/dante_pos.png)

The second reference image is the following one:

![Test Image 4](https://github.com/creminem94/ComputerVisionProject/blob/main/dante/test/1048/_SAM1048.JPG)

These are the projections, where in red real 2D points and  in blue circles 2D projection points:

![Test Image 5](https://github.com/creminem94/ComputerVisionProject/blob/main/dante/test/1048/result/dante_projection.png)

This is the camera pose estimation with respect to the cloud of points:

![Test Image 6](https://github.com/creminem94/ComputerVisionProject/blob/main/dante/test/1048/result/dante_pose.png)

The last reference image is:

![Test Image 7](https://github.com/creminem94/ComputerVisionProject/blob/main/dante/test/1097/_SAM1097.JPG)

These are the projections, where in red real 2D points and  in blue circles 2D projection points:

![Test Image 8](https://github.com/creminem94/ComputerVisionProject/blob/main/dante/test/1097/result/dante_projection.png)

This is the camera pose estimation with respect to the cloud of points:

![Test Image 9](https://github.com/creminem94/ComputerVisionProject/blob/main/dante/test/1097/result/dante_pose.png)







