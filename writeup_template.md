## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---
[image1]: ./misc_images/noisy_ptcloud.png
[image2]: ./misc_images/stat_inliers.png
[image3]: ./misc_images/stat_outliers.png
[image4]: ./misc_images/world_1.png
[image5]: ./misc_images/world_2.png
[image6]: ./misc_images/world_3.png
[image7]: ./misc_images/reg_confusion_mtx.png
[image8]: ./misc_images/normalized_confusion_mtx.png

# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

### Helper Functions from features.py
Compute Color Histograms/Compute Normal Histograms
This function steps through each point in a pointcloud. First it converts the float type pointcloud values to RGB and then to HSV using helper functions. Using the HSV color space improves performance in low-light conditions. The function then populates these lists with color values before dividing up each of the color channels into 32 bins in the range (0, 256). These histograms are concatenated together and normalized before being returned. Compute normal histograms goes through a similar process to return a histogram of 'normals', features to provide shape information.

### pcl_callback 
Stepping through the pcl_callback function, which takes in a pcl_msg from the Point Cloud Subscriber, we first convert that ROS message to pcl with the ros_to_pcl helper function. This gives us a point cloud to work with.

### Statistical Outlier Filter
Since there is noise in the data, a statistical outlier filter is applied. This assumes a normal distribution, compares neighboring points to the mean and classifies those points lying outside a certain set-threshold (like 1 to 2 standard deviations from the mean) as outliers and discards them. This process cuts down on noise quite well if you choose the size of the 'neighborhood' correctly (meaning how many pixels are compared at a time) and also the distance from the mean chosen to define 'outlier' territory.

## Noisy Pointcloud
![Noisy Pointcloud][image1]

## Inliers
![Statistical Outliers][image2]

## Outliers
![Statistical Inliers][image3]

### Voxel Downsampling
Next voxel grid downsampling is performed. This is simple but cool. RGB-D cameras provide data-dense input representations. We need to cut down on that for processing reasons. So you define a box of a certain size in the 3-D world of pixels in the input space (a voxel is a volumetric pixel). Find the center. Take the average value over all the pixels inside the box. Assign it to the center and discard the rest. (The size/volume of the box matters). You've dramatically cut down on the amount of data you are dealing with at the cost of resolution. This is a trade-off that when balanced properly leads to a recognizable and accurate pointcloud representation of an object that isn't as data dense as it used to be. 

### Passthrough Filter
A passthrough filter is applied next. This acts as a cropping tool to cut off pixels beyond a given setting in the x, y, z space. For instance, if you know the height of the table that is in front of your robot (the top of which is making up it's workspace) you can discard pixels that come from below the height of the table (in the 'z' direction) over a specified range. This effectively eliminates everything below the table from the picture.

### RANSAC Filter
This leaves the pixels forming the tabletop and the objects sitting on it. To separate these from each other, RANSAC plane filtering is used. Random Sample Consensus algorithm looks for the components of geometric shapes (two points make a line, three non-colinear points make a plane) in order to find the 'plane' shape of the table and divide the pixels into ones that fit the plane model (inliers) and ones that do not (outliers). These extracted outliers comprise our isolated objects of interest.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

### Limitations of RANSAC
The downside of RANSAC model fitting is that you have to compute it over the entire pointcloud once per object in order to filter out all the other objects and leave just one object of interest. This is not optimal. Also many objects in the scene look similar to each other geometrically (cylindrical objects like cups or cans) and therefore would be mistaken for each other by RANSAC. So we use other approaches to define our objects of interest. Broadly these are defined as Clustering Algorithms.

### K-Means Clustering
Visualizing a graph with the data spread out over it in scatterplot fashion, we first randomly assign k-means or 'centroids' where data will cluster. The distance between each point and each centroid is measured and each point is assigned to it's nearest centroid. The new center of mass of each new group is calculated and the centroid's position is reassigned here. The cycle then repeats, ending at a specified number of iterations or when a pre-defined minimum of movement is found following centroid position re-calculation.
K-Means clustering is strong when the anticipated number of clusters is known, as in our situation. When the number of clusters is unknown, the shape of the data clusters is complex or when data clusters are highly overlapping this is not a good method. Our clusters are of a known number, have fairly well defined, distinct shapes and are mostly non-overlapping. 

### DBSCAN
Density Based Spatial Clustering for Applications with Noise is strong when something is known concerning the expected density of clusters but the number of clusters is unknown. This algorithm defines clusters as those points lying within a certain minimum euclidean distance (in any dimension mind you: color space, temperature, texture, any other feature space) from the next nearest point. It's like each point gets it's own little shell of diameter x and any other point touching it's shell is considered part of it's group. And then all of that points' neighbors and so on, starting over from a new point at random only after all in-contact neighbors have been added to the group. This prevents many redundant neighbor visits and a complicated merge problem. Constraints to be defined include the minimum number of samples to be considered a group and the threshold distance used to define what is a neighbor. It is a good algorithm when outliers need to be exluded. Also there is no need to define convergence/termination.

### K-D Tree
In our implementation, a nearest-neighbor search algorithm called k-d tree is used. This requires x,y,z data only, no color. So a white-cloud is formed using a xyzrgb_to_xyz helper function. Clusters are found and extracted after min/max tolerances and cluster size are set. Colors are assigned to each cluster to distinguish them from one another. We have segmented our data into individual objects. For each point-cloud object we have RGB color data as well as 3-D spatial information. Now we need to identify what those objects are.

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.


### Support Vector Machine
Our classifier works based on the features of the data identified in the HSV color space and by the shape given by the surface normals. By 'binning up' our color data into color histogram bins, we can obtain a unique histogram signature of each object to learn from. Similarly, the distribution of surface normals contains shape information for each object that can be learned from. A support vector machine is used to characterize the parameter space into distinct classes. This algorithm draws decision boundaries around data groups, evaluates the level of success of these boundaries compared to a ground-truth label (requires labeled data for training) and then iteratively improves on the best location for the boundaries. After the feature space has been subdivided in this way, new unlabeled data can be categorized simply via reference to where it falls in the space due to its features. In other words it can be used for object recognition. 

The confusion matrix below demonstrates the accuracy of recognition across every type of object trained. An overall accuracy of 89% was attained after 200 training iterations on each object. Higher accuracy scores could be obtained with more iterations but on such a relatively shallow data set my deep-learning nanodegree instincts made me fear over-fit. Testing this robot's object recognition on un-known objects is not part of the challenge, however. So perhaps over-fit may not be a bad thing in this situation.

## Normalized Confusion Matrix
![Normalized][image8]

# Non-normalized Confusion Matrix
![Non-normalized][image7]

The basic flow of object recognition is implemented in the following manner:
Instantiate two empty lists: one for detected objects and one for labels.
Loop over the list of euclidean-clustered objects, extracting those points which correspond to our extracted_outliers. Convert to ROS from PCL. Compute associated feature vectors using compute_color_histograms and compute_normal_histograms functions. Normalize and concatenate these into a feature vector. Use the SVM boundaries to predict the identity of the object based on it's feature vector. Assign a label based on the prediction. Add the labeled object to the detected object list. Publish the detected objects list. Profit.

## World 1
![World-1][image4]

## World 2
![World-2][image5]

## World 3
![World-3][image6]

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.


### pr2_mover
After the pcl_callback function comes the pr2_mover function. This function loads the parameters with regard to the scene/object data and requests a pick/place service. 
A number of variables are initialized including the object name, group, pick_pose, place_pose and the arm needed to grab the object. The centroid of each object is calculated as the mean of it's numpy points array. This will be used later to define the pick_pose. The pick list is then parsed and looped over in order to assign appropriate data to the variables we defined above for each object. The pick_list object name data field is compared to the label of a point cloud object and when they match, the scalar-version of the centroid data is assigned as that object's pick-pose data. The place pose is defined based on the object's group. Arm data is assigned accordingly. This data is then written out in yaml dictionary format for a pick_place routine to use in directing robot motion.

### Going Forward
To improve on this project I would like to complete the challenge exercises so that the pick_place procedure can be carried out.


