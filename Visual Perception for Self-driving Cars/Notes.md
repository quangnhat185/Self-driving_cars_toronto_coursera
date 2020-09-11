## Module 1: Basics of 3D Computer Vision
This module introduce the main concepts from the broad field of computer vision need to progress through perception methods for self-driving vehicles. The main components include camera models and their calibration, monocular and stereo vision, projective geometry, and covolution operations.
- **Camera Projective Geometry**

  3D points in the world coordinate frame can be projected to 2D points in the image coordinate frame using projective geometry equations.
  
  - **Projection from World coordinates to Image coordinates**:
    1. Projection from World coordinates to Camera coordinates.
    2. Project from Camera coordinates to Image coordinates.
    
    <p align="center"><img src="./img/computing_projection.jpg" width=640></img></p><br>

- **Camera calibration**
  
  - Use scenes with **known geometry** (e.g., checkerboard) to:
    - Correspond 2D image coordinates to 3D world coordinates
    - Find the **Least Squares Solution** (or non-linear solution) of the parameters of P
    - If we have N 3D points and their corresponding N 2D projects, set up homogeneous linear system.
    - Solve with Singular Value Decomposition (SVD) to get the least square solution
    - **Advantages**:
      - Easy to formulate.
      - Closed form solution.
      - Often provides really good initial points for nonlinear calibration approaches.
      
    - **Disadvantages**:
      - Does not directly provide camera parameters
      - Does not model radial distortion and other complex phenomena.
      - Does not allow for constraints such as known focal length to be imposed

- **Visual Depth Perception - Stereopsis**
  - Assumption:
    - Sensor is constructed from two identical cameras
    - The two cameras have parallel optical axes
    <p align="center"><img src="./img/computing_3D_points.jpg" width=640></img></p><br>
    
  
  - **Disparity**: the difference in image location of the same 3D point under perspective to two different cameras. 
    - To compute disparity, we need to be able to find the same point in the left and right stereo camera images.
    - We can constrain our correspondence search to be along the epipolar line, reducing the search from 2D to 1D (assuming optical axes of the two cameras are parallel).
    - We can use stereo rectification to wrap images originating from two cameras with non-parallel optical axes to force epipolar lines to be horizontal.
    
    <p align="center"><img src="./img/epiolar_line.jpg" width=640></img></p><br>
  
  - **A Basic Stereo Algorithm**
    - Given: Rectified Images and Stereo Calibration. For each epipolar line:
      1. Take a pixel on this line in the left image
      2. Compare these left image pixels to every pixel in the right image on the same epipolar line.
      3. Pick the pixel that has minimum cost.
      4. Compute disparity d by subtracting right pixel location from the left one
  
- **Image Filtering**  
    - **Convolution**: is a cross-correlation where the filter is flipped both horizontally and vertically before being applied to the image. 
    - **Template Matching**: is a problem where we are given a pattern or a template, and we want to find its location in the image. The pixel with the **highest response** from Cross-correlation is the location of the template in an image. 
    <p align="center"><img src="./img/template_matching.jpg" width=640></img></p><br>
    
    - **Gradient Computation**
      - Conolution can be used for Image gradient computation
      - Image gradients are extremely useful for detection of edges and corners.
      - Define a finite difference kernel, and apply it to the image to get the image gradient.
      <p align="center"><img src="./img/gradient_computation.jpg" width=640></img></p><br>
      
## Module 2: Visual Feature - Detection, Description and Matching
Visual features are used to track motion through an environment and to recognize places in a map. this module describes how features can be detected and tracked through a sequence of images and fused with other sources for localization as described in Course 2. Features extraction is also fundamental to object detection and semantic segmentation in deep networks, and this module introduces some of the feature detection methods employed in that context as well.

- **Feature Detection**:
  - Feature are **points of interest** in an image
  
  - **Points of interest** should have following characteristics:
    1. Saliency: distinctive, identifiable, and different from its immediate neighborhood
    2. Repeatability: can be found in multiple images using same operations
    3. Locality: occupies a relatively small subset of image space
    4. Quantity: enough points represented in the image
    5. Efficiency: reasonable computation time 
    
- **Feature Extraction**:
  - Repetitive texture less patches are challenging to detect consistently.
  - Patches with large contrast changes (gradients) are easier to detect (edges).
  - Gradients in at least two (significantly) different orientations are the easiest to detect. 

- **Feature Detection: Algorithm**
  - Harris {corners}: Easy to compute, but not scale invariant (1988). 
  - Harris-Laplace {corners}: Same produce as Harris detector, addtion of scale selection based on Lapacian. Scale invariance (2001)
  - Feature from accelerated segment test (FAST) {corners}: Machine learning approach for fast cornet detection (2006).
  - Laplacian of Gaussian (LOG) detector {blob}: Use the concept of scale space in a large neighborhood (blob). Somewhat scale invariant (1998).
  - Difference of Gaussian (DOG) detector (blobs): Approximate LOG but is faster to compute (2004)
  
- **Feature Descriptors**
    - **Feature:** Point of interest in an image defined by its image pixel coordinates **[u,v]**
    - **Descriptor**: An N-dimensional vector that provides a summary of the image information around the detected feature
    <p align="center"><img src="./img/feature_descriptor.jpg" width=640></img></p><br>
    
    - **Feature descriptors should have the following characteristics:**
      - **Repeatability**: manifested as **robustness and invariance** to translation, rotation, scale, and illumination changes
      - **Distinctiveness**: should allow us to distinguish between two close by features, very important for matching later on
      - **Compactness & Efficiency**: reasonable computation time

- **Designing Invariant Descriptors: SIFT**
  - Scale Invariant Feature Transform (SIFT) descriptors (1999):
    1. 16 x 16 window around detected feature
    2. Separate into 4 cells, each comprised of 4 x 4 path of pixels. 
    3. Compute edge orientation of each pixel in the cell.
    4. Suppress weak edges using a predefined threshold.
    5. Construct 32 dimensional histogram of orientations for each cell, then concatenate to get 128 dimensional descriptor.
    
    <p align="center"><img src="./img/SIFT.jpg" width=640></img></p><br>

  - The above system process is usually compute on rotated and scaled version of the 16 x 16 window, allowing for better scale robustness
  
  - Combined with the DOG feature detector, SIFT descriptors provide a scale, rotation and illumination invariant detector/descriptor pair
  
- **Feature matching**:
  - General process:
    1. Identify image features, distinctive point in our images.
    2. We associate a descriptor from each feature from its neighborhood.
    3. Use descriptors to match features across two or more images.
  
  - Brutal Force Feature Matching:
    - Define a distance function that compares the two descriptors
    <p align="center"><img src="./img/distance_function.jpg" width=640></img></p><br>
    
    <p align="center"><img src="./img/brutal_force.jpg" width=640></img></p><br>
      
  - Brutal force feature matching might not be fast enough for extremely large amounts of features

  - Ambiguous matches are features in the first image that have similar distance to two or more features in the second image. We can handle it by computing **distance ratio**, and making sure this ratio is lower than predefined threshold. 
  <p align="center"><img src="./img/distance_ratio.jpg" width=640></img></p><br>
  
- **Outlier Rejection**
  - **Image Features: Localization**
  <p align="center"><img src="./img/image_feature_localization.jpg" ></img></p><br>
  
  - **Ouliers** are wrong feature matching outputs, that can occur due to errors in any of the three stages of feature usage. 
  <p align="center"><img src="./img/outlier.jpg" ></img></p><br>
  
    - **Random Sample Consensus (RANSAC)**: One of the most used model-based methods for outlier rejection in robotics. 
    <p align="center"><img src="./img/RANSAC.jpg" ></img></p><br>
    

- **Visual Odometry**
  - Visual Odometry (VO) is the process of incrementally estimating the pose of the vehicle by examining the changes that motion induces on the images of its onboard cameras. 
  
  - **VO Pros**: 
    - No affected by wheel slip in uneven terrain, rainy/snowy weather or other adverse conditions. 
    - More accurate trajectory estimates compared to wheel odometry.
  
  - **VO Cons**:
    - Usually need an external sensor to estimate **absolute scale**.
    - Camera is passive sensor, might not be very robust against weather conditions and illumination changes.
    - Any form of odometry (incremental state estimate) drifts over time.
    
  <p align="center"><img src="./img/visual_odometry.jpg" ></img></p><br>
  
  - **Motion Estimation**:
    - Visual odometry can be performed using 2D-3D feature correspondences and the Pnp algorithm.
    <p align="center"><img src="./img/motion_estimation_1.jpg" ></img></p><br>
    
  
