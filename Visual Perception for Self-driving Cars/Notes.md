## Module 1: Basics of 3D Computer Vision

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
      
