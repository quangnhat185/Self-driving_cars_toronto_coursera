from numpy import *

def sph_to_cart(epsilon, alpha, r):
  """
  Transform sensor readings to Cartesian coordinates in the sensor
  frame. The values of epsilon and alpha are given in radians, while 
  r is in metres. Epsilon is the elevation angle and alpha is the
  azimuth angle (i.e., in the x,y plane).
  """
  p = zeros(3)  # Position vector 
  
  p[0] = r * cos(alpha) * cos(epsilon)
  p[1] = r * sin(alpha) * cos(epsilon)
  p[2] = r * sin(epsilon)
  
  return p
  
def estimate_params(P):
  """
  Estimate parameters from sensor readings in the Cartesian frame.
  Each row in the P matrix contains a single 3D point measurement;
  the matrix P has size n x 3 (for n points). The format is:
  
  P = [[x1, y1, z1],
       [x2, x2, z2], ...]
       
  where all coordinate values are in metres. Three parameters are
  required to fit the plane, a, b, and c, according to the equation
  
  z = a + bx + cy
  
  The function should retrn the parameters as a NumPy array of size
  three, in the order [a, b, c].
  """
  param_est = zeros(3)
  
  one_mat = mat(ones(len(P))).T
  x = mat(P[:, 0]).T
  y = mat(P[:, 1]).T
  z = mat(P[:, 2]).T
  
  B = z
  A = hstack((one_mat, x, y))
  params = linalg.inv(A.T.dot(A)).dot(A.T).dot(B)
  
  param_est[0] = params[0,0]
  param_est[1] = params[1,0]
  param_est[2] = params[2,0]
  
  
  return param_est

if __name__=="__main__":
    P = random.rand(20,3)
    q = estimate_params(P)
    print(q)
