%DRAW_GRAPHICS  Draw the state of the EKF-SLAM
%
%   DRAW_GRAPHICS(X, P, R, M) will draw the current state of the estimator.
%   X and P are respectively the state vector and associated covariance
%   matrix. R is the pointer to the spaces in the estimator that contain
%   the robot variables, and finally M is the pointer to all the landmarks
%   known by the estimator (that is, the map). Covariances in P are
%   represented by ellipses in the figure.
%
%   This function will also draw the position of the robot and of all
%   landmarks (mapped and non-mapped).
%
%   This function should be called once at the end of each iteration of the
%   temporal loop in SLAM2D_BE. A call to this function is already placed
%   in the good position. Without this call you won't be able to see what's
%   going on inside the estimator!
%
%   See also init_graphics.
%
