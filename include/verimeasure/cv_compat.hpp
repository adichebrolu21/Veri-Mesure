// One include for everything this project needs out of OpenCV's image
// processing and geometry surface.
//
// OpenCV 5 reorganised its modules. Shape analysis and the homography solvers
// left imgproc and calib3d for a new "geometry" module:
//
//     contourArea, arcLength, boundingRect, minAreaRect, moments,
//     convexHull, getRotationMatrix2D, invertAffineTransform,
//     findHomography
//
// OpenCV 4 keeps them where they always were. Rather than scatter version
// checks through the sources, every file includes this and gets the right set
// either way. There is no behavioural difference: the functions are identical,
// only their declaring header changed.
//
// __has_include is checked as well as the version macro, because a "world"
// build or a partial install can report version 5 without shipping every
// module header.

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>

#if defined(CV_VERSION_MAJOR) && CV_VERSION_MAJOR >= 5
  #if defined(__has_include)
    #if __has_include(<opencv2/geometry.hpp>)
      #include <opencv2/geometry.hpp>
    #elif __has_include(<opencv2/geometry/2d.hpp>)
      #include <opencv2/geometry/2d.hpp>
      #include <opencv2/geometry/3d.hpp>
    #else
      #error "OpenCV 5 detected but the geometry module headers are missing. Install the geometry module, or build against OpenCV 4."
    #endif
  #else
    #include <opencv2/geometry.hpp>
  #endif
#else
  #include <opencv2/calib3d.hpp>
#endif
