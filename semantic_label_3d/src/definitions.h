#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// Settings
// Features to use
#define USE_CONTACT_FEATURES
//#define USE_VISIBILITY_FEATURES
#define USE_HISTOGRAM_FEATURES

#define THRESHOLD_FALLING_DIST 20
#define N_CONTINUOUS_PTS 1
#define EPS_INTERSECT 5
#define EPS_FLOAT 1e-6
#define EPS_DXDY 5 
#define EPS_GRID 50
#define EPS_SAMPLE_Z 5
#define EPS_INTERVAL_Z 5
#define MIN_GRID_POINTS 5
#define MINX 200
#define MAXX 1100
#define MINY -500
#define MAXY 500
#define N_ENV_SAMPLES 100
#define N_OBJ_SAMPLES 50
#define N_OBJ_CONFIGS 24 
//#define CV_LEARNING_ALGO CvNormalBayesClassifier 
#define CV_LEARNING_ALGO CvBoost 
#define USE_OPENCV_ML

// Common helpers
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define PI (3.141592653589793)
#define sqr(x) ((x)*(x))
#define MIN(x,y) ( (x)<(y) ? (x) : (y) )
#define MAX(x,y) ( (x)>(y) ? (x) : (y) )
//#define ROS_DEBUG printf 

#define DEBUG_CONTACT_FEATURE

#endif // DEFINITIONS_H
