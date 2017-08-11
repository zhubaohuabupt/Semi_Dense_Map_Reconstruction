#ifdef __ARM_NEON__
#include "SSE_to_NEON/sse_to_neon.hpp"
#else
#include <emmintrin.h>
#endif
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include "precomp.hpp"
#define CV_SSE2 1
using namespace cv;
void FAST_( InputArray image, CV_OUT vector<KeyPoint>& keypoints,int threshold, bool nonmaxSupression=true );
