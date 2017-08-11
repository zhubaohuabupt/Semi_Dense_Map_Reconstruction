
#include <stdexcept>
#include <point.h>
#include <frame.h>
#include <feature.h>
 
namespace DepthEstimation {

int Point::point_counter_ = 0;

Point::Point(const Vector3d& pos) :
  id_(point_counter_++),
  pos_(pos),
  normal_set_(false),
  type_(TYPE_UNKNOWN)
{}

Point::Point(const Vector3d& pos, Feature* ftr) :
  id_(point_counter_++),
  pos_(pos),
  normal_set_(false),
  type_(TYPE_UNKNOWN)
{
  obs_.push_front(ftr);
}

Point::~Point(){}

bool Point::getCloseViewObs(const Vector3d& framepos, Feature*& ftr) const
{
  // TODO: get frame with same point of view AND same pyramid level!
  Vector3d obs_dir(framepos - pos_); obs_dir.normalize();
  auto min_it=obs_.begin();
  double min_cos_angle = 0;
  for(auto it=obs_.begin(), ite=obs_.end(); it!=ite; ++it)
  {
    Vector3d dir((*it)->frame->pos() - pos_); dir.normalize();
    double cos_angle = obs_dir.dot(dir);
    if(cos_angle > min_cos_angle)
    {
      min_cos_angle = cos_angle;
      min_it = it;
    }
  }
  ftr = *min_it;
  if(min_cos_angle < 0.5) // assume that observations larger than 60° are useless
    return false;
  return true;
}

}
