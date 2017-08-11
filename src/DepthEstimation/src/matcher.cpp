
#include <cstdlib>
#include <vikit/abstract_camera.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>
#include <vikit/patch_score.h>
#include <matcher.h>
#include <frame.h>
#include <feature.h>
#include <point.h>
#include <config.h>
#include <feature_alignment.h>
#include <fstream>
 std::ofstream tau("/home/baohua/tau.txt");
  std::ofstream Costzmsad("/home/baohua/Costzmsad.txt");
 static int ii=0;
 ///显示极线匹配点对
#define ShowMatchByEpi 0
 ///显示极线方向的梯度
#define ShowGradAlongEpiloar 0

namespace DepthEstimation {

namespace warp {

void getWarpMatrixAffine(
    const vk::AbstractCamera& cam_ref,
    const vk::AbstractCamera& cam_cur,
    const Vector2d& px_ref,
    const Vector3d& f_ref,
    const double depth_ref,
    const SE3& T_cur_ref,
    const int level_ref,
    Matrix2d& A_cur_ref)
{
  // Compute affine warp matrix A_ref_cur
  const int halfpatch_size = 5;
  const Vector3d xyz_ref(f_ref*depth_ref);
//cout<<"px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)"<<px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)<<endl;
//  cout<<"px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)"<<px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)<<endl;
//cout<<"cam_ref.fx_"<<cam_ref.width()<<endl;
  Vector3d xyz_du_ref(cam_ref.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)));
  Vector3d xyz_dv_ref(cam_ref.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)));
 // cout<<"xyz_du_ref"<<xyz_du_ref<<endl;cout<<"xyz_dv_ref"<<xyz_dv_ref<<endl;
  //cout<<"xyz_ref[2]/xyz_du_ref[2]"<<xyz_ref[2]/xyz_du_ref[2]<<endl;
  xyz_du_ref *= xyz_ref[2]/xyz_du_ref[2];
  xyz_dv_ref *= xyz_ref[2]/xyz_dv_ref[2];

  const Vector2d px_cur(cam_cur.world2cam(T_cur_ref*(xyz_ref)));
  const Vector2d px_du(cam_cur.world2cam(T_cur_ref*(xyz_du_ref)));
  const Vector2d px_dv(cam_cur.world2cam(T_cur_ref*(xyz_dv_ref)));
  //  cout<<"T_cur_ref*(xyz_ref)"<<T_cur_ref*(xyz_ref)<<endl;  cout<<"T_cur_ref*(xyz_du_ref)"<<T_cur_ref*(xyz_du_ref)<<endl;cout<<"T_cur_ref*(xyz_dv_ref)"<<T_cur_ref*(xyz_dv_ref)<<endl;
  A_cur_ref.col(0) = (px_du - px_cur)/halfpatch_size;
  A_cur_ref.col(1) = (px_dv - px_cur)/halfpatch_size;
}

int getBestSearchLevel(
    const Matrix2d& A_cur_ref,
    const int max_level)
{
  // Compute patch level in other image
  int search_level = 0;
  double D = A_cur_ref.determinant();
  while(D > 3.0 && search_level < max_level)
  {
    search_level += 1;
    D *= 0.25;
  }
  return search_level;
}

void warpAffine(
    const Matrix2d& A_cur_ref,
    const cv::Mat& img_ref,
    const Vector2d& px_ref,
    const int level_ref,
    const int search_level,
    const int halfpatch_size,
    uint8_t* patch)
{
  const int patch_size = halfpatch_size*2 ;
  const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
  if(isnan(A_ref_cur(0,0)))
  {
    printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
    return;
  }

  // Perform the warp on a larger patch.
  uint8_t* patch_ptr = patch;
  const Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref);
  for (int y=0; y<patch_size; ++y)
  {
    for (int x=0; x<patch_size; ++x, ++patch_ptr)
    {
      Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
      px_patch *= (1<<search_level);
      const Vector2f px(A_ref_cur*px_patch + px_ref_pyr);
      if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
        *patch_ptr = 0;
      else
        *patch_ptr = (uint8_t) vk::interpolateMat_8u(img_ref, px[0], px[1]);
    }
  }
}

} // namespace warp

bool depthFromTriangulation(
    const SE3& T_search_ref,
    const Vector3d& f_ref,
    const Vector3d& f_cur,
    double& depth)
{
  Matrix<double,3,2> A; A << T_search_ref.rotation_matrix() * f_ref, f_cur;
  const Matrix2d AtA = A.transpose()*A;
  if(AtA.determinant() < 0.000001)
    return false;
  const Vector2d depth2 = - AtA.inverse()*A.transpose()*T_search_ref.translation();
  depth = fabs(depth2[0]);
  return true;
}

void Matcher::createPatchFromPatchWithBorder()
{
  uint8_t* ref_patch_ptr = patch_;
  for(int y=1; y<patch_size_+1; ++y, ref_patch_ptr += patch_size_)
  {
    uint8_t* ref_patch_border_ptr = patch_with_border_ + y*(patch_size_+2) + 1;
    for(int x=0; x<patch_size_; ++x)
      ref_patch_ptr[x] = ref_patch_border_ptr[x];
  }
}

bool Matcher::findMatchDirect(
    const Point& pt,
    const Frame& cur_frame,
    Vector2d& px_cur)
{
  if(!pt.getCloseViewObs(cur_frame.pos(), ref_ftr_))
    return false;

  if(!ref_ftr_->frame->cam_->isInFrame(
      ref_ftr_->px.cast<int>()/(1<<ref_ftr_->level), halfpatch_size_+2, ref_ftr_->level))
    return false;

  // warp affine
  warp::getWarpMatrixAffine(
      *ref_ftr_->frame->cam_, *cur_frame.cam_, ref_ftr_->px, ref_ftr_->f,
      (ref_ftr_->frame->pos() - pt.pos_).norm(),
      cur_frame.T_f_w_ * ref_ftr_->frame->T_f_w_.inverse(), ref_ftr_->level, A_cur_ref_);
  search_level_ = warp::getBestSearchLevel(A_cur_ref_, Config::nPyrLevels()-1);
  warp::warpAffine(A_cur_ref_, ref_ftr_->frame->img_pyr_[ref_ftr_->level], ref_ftr_->px,
                   ref_ftr_->level, search_level_, halfpatch_size_+1, patch_with_border_);
  createPatchFromPatchWithBorder();

  // px_cur should be set
  Vector2d px_scaled(px_cur/(1<<search_level_));

  bool success = false;
  if(ref_ftr_->type == Feature::EDGELET)
  {
    Vector2d dir_cur(A_cur_ref_*ref_ftr_->grad);
    dir_cur.normalize();
    success = feature_alignment::align1D(
          cur_frame.img_pyr_[search_level_], dir_cur.cast<float>(),
          patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
  }
  else
  {
    success = feature_alignment::align2D(
      cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
      options_.align_max_iter, px_scaled);
  }
  px_cur = px_scaled * (1<<search_level_);
  return success;
}

bool Matcher::findEpipolarMatchDirect(
    const Frame& ref_frame,
    const Frame& cur_frame,
    const Feature& ref_ftr,
    const double d_estimate,
    const double d_min,
    const double d_max,
    double& depth)
{
  SE3 T_cur_ref = cur_frame.T_f_w_ * ref_frame.T_f_w_.inverse();

  int zmssd_best = PatchScore::threshold();
  Vector2d uv_best;
//cout<<"a"<<endl;
  // Compute start and end of epipolar line in old_kf for match search, on unit plane!
  Vector2d A = vk::project2d(T_cur_ref * (ref_ftr.f*d_min));
  Vector2d B = vk::project2d(T_cur_ref * (ref_ftr.f*d_max));
  epi_dir_ = A - B;
//cout<<"b"<<endl;

//cout<<"ref_ftr.px: "<<ref_ftr.px<<endl;
//cout<<"ref_ftr.f : "<<ref_ftr.f<<endl;
//cout<<"ref_ftr.level : "<<ref_ftr.level<<endl;
  // Compute affine warp matrix
  warp::getWarpMatrixAffine(
      *ref_frame.cam_, *cur_frame.cam_, ref_ftr.px, ref_ftr.f,
      d_estimate, T_cur_ref, ref_ftr.level, A_cur_ref_);
//cout<<"c"<<endl;
  // feature pre-selection
  reject_ = false;
  if(ref_ftr.type == Feature::EDGELET && options_.epi_search_edgelet_filtering)
  {
    const Vector2d grad_cur = (A_cur_ref_ * ref_ftr.grad).normalized();
    const double cosangle = fabs(grad_cur.dot(epi_dir_.normalized()));
    if(cosangle < options_.epi_search_edgelet_max_angle) {
      reject_ = true;
      return false;
    }
  }
//cout<<"111"<<endl;
  search_level_ = warp::getBestSearchLevel(A_cur_ref_, Config::nPyrLevels()-1);

  // Find length of search range on epipolar line
  Vector2d px_A(cur_frame.cam_->world2cam(A));
  Vector2d px_B(cur_frame.cam_->world2cam(B));
  epi_length_ = (px_A-px_B).norm() / (1<<search_level_);
//cout<<"111"<<endl;
  // Warp reference patch at ref_level
  warp::warpAffine(A_cur_ref_, ref_frame.img_pyr_[ref_ftr.level], ref_ftr.px,
                   ref_ftr.level, search_level_, halfpatch_size_+1, patch_with_border_);
//cout<<"222"<<endl;
  createPatchFromPatchWithBorder();

  if(epi_length_ < 2.0)
  {
    px_cur_ = (px_A+px_B)/2.0;
    Vector2d px_scaled(px_cur_/(1<<search_level_));
    bool res;
    if(options_.align_1d)
      res = feature_alignment::align1D(
          cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
          patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
    else
      res = feature_alignment::align2D(
          cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
          options_.align_max_iter, px_scaled);
//cout<<"333"<<endl;
    if(res)
    {
      px_cur_ = px_scaled*(1<<search_level_);
if (ShowMatchByEpi)     ////show the searching epiplor and the matching result
      showPointMatch(ref_frame.img_pyr_[0],ref_ftr.px,cur_frame.img_pyr_[0], px_cur_,px_A,px_B);//////////////////////////////////////////////////////

      if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth))
        return true;
    }
    return false;
  }
//cout<<"4444"<<endl;
  size_t n_steps = epi_length_/0.7; // one step per pixel
  Vector2d step = epi_dir_/n_steps;

  if(n_steps > options_.max_epi_search_steps)
  {
    printf("WARNING: skip epipolar search: %zu evaluations, px_lenght=%f, d_min=%f, d_max=%f.\n",
           n_steps, epi_length_, d_min, d_max);
    return false;
  }

  // for matching, precompute sum and sum2 of warped reference patch
  int pixel_sum = 0;
  int pixel_sum_square = 0;
  PatchScore patch_score(patch_);

  // now we sample along the epipolar line
  Vector2d uv = B-step;
  Vector2i last_checked_pxi(0,0);
  ++n_steps;
  for(size_t i=0; i<n_steps; ++i, uv+=step)
  {
    Vector2d px(cur_frame.cam_->world2cam(uv));
    Vector2i pxi(px[0]/(1<<search_level_)+0.5,
                 px[1]/(1<<search_level_)+0.5); // +0.5 to round to closest int

    if(pxi == last_checked_pxi)
      continue;
    last_checked_pxi = pxi;

    // check if the patch is full within the new frame
    if(!cur_frame.cam_->isInFrame(pxi, patch_size_, search_level_))
      continue;

    // TODO interpolation would probably be a good idea
    uint8_t* cur_patch_ptr = cur_frame.img_pyr_[search_level_].data
                             + (pxi[1]-halfpatch_size_)*cur_frame.img_pyr_[search_level_].cols
                             + (pxi[0]-halfpatch_size_);
    int zmssd = patch_score.computeScore(cur_patch_ptr, cur_frame.img_pyr_[search_level_].cols);

    if(zmssd < zmssd_best) {
      zmssd_best = zmssd;
      uv_best = uv;
    }
  }
  if(zmssd_best < PatchScore::threshold())
  {
    if(options_.subpix_refinement)
    {
      px_cur_ = cur_frame.cam_->world2cam(uv_best);
      Vector2d px_scaled(px_cur_/(1<<search_level_));
      bool res;
      if(options_.align_1d)
        res = feature_alignment::align1D(
            cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
            patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
      else
        res = feature_alignment::align2D(
            cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
            options_.align_max_iter, px_scaled);
      if(res)
      {
        px_cur_ = px_scaled*(1<<search_level_);
if( ShowMatchByEpi) ////show the searching epiplor and the matching result
        showPointMatch(ref_frame.img_pyr_[0],ref_ftr.px,cur_frame.img_pyr_[0], px_cur_,px_A,px_B);/////////////////////////////////////////////

        if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth))
          return true;
      }
      return false;
    }
    px_cur_ = cur_frame.cam_->world2cam(uv_best);
    if(depthFromTriangulation(T_cur_ref, ref_ftr.f, vk::unproject2d(uv_best).normalized(), depth))
      return true;
  }
  return false;
}
  ///we Calculate the InitTau refering to LSD
////////////////////////////////////////////////////////////同时求出极线和光学不确定性
bool Matcher::findEpipolarMatchDirectWithInitTau(
    const Frame& ref_frame,
    const Frame& cur_frame,
    const Feature& ref_ftr,
    const double d_estimate,
    const double d_min,
    const double d_max,
    double& depth,double &InitTau)
{
ii++;
  SE3 T_cur_ref = cur_frame.T_f_w_ * ref_frame.T_f_w_.inverse();

  float zmssd_best = PatchScore::threshold();
  Vector2d uv_best;
  // Compute start and end of epipolar line in old_kf for match search, on unit plane!
  Vector2d A = vk::project2d(T_cur_ref * (ref_ftr.f*d_min));
  Vector2d B = vk::project2d(T_cur_ref * (ref_ftr.f*d_max));
  epi_dir_ = A - B;
  // Compute affine warp matrix
  warp::getWarpMatrixAffine(
      *ref_frame.cam_, *cur_frame.cam_, ref_ftr.px, ref_ftr.f,
      d_estimate, T_cur_ref, ref_ftr.level, A_cur_ref_);

  // feature pre-selection
  reject_ = false;
  if(ref_ftr.type == Feature::EDGELET && options_.epi_search_edgelet_filtering)
  {
    const Vector2d grad_cur = (A_cur_ref_ * ref_ftr.grad).normalized();
    const double cosangle = fabs(grad_cur.dot(epi_dir_.normalized()));
    if(cosangle < options_.epi_search_edgelet_max_angle) {
      reject_ = true;
      return false;
    }
  }

  search_level_ = warp::getBestSearchLevel(A_cur_ref_, Config::nPyrLevels()-1);
  // Find length of search range on epipolar line
  Vector2d px_A(cur_frame.cam_->world2cam(A));
  Vector2d px_B(cur_frame.cam_->world2cam(B));
  epi_length_ = (px_A-px_B).norm() / (1<<search_level_);
  // Warp reference patch at ref_level
  warp::warpAffine(A_cur_ref_, ref_frame.img_pyr_[ref_ftr.level], ref_ftr.px,
                   ref_ftr.level, search_level_, halfpatch_size_+1, patch_with_border_);
  createPatchFromPatchWithBorder();
   Vector2d epi_dir_unit ( (px_A-px_B)/(px_A-px_B).norm() );
  if(epi_length_ < 2.0)
  {
    px_cur_ = (px_A+px_B)/2.0;
    Vector2d px_scaled(px_cur_/(1<<search_level_));
    bool res;
    if(options_.align_1d)
      res = feature_alignment::align1D(
          cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
          patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
    else
      res = feature_alignment::align2D(
          cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
          options_.align_max_iter, px_scaled);
    if(res)
    {
      px_cur_ = px_scaled*(1<<search_level_);
if (ShowMatchByEpi)     ////show the searching epiplor and the matching result
      showPointMatch(ref_frame.img_pyr_[0],ref_ftr.px,cur_frame.img_pyr_[0], px_cur_,px_A,px_B);//////////////////////////////////////////////////////
 InitTau=computePhotometricError(epi_dir_unit, px_cur_,cur_frame);//////////////////////////////////////////////光学不确定性
      if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth))
        return true;
    }
    return false;
  }
  size_t n_steps = epi_length_/0.7; // one step per pixel
  Vector2d step = epi_dir_/n_steps;

  if(n_steps > options_.max_epi_search_steps)
  {
    printf("WARNING: skip epipolar search: %zu evaluations, px_lenght=%f, d_min=%f, d_max=%f.\n",
           n_steps, epi_length_, d_min, d_max);
    return false;
  }

  // for matching, precompute sum and sum2 of warped reference patch
  int pixel_sum = 0;
  int pixel_sum_square = 0;
  PatchScore patch_score(patch_);

  // now we sample along the epipolar line
  Vector2d uv = B-step;
  Vector2i last_checked_pxi(0,0);
  ++n_steps;

  for(size_t i=0; i<n_steps; ++i, uv+=step)
  {
    Vector2d px(cur_frame.cam_->world2cam(uv));
    Vector2i pxi(px[0]/(1<<search_level_)+0.5,
                 px[1]/(1<<search_level_)+0.5); // +0.5 to round to closest int

    if(pxi == last_checked_pxi)
      continue;
    last_checked_pxi = pxi;

    // check if the patch is full within the new frame
    if(!cur_frame.cam_->isInFrame(pxi, patch_size_, search_level_))
      continue;

    // TODO interpolation would probably be a good idea
    uint8_t* cur_patch_ptr = cur_frame.img_pyr_[search_level_].data
                             + (pxi[1]-halfpatch_size_)*cur_frame.img_pyr_[search_level_].cols
                             + (pxi[0]-halfpatch_size_);
    float zmssd = patch_score.computeScore(cur_patch_ptr, cur_frame.img_pyr_[search_level_].cols);

                                                                                                                              //     cost=   zmsad;
                                                                                                                             //      zmsad->calCost(cur_patch_ptr,patch_score.ref_patch_,patch_size_*patch_size_/2);
Costzmsad<<zmssd<<endl;
     if(zmssd < zmssd_best) {
      zmssd_best = zmssd;
      uv_best = uv;
    }

  }
  if(zmssd_best < PatchScore::threshold())
  {
    if(options_.subpix_refinement)
    {
      px_cur_ = cur_frame.cam_->world2cam(uv_best);
      Vector2d px_scaled(px_cur_/(1<<search_level_));
      bool res;
      if(options_.align_1d)
        res = feature_alignment::align1D(
            cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
            patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
      else
        res = feature_alignment::align2D(
            cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
            options_.align_max_iter, px_scaled);
      if(res)
      {
        px_cur_ = px_scaled*(1<<search_level_);
if( ShowMatchByEpi) ////show the searching epiplor and the matching result
        showPointMatch(ref_frame.img_pyr_[0],ref_ftr.px,cur_frame.img_pyr_[0], px_cur_,px_A,px_B);/////////////////////////////////////////////
 InitTau=computePhotometricError(epi_dir_unit, px_cur_,cur_frame) ;//////////////////////////////////////////////光学不确定性
        if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth))
          return true;
      }
      return false;
    }
    px_cur_ = cur_frame.cam_->world2cam(uv_best);
    if(depthFromTriangulation(T_cur_ref, ref_ftr.f, vk::unproject2d(uv_best).normalized(), depth))
      return true;
  }
  return false;
}
 double Matcher::computePhotometricError(const Vector2d&epi_dir_unit, const Vector2d&px_cur_,const Frame& cur_frame)
 {

   double segma2photo=64,grad2AloneEpipolar=-1;
   Vector2d close(0.5,0.5);

     Vector2d px_cur_befor2(px_cur_-3.2*epi_dir_unit+close);                                                                                                                                                                // cout<<"epi_dir_unit  "<<epi_dir_unit<<endl;
     Vector2d px_cur_befor1(px_cur_-1.6*epi_dir_unit+close);
     Vector2d    px_cur(px_cur_+close);
    Vector2d    px_cur_after1(px_cur_+1.6*epi_dir_unit+close);
     Vector2d    px_cur_after2(px_cur_+3.2*epi_dir_unit+close);
                                                                                                                            if  ( ShowGradAlongEpiloar )
                                                                                                                                     {
                                                                                                                                         cv::Mat show=cur_frame.img_pyr_[0].clone();
                                                                                                                                         cv::cvtColor(show,show,CV_GRAY2BGR);
                                                                                                                                                    cv::line(show,cv::Point(   (int)px_cur_befor2[0], (int)px_cur_befor2[1]),cv::Point( (int)px_cur_befor1[0], (int)px_cur_befor1[1]),cv::Scalar(255,0,0));
                                                                                                                                                    cv::line(show,cv::Point( (int)px_cur_befor1[0], (int)px_cur_befor1[1]),cv::Point( (int)px_cur[0], (int)px_cur[1]),cv::Scalar(255,0,0));
                                                                                                                                                    cv::line(show,cv::Point( (int)px_cur[0], (int)px_cur[1]),cv::Point( (int)px_cur_after1[0], (int)px_cur_after1[1]),cv::Scalar(255,0,0));
                                                                                                                                                    cv::line(show,cv::Point( (int)px_cur_after1[0], (int)px_cur_after1[1]),cv::Point( (int)px_cur_after2[0], (int)px_cur_after2[1]),cv::Scalar(255,0,0));
                                                                                                                                                    cv::imshow("  ",show);
                                                                                                                                                    cv::waitKey(10);
                                                                                                                                        }

      uchar   px_cur_befor2_Color=cur_frame.img_pyr_[0].at<uchar>((uchar)px_cur_befor2[1],(uchar)px_cur_befor2[0]);
      uchar   px_cur_befor1_Color=cur_frame.img_pyr_[0].at<uchar>((uchar)px_cur_befor1[1],(uchar)px_cur_befor1[0]);
      uchar   px_cur_Color=cur_frame.img_pyr_[0].at<uchar>((uchar)px_cur[1],(uchar)px_cur[0]);
       uchar  px_cur_after1_Color=cur_frame.img_pyr_[0].at<uchar>((uchar)px_cur_after1[1],(uchar)px_cur_after1[0]);
     uchar  px_cur_after2_Color=cur_frame.img_pyr_[0].at<uchar>((uchar)px_cur_after2[1],(uchar)px_cur_after2[0]);
//calculate the gradient along Epipolar
       grad2AloneEpipolar=( px_cur_befor2_Color-px_cur_befor1_Color)*( px_cur_befor2_Color-px_cur_befor1_Color)+
                                            (px_cur_befor1_Color-px_cur_Color)*(px_cur_befor1_Color-px_cur_Color)+
                                            (px_cur_Color-px_cur_after1_Color)*(px_cur_Color-px_cur_after1_Color)+
                                            (px_cur_after1_Color-px_cur_after2_Color)* (px_cur_after1_Color-px_cur_after2_Color);
       return segma2photo/max(grad2AloneEpipolar,0.001);



 }
 double Matcher::computeGeometricError(const Vector2d&epi_dir_unit, const Vector2d&best_match,const Frame& cur_frame)
 {
   double segma2Epipolar=64;
   int grady=cur_frame.img_pyr_[0].at<uchar>(best_match[1]-1,best_match[0])-cur_frame.img_pyr_[0].at<uchar>(best_match[1]+1,best_match[0]);
   int gradx=cur_frame.img_pyr_[0].at<uchar>(best_match[1],best_match[0]-1)-cur_frame.img_pyr_[0].at<uchar>(best_match[1],best_match[0]-1);
   cout<<"epi_dir_unit  "<<epi_dir_unit<<endl;
   cout<<"grady  "<<grady<<"  gradx  "<<gradx<<endl;
   Vector2d grd(gradx,grady);
   double jiajiao=abs(gradx*epi_dir_unit[0]+grady*epi_dir_unit[1]);
return segma2Epipolar/max(jiajiao,0.001);
 }
 double Matcher::SubpixelInterpolation( const Vector2d&px_cur_,const Frame& cur_frame)
 {
     // compute interpolation weights
     int px_cur_intx=floor(px_cur_[0]);
     int px_cur_inty=floor(px_cur_[1]);
     float subpix_x = px_cur_[0]-px_cur_intx;
     float subpix_y = px_cur_[1]-px_cur_inty;
     float wTL = (1.0-subpix_x)*(1.0-subpix_y);
     float wTR = subpix_x * (1.0-subpix_y);
     float wBL = (1.0-subpix_x)*subpix_y;
     float wBR = subpix_x * subpix_y;
double afterinterpolation=wTL*cur_frame.img_pyr_[0].at<uchar>(px_cur_inty,px_cur_intx)+wTR*cur_frame.img_pyr_[0].at<uchar>(px_cur_inty,px_cur_intx+1)+
                                            wBL*cur_frame.img_pyr_[0].at<uchar>(px_cur_inty+1,px_cur_intx)+wBR*cur_frame.img_pyr_[0].at<uchar>(px_cur_inty+1,px_cur_intx+1);
return afterinterpolation;
 }
void Matcher::showPointMatch(const cv::Mat& ref,const Vector2d & p_ref,const cv::Mat&  cur,const Vector2d & p_cur,const Vector2d &px_A,const Vector2d &px_B)
{
#ifdef  testgrad2AloneEpipolar
char c[20];
sprintf(c,"%d",ii);
string a="/home/baohua/dd/";
a=a+c+".png";
#endif
    cv::Mat showch1,showch3;
    int row_=ref.rows;
    int col=ref.cols;
    int interval=20;
    int col_=2*ref.cols+interval;
    showch1=cv::Mat::zeros(row_,col_,CV_8UC1);
    for(int j=0;j<row_;j++)
        for(int i=0;i<col_;i++)
        {  if(i<col)                                showch1.at<uchar>(j,i)=ref.at<uchar>(j,i);
            else if(i<col+interval&&i>=col)  showch1.at<uchar>(j,i)=0;
            else
                showch1.at<uchar>(j,i)=cur.at<uchar>(j,i-col-interval);
        }
 cv::cvtColor(showch1,showch3,CV_GRAY2BGR  );
 cv::circle(showch3,cv::Point(p_ref[0],p_ref[1]),3,cv::Scalar(0,255,255));
 cv::circle(showch3,cv::Point(p_cur[0]+col+interval,p_cur[1]),3,cv::Scalar(0,255,255));
cv::line(showch3,cv::Point(p_ref[0],p_ref[1]),cv::Point(p_cur[0]+col+interval,p_cur[1]),cv::Scalar(0,255,0),2);
#define whole
#ifdef whole
float k=(px_A[1]-px_B[1])/(px_A[0]-px_B[0]);
float b=px_A[1]-k*px_A[0];
cv::Point left(0,b),right(-b/k,0);
cv::line(showch3,cv::Point(left.x+col+interval,left.y),cv::Point(right.x+col+interval,right.y),cv::Scalar(255,0,255),2);//epipolar
#else
cv::line(showch3,cv::Point(px_A[0]+col+interval,px_A[1]),cv::Point(px_B[0]+col+interval,px_B[1]),cv::Scalar(255,0,255),2);//epipolar
#endif

imshow(" Point  Match ",showch3);
//imwrite("/home/baohua/PointMatch.png ",showch3);
#ifdef  testgrad2AloneEpipolar
imwrite(a,showch3);
#endif
cv::waitKey(20);
}
}
