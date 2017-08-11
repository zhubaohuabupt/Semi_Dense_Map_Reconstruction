/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include<iostream>
#include <pangolin/gl/gldraw.h>
namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer_KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer_KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer_GraphLineWidth"];
    mPointSize = fSettings["Viewer_PointSize"];
    mCameraSize = fSettings["Viewer_CameraSize"];
    mCameraLineWidth = fSettings["Viewer_CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
   const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }
    glEnd();




}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize*3;
    const float h = w*0.5;
    const float z = 10*mCameraSize;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

///////////draw wheel
  cv::Point3f leftup=cv::Point3f(-w,0,z*4/5);
    cv::Point3f rightup=cv::Point3f(w,0,z*4/5);
    cv::Point3f leftdown=cv::Point3f(-w,0,z*1/5);
      cv::Point3f rightdown=cv::Point3f(w,0,z*1/5);
    float wheelr=mCameraSize;
      DrawCircle(leftup,cv::Point3f(0,0,0),wheelr);
      DrawCircle(rightup,cv::Point3f(0,0,0),wheelr);
      DrawCircle(leftdown,cv::Point3f(0,0,0),wheelr);
      DrawCircle(rightdown,cv::Point3f(0,0,0),wheelr);
//////draw Car
    DrawCar(wheelr, w, h, z/2);
    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}
void MapDrawer::  DrawCircle(cv::Point3f center,cv::Point3f color,float wheelsize)
{

    float x=center.x;

     float y=center.y;
      float z=center.z;

      float l=x-wheelsize/4;
      float r=x+wheelsize/4;
      float gen2d=0.71*wheelsize;
      glLineWidth(mCameraLineWidth);
      glColor3f(color.x,color.y,color.z);
       glBegin(GL_LINES);
    glVertex3f( l,0+y,(float)(z-wheelsize));
    glVertex3f( l,gen2d+y,z-gen2d);

     glVertex3f(l,gen2d+y,z-gen2d);
     glVertex3f( l  ,wheelsize+y,z);

    glVertex3f(  l  ,wheelsize+y,z);
glVertex3f (l,gen2d+y,z+gen2d);

glVertex3f( l,gen2d+y,z+gen2d);
glVertex3f(l,0+y,z+wheelsize);


glVertex3f(l,0+y,z+wheelsize);
glVertex3f( l,-gen2d+y,z+gen2d);

glVertex3f( l,-gen2d+y,z+gen2d);
glVertex3f(l,-wheelsize+y,z);


glVertex3f(l,-wheelsize+y,z);
glVertex3f( l,-gen2d+y,z-gen2d);

glVertex3f(l,-gen2d+y,z-gen2d);
 glVertex3f(l,0,z-wheelsize);
       /////////////////
   glVertex3f( r,0+y,(float)(z-wheelsize));
   glVertex3f( r,gen2d+y,z-gen2d);

    glVertex3f(r,gen2d+y,z-gen2d);
    glVertex3f( r ,wheelsize+y,z);

    glVertex3f( r ,wheelsize+y,z);
   glVertex3f (r,gen2d+y,z+gen2d);

   glVertex3f( r,gen2d+y,z+gen2d);
   glVertex3f(r,0+y,z+wheelsize);


   glVertex3f(r,0+y,z+wheelsize);
   glVertex3f( r,-gen2d+y,z+gen2d);

   glVertex3f( r,-gen2d+y,z+gen2d);
   glVertex3f(r,-wheelsize+y,z);


   glVertex3f(r,-wheelsize+y,z);
   glVertex3f( r,-gen2d+y,z-gen2d);

   glVertex3f(r,-gen2d+y,z-gen2d);
      glVertex3f(r,0,z-wheelsize);


         //////////////////////////////////////////////////
          glVertex3f( l,0+y,(float)(z-wheelsize));
             glVertex3f( r,0+y,(float)(z-wheelsize));

          glVertex3f( l,gen2d+y,z-gen2d);
           glVertex3f( r,gen2d+y,z-gen2d);


           glVertex3f( l  ,wheelsize+y,z);
           glVertex3f( r ,wheelsize+y,z);

          glVertex3f( l,gen2d+y,z+gen2d);
           glVertex3f( r,gen2d+y,z+gen2d);

      glVertex3f(l,0+y,z+wheelsize);
        glVertex3f(r,0+y,z+wheelsize);


      glVertex3f( l,-gen2d+y,z+gen2d);
      glVertex3f( r,-gen2d+y,z+gen2d);

      glVertex3f(l,-wheelsize+y,z);
       glVertex3f(r,-wheelsize+y,z);


      glVertex3f( l,-gen2d+y,z-gen2d);
        glVertex3f( r,-gen2d+y,z-gen2d);

             /////////////////
  glEnd();
}

 void MapDrawer:: DrawCar(float wheelsize,float w,float h,float z)
{
    //draw car
      float carhdown=-wheelsize;
      float carhup=-(wheelsize+2*h);

cv::Point3f center(0,(carhdown+carhup)/2,z);
cv::Point3f length(w,h,z);
glDrawColouredCuboid(center,length);
Drawbar(wheelsize,w,h,z);
}
 void MapDrawer::Drawbar(float wheelsize,float w,float h,float z)
 {

cv::Point3f center(0,-(wheelsize+5*h+4*h)/2,z);
cv::Point3f length(w/8,4*h,z/16);
glDrawColouredCuboid(center,length);
DrawCam(wheelsize,w,h,z);
 }
 void MapDrawer::DrawCam(float wheelsize,float w,float h,float z)
{
 cv::Point3f center(0,-(wheelsize+5*h+8*h+4*h)/2,z);
 cv::Point3f length(2*w/3,h/6,z/16);
 glDrawColouredCuboid(center,length);
  cv::Point3f LeftcamCenter(-2*w/3,-(wheelsize+5*h+8*h+4*h)/2,z);
  cv::Point3f LeftcamLength(w/3,h*2/3,z/4);
glDrawColouredPyramid(LeftcamCenter,LeftcamLength);//leftCam
cv::Point3f RightcamCenter(2*w/3,-(wheelsize+5*h+8*h+4*h)/2,z);
cv::Point3f RightcamLength(w/3,h*2/3,z/4);
glDrawColouredPyramid(RightcamCenter,RightcamLength);//RightCam
}
  void MapDrawer::glDrawColouredCuboid(cv::Point3f center ,cv::Point3f length)
 {
      const GLfloat verts[] = {
         length.x+center.x,-length.y+center.y,-length.z+center.z, - length.x+center.x, -length.y+center.y,-length.z+center.z,   -length.x+center.x,length.y+center.y,-length.z+center.z,     length.x+center.x,length.y+center.y,-length.z+center.z,// FRONT
            length.x+center.x,length.y+center.y,length.z+center.z,   length.x+center.x,-length.y+center.y,length.z+center.z,  - length.x+center.x,-length.y+center.y,length.z+center.z, - length.x+center.x, length.y+center.y,length.z+center.z,   // BACK
           length.x+center.x,length.y+center.y,-length.z+center.z,   length.x+center.x,-length.y+center.y,-length.z+center.z,   length.x+center.x,-length.y+center.y,length.z+center.z,  length.x+center.x, length.y+center.y,length.z+center.z,  // LEFT
       - length.x+center.x,length.y+center.y,-length.z+center.z, -  length.x+center.x,-length.y+center.y,-length.z+center.z, - length.x+center.x,-length.y+center.y,length.z+center.z, -length.x+center.x, length.y+center.y,length.z+center.z, // RIGHT
        - length.x+center.x,length.y+center.y,length.z+center.z,  - length.x+center.x, length.y+center.y,-length.z+center.z,  length.x+center.x,length.y+center.y,-length.z+center.z,     length.x+center.x,length.y+center.y,length.z+center.z,// TOP
     - length.x+center.x,-length.y+center.y,length.z+center.z,  - length.x+center.x, -length.y+center.y,-length.z+center.z,  length.x+center.x,-length.y+center.y,-length.z+center.z,     length.x+center.x,-length.y+center.y,length.z+center.z,  // BOTTOM
      };
     glVertexPointer(3, GL_FLOAT, 0, verts);
     glEnableClientState(GL_VERTEX_ARRAY);

     glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
     glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glDrawArrays(GL_TRIANGLE_FAN, 4, 4);

     glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
     glDrawArrays(GL_TRIANGLE_FAN, 8, 4);
     glDrawArrays(GL_TRIANGLE_FAN, 12, 4);

     glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
     glDrawArrays(GL_TRIANGLE_FAN, 16, 4);
    glDrawArrays(GL_TRIANGLE_FAN, 20, 4);

     glDisableClientState(GL_VERTEX_ARRAY);
 }
    void MapDrawer::glDrawColouredPyramid(cv::Point3f center ,cv::Point3f length)
    {
        const GLfloat verts[] = {
           length.x+center.x,-length.y+center.y,length.z+center.z, - length.x+center.x, -length.y+center.y,length.z+center.z,   -length.x+center.x,length.y+center.y,length.z+center.z,     length.x+center.x,length.y+center.y,length.z+center.z,// 4
              length.x+center.x,-length.y+center.y,length.z+center.z, - length.x+center.x, -length.y+center.y,length.z+center.z,   center.x,center.y,center.z,//3
          - length.x+center.x,-length.y+center.y,length.z+center.z, - length.x+center.x, length.y+center.y,length.z+center.z,   center.x,center.y,center.z,//3
            -length.x+center.x,length.y+center.y,length.z+center.z,  length.x+center.x, length.y+center.y,length.z+center.z,   center.x,center.y,center.z,//3
            length.x+center.x,length.y+center.y,length.z+center.z,  length.x+center.x, -length.y+center.y,length.z+center.z,   center.x,center.y,center.z,//3
        };
       glVertexPointer(3, GL_FLOAT, 0, verts);
       glEnableClientState(GL_VERTEX_ARRAY);

       glColor4f(1.0f, 0.5f, 0.0f, 1.0f);
       glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

       glColor4f(0.5f, 0.2f, 1.0f, 1.0f);
       glDrawArrays(GL_TRIANGLE_FAN, 7, 3);
       glDrawArrays(GL_TRIANGLE_FAN, 13, 3);

       glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
       glDrawArrays(GL_TRIANGLE_FAN, 4, 3);
      glDrawArrays(GL_TRIANGLE_FAN, 10, 3);

       glDisableClientState(GL_VERTEX_ARRAY);

    }
} //namespace ORB_SLAM
