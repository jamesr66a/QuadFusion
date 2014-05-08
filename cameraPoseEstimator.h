#ifndef CAMERA_POSE
#define CAMERA_POSE

/******************************************
 * cameraPoseEstimator.h
 * Matthew Bailey 
 * Chris Wakeley
 * Pachu Chembukave
 *
 * This file encapsulates the vision system
 * in a class with a straightforward interface.
 ******************************************/

// selection of camera:
// Laptop camera is for debugging,
// define USE_ECON_CAMERA for code on Gumstix.
#define USE_ECON_CAMERA
//#define USE_LAPTOP_CAMERA

#include <iostream>
#include <fstream>

#include <string>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <map>
#include <mutex>
#include <atomic>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "findPose.h"
#include "SquarePattern.h"
#include "StoredPatterns.h"
#include "econ.h"

using namespace cv;

struct CandidateTag {
  Point2f corner[4];
  SquarePattern pattern;
  Mat r;
  Mat t;
  Mat rp;
  Mat tp;
};

class CameraPoseEstimator {
public:
  CameraPoseEstimator();
//  ~CameraPoseEstimator();

  void continuousRead();
  bool dataAvailable();
  void getPose(Pose3D&);
/*  int getRawPose(Pose3D&);
  int getTagPose(Pose3D&);
*/

private:
  void getImage(Mat&);
  void findCandidateTags(vector<CandidateTag*>&, Mat&);
  void filterTags(vector<CandidateTag*>&, vector<CandidateTag*>&, vector<CandidateTag*>&);
  void registerUnknownTags(vector<CandidateTag*>&, Mat&, Mat&);

  std::atomic_bool hasNewData;

  static const int NUMROWS = 480;
  static const int NUMCOLS = 640;

  #ifdef USE_ECON_CAMERA
    static const int cameraId = 2;
  #else
    static const int cameraId = 0;
  #endif

  static const int polygonCloseResolution = 1000;//15;
  float innerSquareLength;

  #ifdef USE_ECON_CAMERA
    econ* capture;
  #else
    VideoCapture capture;
  #endif

  SquarePatternHandle* squareHandle;
  SquarePatternHandle* candidateHandle;

  Mat cameraMatrix, distCoeffs, distmap1, distmap2;

  vector<Point3f> testPointGrid;
  vector<Point3f> testOuterSquare;
  vector<SquarePattern> foundPatterns;
//  std::map<SquarePattern, tagPose> foundTags;

//  Pose3D pose;
  vector<Pose3D> poseList;
  std::mutex listAccess;
};

CameraPoseEstimator::CameraPoseEstimator() {

  // set up camera and tag handler
  #ifdef USE_ECON_CAMERA
    capture = new econ(cameraId, NUMCOLS, NUMROWS);
  #else
    capture.open(cameraId);
    if( !capture.isOpened()){
      fprintf(stderr, "Could not initialize video (%d) capture\n", cameraId);
    }
    Mat src;
    capture.read(src);
  #endif

  squareHandle = new SquarePatternHandle(3);
  candidateHandle = new SquarePatternHandle(3);

  hasNewData = false;

  // add the initial tag to the tag handler
  tagPose initialPose;
  initialPose.r_vec = (Mat_<double>(3,1) << 0.,0.,0.);  
  initialPose.t = (Mat_<double>(3,1) << 0.,0.,0.);
  addPattern(*squareHandle, INITIAL_PATTERN, initialPose);
  
  // add the candidate patterns to the candidate handler
  for(int k = 0 ; k < numPatterns; k++) {
    candidateHandle->add(storedPatterns[k]);
  }

  // set up the camera matrix and distortion matrix, found through camera calibration.
  cameraMatrix = (Mat_<float>(3,3) << 5.7951952713850142e+02, 0., 3.1950000e+02, 0., 5.7951952713850142e+02, 2.395000e+02, 0., 0., 1.);
  distCoeffs = (Mat_<float>(5,1) << 3.3071823987974308e-01, -2.5506436646233288e+00, 0., 0., 5.0980401718181261e+00);

  // set up the test points for the pattern grid
  Point3f cur;  
  for(int i = 0; i < gridSize; i++) {
    for(int j = 0; j < gridSize; j++) {
      cur.x = gridBorderOffset + ((float)i)*GapLength + ((float)i + 0.5)*SquareSideLength;
      cur.y = gridBorderOffset + ((float)j)*GapLength + ((float)j + 0.5)*SquareSideLength;
      cur.z = 0;
      testPointGrid.push_back(cur);
    }
  }
  
  innerSquareLength = gridSize * (SquareSideLength + GapLength) - GapLength + 2*gridBorderOffset;

  // set up the four corner points
  cur.x = 0;
  cur.y = 0;
  cur.z = 0;
  testOuterSquare.push_back(cur);
  
  cur.x = 2*gridBorderOffset + (gridSize - 1)*GapLength + (gridSize)*SquareSideLength;
  cur.y = 0;
  cur.z = 0;
  testOuterSquare.push_back(cur);
  
  cur.x = 2*gridBorderOffset + (gridSize - 1)*GapLength + (gridSize)*SquareSideLength;
  cur.y = 2*gridBorderOffset + (gridSize - 1)*GapLength + (gridSize)*SquareSideLength;
  cur.z = 0;
  testOuterSquare.push_back(cur);
  
  cur.x = 0;
  cur.y = 2*gridBorderOffset + (gridSize - 1)*GapLength + (gridSize)*SquareSideLength;
  cur.z = 0;
  testOuterSquare.push_back(cur);
       
  // set up the distortion matrices
  distmap1 = Mat(NUMROWS, NUMCOLS, CV_16SC2);
  distmap2 = Mat(NUMROWS, NUMCOLS, CV_16UC1);
  
  //initialize the distortion maps
  initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat_<double>::eye(3,3), cameraMatrix, 
                          Size(NUMCOLS, NUMROWS), distmap1.type(), distmap1, distmap2);
}

void CameraPoseEstimator::getImage(Mat& img) {
  Mat src, src_gray;

  // capture image from camera
  #ifdef USE_ECON_CAMERA
    capture->readImg(src);  
  #else
    capture.read(src);
  #endif

  // convert to grayscale
  cvtColor(src, src_gray, COLOR_RGB2GRAY );

  // undistort the image
  remap(src_gray, img, distmap1, distmap2, INTER_LINEAR, BORDER_CONSTANT );
}

void CameraPoseEstimator::findCandidateTags(vector<CandidateTag*>& candidateTags, Mat& img) {
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  vector<vector<Point> > innerContours;
  Mat thresholded;

  //Use canny to create an edge map
  Canny(img, thresholded, 50, 100, 3);

  //find the contours in the edgemap  *******THIS MIGHT BE ABLE TO BE OPTIMIZED********        
  findContours(thresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0,0));
               
  //find the quadrilaterals in the contours  
  foundPatterns.clear();  
//  foundTags.clear();    
  for(int i = 0; i < contours.size(); i++) {
    if(/*hierarchy[i][2] >= 0  &&*/
       sqrt((contours[i][0].x - contours[i][contours[i].size() - 1].x)*
       (contours[i][0].x - contours[i][contours[i].size() - 1].x) +
       (contours[i][0].y - contours[i][contours[i].size() - 1].y)*
       (contours[i][0].y - contours[i][contours[i].size() - 1].y)) < polygonCloseResolution
       ){ 
         
      //fits a polygon to the contour
      vector<Point> polygons;
      approxPolyDP(contours[i], polygons, 10, true);
            
      //if the fit polygon is four sided
      if(polygons.size() == 4){
              
        //add its inner contours to innerContours
        int nextcont = hierarchy[i][2];
        innerContours.clear();
        while(nextcont >= 0){
          vector<Point> innerpolygons;
          approxPolyDP(contours[nextcont], innerpolygons, 10, true);
                
          //if the polygon has at least 4 'children'
          if(innerpolygons.size() >= 4) {
            innerContours.push_back(innerpolygons);
          }
          nextcont = hierarchy[nextcont][0];
        }
              
        if(innerContours.size() < 2) continue;

        //construct the pattern in the quad  
        CandidateTag* newTag = new CandidateTag;
        newTag->pattern = NULL_PATTERN;

        vector<Point2f> imagePoints;
        vector<Point2f> squareVector;
        Point2f cur;
            
        for(int z = 0; z < 4; z++) {
          cur.x = polygons[z].x;
          cur.y = polygons[z].y;
          squareVector.push_back(cur);

          newTag->corner[z] = polygons[z];
        }

        solvePnP(testOuterSquare, squareVector, cameraMatrix, distCoeffs, newTag->r, newTag->t, false, CV_ITERATIVE);
        projectPoints(testPointGrid, newTag->r, newTag->t, cameraMatrix, distCoeffs, imagePoints);

        for(int z = 0; z < imagePoints.size(); z++) {
          for(vector< vector<Point> >::iterator m = innerContours.begin(); m != innerContours.end(); ++m) {
            if(pointPolygonTest(*m, imagePoints[z], false) >= 0) {
              candidateHandle->set(newTag->pattern, int(z/gridSize), z%gridSize, true);
              vector< vector<Point> >::iterator n = m;
              --m;
              innerContours.erase(n);
              break;
            }
          }
        }

        rotation temprot = candidateHandle->findMatchingPattern(newTag->pattern);
        if(temprot.pattern != NULL_PATTERN) {
          newTag->pattern = temprot.pattern;
          foundPatterns.push_back(temprot.pattern);

          tagPose tempPose;
          double a = temprot.angle;

          Mat r0 = (Mat_<double>(gridSize, 1) << 0., 0., temprot.angle*M_PI_2);
          Mat t0 = (Mat_<double>(gridSize, 1) << ((a == 1 || a == 2)? 
                                                   innerSquareLength : 0.),
                                                 ((a == 2 || a == 3)?
                                                   -innerSquareLength : 0.),
                                                 0.); 

          Mat temp;  
        
          // r0 and t0 transform from the coordinate system of the
          // known tag to the coordinate system of the pattern
          // used in solvePnP.

          // r and t transform from the reference frame of the pattern 
          // used in solvePnP to the reference frame of the camera
              
          // this composition transforms from the tag reference
          // frame to the camera reference frame
 
          composeRT(r0, t0, newTag->r, newTag->t, r0, t0);

          // inverting the above transformation gives a transformation
          // from the coordinate system of the camera to the reference
          // frame of the tag.
          Rodrigues(r0, temp);
          transpose(temp, temp);
          tempPose.t = -temp*t0;
          Rodrigues(temp, tempPose.r_vec);

//          foundTags[temprot.pattern]=tempPose;
          newTag->tp = tempPose.t;
          newTag->rp = tempPose.r_vec;
          candidateTags.push_back(newTag);
        }
      }
    }       
  }       
}

void CameraPoseEstimator::filterTags(vector<CandidateTag*>& knownTags, vector<CandidateTag*>& unknownTags, vector<CandidateTag*>& candidateTags) {
  //update the pose using the known patterns
  Mat r0, t0;

  for(vector<CandidateTag*>::iterator m = candidateTags.begin(); m != candidateTags.end(); ++m) {
    rotation rot = squareHandle->findMatchingPattern((*m)->pattern);
    if(rot.pattern != NULL_PATTERN){
      knownTags.push_back(*m);

      // This composes the transformation from the camera to
      // the tag with the transformation from the tag to the world
      composeRT(       (*m)->rp,                (*m)->tp,
                patternPose[(*m)->pattern].r_vec, patternPose[(*m)->pattern].t,
                       (*m)->rp,                (*m)->tp);
    }
    else {
      unknownTags.push_back(*m);
    }
  }    
}

void CameraPoseEstimator::registerUnknownTags(vector<CandidateTag*>& unknownTags, Mat& r0, Mat& t0) {
  Mat R;
  for(vector<CandidateTag*>::iterator m = unknownTags.begin(); m != unknownTags.end(); ++m) {
    tagPose newPose;
    Rodrigues(-(*m)->rp, R);

    // r0, t0 transform from camera-space to world-space
    // foundTags transforms from camera-space to tag-space
    // invert foundTags, compose with r0, t0 to transform
    // from tag-space to world-space 
    composeRT(-((*m)->rp),           -R*((*m)->tp),
                 r0,                   t0,
              newPose.r_vec,        newPose.t);
      
    addPattern(*squareHandle, (*m)->pattern, newPose);
  }       
}

void CameraPoseEstimator::continuousRead(){
  for(;;) {
    Pose3D pose;
    Mat img;
    vector<CandidateTag*> candidateTags, knownTags, unknownTags;
    this->getImage(img);
    this->findCandidateTags(candidateTags, img);
    this->filterTags(knownTags, unknownTags, candidateTags);

    Mat r0, t0;
    if(knownTags.size() > 0) {
      r0 = knownTags[0]->rp;
      t0 = knownTags[0]->tp;

      // add unknown tags to square pattern handler
      this->registerUnknownTags(unknownTags, r0, t0);

      Mat R;
      Rodrigues(r0, R);
      find3DPose(R, t0, pose);

      this->listAccess.lock();
      this->poseList.push_back(pose);
      this->hasNewData = true;
      this->listAccess.unlock();

//      std::cout << "cam pose: " << r0 << " " << t0 << std::endl;
    }

    for(int i = 0; i < candidateTags.size(); ++i) {
      delete candidateTags[i];
    }
  }
}

bool CameraPoseEstimator::dataAvailable() {
/*  bool hasData = false;
  listAccess.lock();
  hasData = !(poseList.empty());
  listAccess.unlock();

  return hasData;
*/
  return hasNewData;
}

void CameraPoseEstimator::getPose(Pose3D& pose) {
  listAccess.lock();
  pose = poseList.back();
  poseList.pop_back();
  hasNewData = false;
  listAccess.unlock();
}

#endif
