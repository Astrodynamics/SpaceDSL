//$Id$
//------------------------------------------------------------------------------
//                              Camera
//------------------------------------------------------------------------------
// GMAT: General Mission Analysis Tool
//
// Copyright (c) 2002 - 2018 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration.
// All Other Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License"); 
// You may not use this file except in compliance with the License. 
// You may obtain a copy of the License at:
// http://www.apache.org/licenses/LICENSE-2.0. 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either 
// express or implied.   See the License for the specific language
// governing permissions and limitations under the License.
//
// Author: Phillip Silvia, Jr.
// Created: 2009/07/02
/**
 * Controls all camera functionality, including move the camera and tracking
 * objects with a camera. 
 */
//------------------------------------------------------------------------------

#ifndef _CAMERA_H
#define _CAMERA_H

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

class Camera
{
public:
   Camera();
   Camera(double x, double y, double z);
   Camera(Vector3d initial_location);
   ~Camera();

   // Camera's current location
   Vector3d position;
   // Three vectors to track the orientation of the camera
   Vector3d forward;
   Vector3d up;
   Vector3d right;
   // View center location
   Vector3d view_center;
   // The field of view, in degrees
   double fovDeg;
   // The id of the object the camera is tracking
   int trackingId;

   void Relocate (double position_x, double position_y, double position_z,
      double center_x, double center_y, double center_z);
   void Relocate (Vector3d new_position, Vector3d new_center);
   void Translate (double x, double y, double z, bool move_center);
   void TranslateW (double x, double y, double z, bool move_center);
   void TranslateCenter (double x, double y, double z);
   void TranslateCenterW (double x, double y, double z);
   void Rotate (double x_angle, double y_angle, double z_angle, bool use_degrees, bool move_camera);
   void Reset ();
   void ReorthogonalizeVectors();
   void ZoomIn(double zoom);
   void ZoomOut(double zoom);
   /*void TrackStill(int tracking_object_id);
   void TrackFollow(int tracking_object_id);
   void Untrack();
   int TrackingMode();
   bool IsTracking();
   void TrackingUpdate(Vector3d new_position);*/

private:
   enum CameraMode { STILL = 0, STILL_TRACKING = 1, FOLLOW_TRACKING = 2, TACKED_TRACKING = 3 };

   // The location that the camera is linked to
   Vector3d tracking_position;
   CameraMode camera_mode;
   
   Vector3d RotateAround(Vector3d vector, double angle, Vector3d axis);
};

#endif
