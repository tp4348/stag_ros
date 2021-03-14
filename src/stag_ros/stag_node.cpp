/**
MIT License

Copyright (c) 2020 Michail Kalaitzakis and Brennan Cain (Unmanned Systems and
Robotics Lab, University of South Carolina, USA)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Project includes
#ifndef NDEBUG
#include "stag_ros/instrument.hpp"
#endif

#include "stag_ros/stag_node.h"
#include "stag_ros/utility.hpp"
#include "stag_ros/load_yaml_tags.h"
// Stag marker handle
#include "stag/Marker.h"

// ROS includes
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>

#include <stdexcept>
#include <iostream>
#include <stag_ros/common.hpp>

namespace stag_ros {

StagNode::StagNode(ros::NodeHandle &nh,
                   image_transport::ImageTransport &imageT) {
  // Load Parameters
  loadParameters();

  // Initialize Stag
  try {
    stag = new Stag(stag_library, error_correction, false);
  } catch (const std::invalid_argument &e) {
    std::cout << e.what() << std::endl;
    exit(-1);
  }

  // Set Subscribers
  imageSub = imageT.subscribe(
      image_topic, 1, &StagNode::imageCallback, this,
      image_transport::TransportHints(is_compressed ? "compressed" : "raw"));
  cameraInfoSub =
      nh.subscribe(camera_info_topic, 1, &StagNode::cameraInfoCallback, this);
  markersSub =
      nh.subscribe(markers_topic, 1, &StagNode::markersArrayCallback, this);

  // Set Publishers
  if (debug_images)
    imageDebugPub = imageT.advertise("stag_ros/image_markers", 1);
  markersPub = nh.advertise<geometry_msgs::PoseStamped>(markers_topic, 1);
  markersArrayPub = nh.advertise<vision_msgs::Detection2DArray>(markers_array_topic, 1);


  // Initialize camera info
  got_camera_info = false;
  cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
  distortionMat = cv::Mat::zeros(1, 5, CV_64F);
  rectificationMat = cv::Mat::zeros(3, 3, CV_64F);
  projectionMat = cv::Mat::zeros(3, 4, CV_64F);
}

StagNode::~StagNode() { delete stag; }

void StagNode::loadParameters() {
  // Create private nodeHandle to load parameters
  ros::NodeHandle nh_lcl("~");

  nh_lcl.param("libraryHD", stag_library, 15);
  nh_lcl.param("errorCorrection", error_correction, 7);
  nh_lcl.param("raw_image_topic", image_topic, std::string("image_raw"));
  nh_lcl.param("camera_info_topic", camera_info_topic,
               std::string("camera_info"));
  nh_lcl.param("markers_topic", markers_topic,
               std::string("stag_ros/markers"));
  nh_lcl.param("markers_array_topic", markers_array_topic,
               std::string("stag_ros/markers_array"));
  nh_lcl.param("is_compressed", is_compressed, false);
  nh_lcl.param("show_markers", debug_images, true);
  nh_lcl.param("publish_tf", publish_tf, false);
  nh_lcl.param("tag_tf_prefix", tag_tf_prefix, std::string("STag_"));

  loadTagsBundles(nh_lcl, "tags", "bundles", tags, bundles);
}

bool StagNode::getTagIndex(const int id, int &tag_index) {
  for (int i = 0; i < tags.size(); ++i) {
    if (tags[i].id == id) {
      tag_index = i;
      return true;
    }
  }
  return false;  // not found
}

void StagNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
#ifndef NDEBUG
  INSTRUMENT;
#endif
  if (got_camera_info) {
    cv::Mat gray;
    msgToGray(msg, gray);

    // Process the image to find the markers
    stag->detectMarkers(gray);
    std::vector<Marker> markers = stag->getMarkerList();

    // Publish debug image
    if (debug_images) {
      cv_bridge::CvImage rosMat;
      rosMat.header = msg->header;
      rosMat.encoding = "bgr8";
      rosMat.image = stag->drawMarkers();

      sensor_msgs::Image rosImage;
      rosMat.toImageMsg(rosImage);

      imageDebugPub.publish(rosImage);
    }

    // For each marker in the list
    if (markers.size() > 0) {
      ROS_INFO("STag: Marker detected");
      // Create markers msg
      std::vector<cv::Mat> tag_pose(tags.size(), cv::Mat::zeros(3, 4, CV_64F));

      // Create marker msg
      int tag_index;

      for (int i = 0; i < markers.size(); i++) {
        // if tag is a single tag, push back
        if (getTagIndex(markers[i].id, tag_index)) {
          std::vector<cv::Point2d> tag_image(5);
          std::vector<cv::Point3d> tag_world(5);

          tag_image[0] = markers[i].center;
          tag_world[0] = tags[tag_index].center;

          for (size_t ci = 0; ci < 4; ++ci) {
            tag_image[ci + 1] = markers[i].corners[ci];
            tag_world[ci + 1] = tags[tag_index].corners[ci];
          }

          cv::Mat marker_pose = cv::Mat::zeros(3, 4, CV_64F);
          Common::solvePnpSingle(tag_image, tag_world, marker_pose,
                                 cameraMatrix, distortionMat);
          tag_pose[tag_index] = marker_pose;

        } else {
	  ROS_WARN("STag: No corresponding tag detected ID");
	  return;
	}
      }

      if (tag_pose[tag_index].empty()) return;

      tf::Matrix3x3 rotMat(
	  tag_pose[tag_index].at<double>(0, 0), tag_pose[tag_index].at<double>(0, 1),
	  tag_pose[tag_index].at<double>(0, 2), tag_pose[tag_index].at<double>(1, 0),
	  tag_pose[tag_index].at<double>(1, 1), tag_pose[tag_index].at<double>(1, 2),
	  tag_pose[tag_index].at<double>(2, 0), tag_pose[tag_index].at<double>(2, 1),
	  tag_pose[tag_index].at<double>(2, 2));
      tf::Quaternion rotQ;
      rotMat.getRotation(rotQ);

      tf::Vector3 tfVec(tag_pose[tag_index].at<double>(0, 3),
			tag_pose[tag_index].at<double>(1, 3),
			tag_pose[tag_index].at<double>(2, 3));
      auto marker_tf = tf::Transform(rotQ, tfVec);
      Common::publishTransform(marker_tf, markersPub, msg->header,
			       tag_tf_prefix, tags[tag_index].frame_id, publish_tf);
    } else { 
	ROS_WARN("No markers detected"); 
    }
  }
}

void StagNode::markersArrayCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
  std::map<std::string, vision_msgs::Detection2D>::iterator it = markersFrames.find(msg->header.frame_id);
  if (it == markersFrames.end()) { 
    vision_msgs::Detection2D markerobj;
    markerobj.header = msg->header;
    vision_msgs::ObjectHypothesisWithPose marker;
    marker.pose.pose = msg->pose;

    // String tokenization ~ ID from frame
    size_t i = 0;
    std::string frame = msg->header.frame_id;
    for ( ; i < frame.length() ; i++) {
      if (std::isdigit(frame[i])) break;
    }
    std::string id = frame.substr(i, frame.length() - 1);
    marker.id = stoi(id);

    markerobj.results.push_back(marker);
    markersFrames.insert(std::pair<std::string, vision_msgs::Detection2D>(msg->header.frame_id, markerobj));
  }else {
    vision_msgs::Detection2DArray array;
    array.header.stamp = ros::Time::now();
    
    for (const auto &d : markersFrames) {
        array.detections.push_back(d.second);
    }
    markersFrames.clear();

    markersArrayPub.publish(array);
  }
}

void StagNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
  if (!got_camera_info) {
    // Get camera Matrix
    cameraMatrix.at<double>(0, 0) = msg->K[0];
    cameraMatrix.at<double>(0, 1) = msg->K[1];
    cameraMatrix.at<double>(0, 2) = msg->K[2];
    cameraMatrix.at<double>(1, 0) = msg->K[3];
    cameraMatrix.at<double>(1, 1) = msg->K[4];
    cameraMatrix.at<double>(1, 2) = msg->K[5];
    cameraMatrix.at<double>(2, 0) = msg->K[6];
    cameraMatrix.at<double>(2, 1) = msg->K[7];
    cameraMatrix.at<double>(2, 2) = msg->K[8];

    // Get distortion Matrix
    distortionMat.at<double>(0, 0) = msg->D[0];
    distortionMat.at<double>(0, 1) = msg->D[1];
    distortionMat.at<double>(0, 2) = msg->D[2];
    distortionMat.at<double>(0, 3) = msg->D[3];
    distortionMat.at<double>(0, 4) = msg->D[4];
    // Get rectification Matrix
    rectificationMat.at<double>(0, 0) = msg->R[0];
    rectificationMat.at<double>(0, 1) = msg->R[1];
    rectificationMat.at<double>(0, 2) = msg->R[2];
    rectificationMat.at<double>(1, 0) = msg->R[3];
    rectificationMat.at<double>(1, 1) = msg->R[4];
    rectificationMat.at<double>(1, 2) = msg->R[5];
    rectificationMat.at<double>(2, 0) = msg->R[6];
    rectificationMat.at<double>(2, 1) = msg->R[7];
    rectificationMat.at<double>(2, 2) = msg->R[8];
    // Get projection Matrix
    projectionMat.at<double>(0, 0) = msg->P[0];
    projectionMat.at<double>(0, 1) = msg->P[1];
    projectionMat.at<double>(0, 2) = msg->P[2];
    projectionMat.at<double>(1, 0) = msg->P[3];
    projectionMat.at<double>(1, 1) = msg->P[4];
    projectionMat.at<double>(1, 2) = msg->P[5];
    projectionMat.at<double>(2, 0) = msg->P[6];
    projectionMat.at<double>(2, 1) = msg->P[7];
    projectionMat.at<double>(2, 2) = msg->P[8];
    projectionMat.at<double>(2, 0) = msg->P[9];
    projectionMat.at<double>(2, 1) = msg->P[10];
    projectionMat.at<double>(2, 2) = msg->P[11];

    got_camera_info = true;
  }
}
}  // namespace stag_ros

int main(int argc, char **argv) {
  ros::init(argc, argv, "stag_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport imageT(nh);

  stag_ros::StagNode stagN(nh, imageT);

  ros::spin();

  return 0;
}
