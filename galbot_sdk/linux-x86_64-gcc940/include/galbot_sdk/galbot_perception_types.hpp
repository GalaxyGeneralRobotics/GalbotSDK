/**
 * @file galbot_perception_types.hpp
 * @author Galbot SDK Team
 * 
 * @copyright Copyright (c) 2023-2026 Galbot. All rights reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Enabled perception pipelines (model sets loaded at init).
 *
 * @robot G1 S1
 */
enum class PerceptionModule {
    FOUNDATION_STEREO = 0,  // High-accuracy stereo depth; for tasks needing precision (e.g. box handling).
    LIGHT_STEREO,   // Lightweight stereo depth; for looser accuracy (e.g. greeting). Not supported in this version.
    STATUS_NUM          // Number of module entries (sentinel).
};

#define SENSORDATA_POINTER_TYPEDEFS(TypeName)         \
    typedef std::shared_ptr<TypeName> Ptr;            \
    typedef std::shared_ptr<const TypeName> ConstPtr; \
    void definePointerTypedefs##__FILE__##__LINE__(void)

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

/**
 * @brief Axis-aligned 2D bounding box with class id and score helpers.
 *
 * @robot G1 S1
 */
typedef struct {
    cv::Rect rect;
    std::pair<int, float> cls;

    int x1() const { return rect.x; }
    int y1() const { return rect.y; }
    int x2() const { return rect.x + rect.width; }
    int y2() const { return rect.y + rect.height; }

    float area() const { return rect.width * rect.height; }
    cv::Point2f center() const {
        return cv::Point2f(x1() + rect.width / 2, y1() + rect.height / 2);
    }
} BBox;

/**
 * @brief Structured garbage / bin-like detection output (3D boxes and optional clouds).
 *
 * @robot G1 S1
 */
struct GarbageResult {
    SENSORDATA_POINTER_TYPEDEFS(GarbageResult);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::map<std::string, Eigen::Vector3f> boxes_shape;       // length, width, height
    std::map<std::string, Eigen::Vector3f> boxes_center_pos;  // x, y, z in base_link
    std::map<std::string, cv::Rect> bboxes;
    std::map<std::string, PointCloudPtr> bboxes_pointcloud;
    void addResult(const std::string& uuid, Eigen::Vector3f box_shape,
                   const Eigen::Vector3f& box_center_pos, const cv::Rect& box) {
        boxes_shape[uuid] = box_shape;
        boxes_center_pos[uuid] = box_center_pos;
        bboxes[uuid] = box;
    }
    void addPointCloud(const std::string& uuid, const PointCloudPtr& bbox_pointcloud) {
        bboxes_pointcloud[uuid] = bbox_pointcloud;
    }
};

/**
 * @brief Single-object detection or instance segmentation record (2D box, class, optional mask/keypoints).
 *
 * @robot G1 S1
 */
struct DetectionAndSegmentationResult {
    SENSORDATA_POINTER_TYPEDEFS(DetectionAndSegmentationResult);
    cv::Rect bbox;                       // Bounding box.
    std::string className;               // Class label.
    int classIndex;                      // Class index.
    float confidence;                    // Confidence score.
    std::vector<cv::Point2f> keypoints;  // 2D keypoints.
    std::vector<cv::Point> polygon;      // Segmentation contour (optional; may be empty).
    DetectionAndSegmentationResult() : className(""), classIndex(-1), confidence(0.0f) {}

    DetectionAndSegmentationResult(const cv::Rect& box, const std::string& name, const int index,
                                   const float conf,
                                   const std::vector<cv::Point2f>& kps = std::vector<cv::Point2f>(),
                                   const std::vector<cv::Point>& poly = std::vector<cv::Point>())
        : bbox(box),
          className(name),
          classIndex(index),
          confidence(conf),
          keypoints(kps),
          polygon(poly) {}

    void printInfo(std::ostream& os, bool showPolygon = false) const {
        os << "Class: " << className << " (index=" << classIndex << "), Conf: " << confidence
           << ", BBox: [" << bbox.x << ", " << bbox.y << ", " << bbox.width << ", " << bbox.height
           << "]";

        if (showPolygon) {
            if (!polygon.empty()) {
                os << ", Polygon: { ";
                for (const auto& p : polygon) os << "(" << p.x << "," << p.y << ") ";
                os << "}";
            } else {
                os << ", Polygon: <none>";
            }
        }

        os << std::endl;
    }
};

inline std::ostream& operator<<(std::ostream& os, const DetectionAndSegmentationResult& d) {
    d.printInfo(os);
    return os;
}

/**
 * @brief Aggregated perception output for one module tick (images, masks, poses, point clouds, etc.).
 *
 * @robot G1 S1
 */
struct DetectionResult {
    SENSORDATA_POINTER_TYPEDEFS(DetectionResult);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp_ns;
    std::string sensorName;
    cv::Mat resultImage;                          // Result image.
    std::vector<PointCloudPtr> resultPointCloud;  // Result point clouds.
    std::vector<DetectionAndSegmentationResult> detectionAndSegmentationResults;
    std::vector<cv::Rect> boundingBoxes;              // Bounding boxes.
    std::vector<std::string> classNames;              // Class names.
    std::vector<int> classIndices;                    // Class indices.
    std::vector<float> confidences;                   // Confidence scores.
    std::vector<std::string> ocrString;               // OCR text strings.
    std::vector<Eigen::Matrix4f> targetPoses;         // Target poses (4x4).
    std::vector<std::vector<cv::Point2f>> keypoints;  // 2D keypoints per detection.
    std::string runningInfo;                          // Runtime / diagnostic message.
    cv::Mat classMask;                                // Class mask (segmentation).
    cv::Mat instanceMask;                             // Instance mask (segmentation); stereo depth output may use this field.
    GarbageResult::Ptr garbage_result;
    std::map<std::string, std::vector<Eigen::Matrix4f>> object6DPoses;  // 6D pose results per object id.
    std::vector<std::vector<float>> grasp_pose_result;
    std::vector<Eigen::Matrix4f> targetPointPoses;
    std::vector<Eigen::Matrix4f> preTargetPointPoses;  // Pre-target point poses.
    std::vector<Eigen::Matrix4f> ocrPointPoses;
    std::map<std::string, std::vector<cv::Mat>> ocrLabelImage;
    std::map<std::string, Eigen::Matrix4f> ocrResults;  // OCR detection / text poses.

    // Append one detection result (bbox + class + score).
    void addResult(const cv::Rect& box, const std::string& className, int classIndex,
                   float confidence) {
        DetectionAndSegmentationResult det_and_seg_result(box, className, classIndex, confidence);
        detectionAndSegmentationResults.emplace_back(det_and_seg_result);
        boundingBoxes.push_back(box);
        classNames.push_back(className);
        classIndices.push_back(classIndex);
        confidences.push_back(confidence);
    }

    void addResult(const cv::Rect& box, const std::vector<cv::Point2f>& kps,
                   const std::string& className, int classIndex, float confidence) {
        DetectionAndSegmentationResult det_and_seg_result(box, className, classIndex, confidence,
                                                          kps);
        detectionAndSegmentationResults.emplace_back(det_and_seg_result);
        boundingBoxes.push_back(box);
        classNames.push_back(className);
        classIndices.push_back(classIndex);
        confidences.push_back(confidence);
        keypoints.push_back(kps);
    }

    // Append a target pose.
    void addPose(const Eigen::Matrix4f& pose) { targetPoses.push_back(pose); }

    void addKeypoints(const std::vector<cv::Point2f>& kps) { keypoints.push_back(kps); }

    void addPointCloud(const PointCloudPtr& cloud) { resultPointCloud.push_back(cloud); }

    void setRunningInfo(const std::string& info) { runningInfo = info; }

    void addClassMask(const cv::Mat& mask) { classMask = mask; }

    void addInstanceMask(const cv::Mat& mask) { instanceMask = mask; }

    // Clear all stored results.
    void clear() {
        timestamp_ns = -1;
        sensorName = "";
        boundingBoxes.clear();
        classNames.clear();
        classIndices.clear();
        confidences.clear();
        targetPoses.clear();
        keypoints.clear();
        runningInfo.clear();
        classMask.release();
        resultPointCloud.clear();
        instanceMask.release();
        detectionAndSegmentationResults.clear();
    }

    std::string getResultInfo() {
        std::stringstream ss;
        ss << " " << detectionAndSegmentationResults.size() << " objs " << std::endl;
        ss << "Target Poses: " << targetPoses.size() << std::endl;
        ss << "Result Point Cloud: " << resultPointCloud.size() << std::endl;
        if (!keypoints.empty()) {
            ss << "Keypoints: " << keypoints.size() << "x" << keypoints.front().size() << std::endl;
        }
        ss << "Running Info: " << runningInfo << std::endl;

        ss << "ocrString: ";

        for (size_t i = 0; i < ocrString.size(); i++) {
            ss << ocrString[i] << ",";
        }

        return ss.str();
    }

    void copyFrom(const DetectionResult& other) {
        timestamp_ns = other.timestamp_ns;
        resultImage = other.resultImage;
        resultPointCloud = other.resultPointCloud;
        detectionAndSegmentationResults = other.detectionAndSegmentationResults;
        boundingBoxes = other.boundingBoxes;
        classNames = other.classNames;
        classIndices = other.classIndices;
        confidences = other.confidences;
        targetPoses = other.targetPoses;
        keypoints = other.keypoints;
        ocrString = other.ocrString;
        runningInfo = other.runningInfo;
        classMask = other.classMask;
        instanceMask = other.instanceMask;
    }

    DetectionResult() : timestamp_ns(-1) {}
};
