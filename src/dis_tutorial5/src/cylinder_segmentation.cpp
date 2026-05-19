#include <iostream>
#include <limits>
#include <pcl/console/print.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planes_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cylinder_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

typedef pcl::PointXYZRGB PointT;

// parameters
float error_margin = 0.025; // tight radius margin — reduces wall-corner false positives
float target_radius = 0.11; // 11cm radius
bool verbose = false;

// cloud filtering
float x_limit_low = 0;
float x_limit_high = 2.5;   // only look within 2.5m
float z_limit_low = -0.25;
float z_limit_high = 0.5;

// RANSAC
int ransac_max_iterations = 100;
float ransac_normal_distance_weight = 0.1;
float ransac_distance_threshold = 0.01;

float marker_height = 0.4;
int max_detected_cylinders = 3;
int min_cylinder_size = 500;
float min_z_spread = 0.07f;   // reject flat objects (floor markings) — real cylinders span >7cm vertically
float max_z_spread = 0.50f;   // reject tall objects (boxes, walls) — barrels are at most ~40cm tall
float min_horizontal_axis_z = -0.30f; // reject horizontal detections whose axis center is at floor level (curbs etc.)

void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // save timestamp from message
    rclcpp::Time now = (*msg).header.stamp;

    // set up PCL objects
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    // set up pointers
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PCLPointCloud2::Ptr pcl_pc(new pcl::PCLPointCloud2);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());

    // convert ROS msg to PointCloud2
    pcl_conversions::toPCL(*msg, *pcl_pc);

    // convert PointCloud2 to templated PointCloud
    pcl::fromPCLPointCloud2(*pcl_pc, *cloud);

    if (verbose) {
        std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
    }

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_limit_low, x_limit_high);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);    
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_limit_low, z_limit_high);
    pass.filter(*cloud_filtered);
    
    if (verbose) {
        std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;
    }

    // Remove dominant planes (floor, walls) before cylinder detection.
    // Without this, RANSAC happily fits wall corners and table edges as "cylinders".
    {
        pcl::SACSegmentation<PointT> plane_seg;
        plane_seg.setOptimizeCoefficients(true);
        plane_seg.setModelType(pcl::SACMODEL_PLANE);
        plane_seg.setMethodType(pcl::SAC_RANSAC);
        plane_seg.setDistanceThreshold(0.02);
        plane_seg.setMaxIterations(100);

        pcl::ExtractIndices<PointT> ex;
        for (int p = 0; p < 3 && cloud_filtered->size() > 300; ++p) {
            pcl::PointIndices::Ptr plane_inl(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr plane_coef(new pcl::ModelCoefficients);
            plane_seg.setInputCloud(cloud_filtered);
            plane_seg.segment(*plane_inl, *plane_coef);
            if (plane_inl->indices.size() < 300) break;
            pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
            ex.setInputCloud(cloud_filtered);
            ex.setIndices(plane_inl);
            ex.setNegative(true);
            ex.filter(*tmp);
            cloud_filtered.swap(tmp);
        }
    }

    // Estimate point normals on plane-removed cloud
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // No axis constraint — allow vertical and horizontal cylinders

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(ransac_normal_distance_weight);
    seg.setMaxIterations(ransac_max_iterations);
    seg.setDistanceThreshold(ransac_distance_threshold);
    seg.setRadiusLimits(target_radius-error_margin, target_radius+error_margin);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment(*inliers_cylinder, *coefficients_cylinder);

    // Copy remaining cloud for iterative extraction
    pcl::PointCloud<PointT>::Ptr remaining_cloud(new pcl::PointCloud<PointT>(*cloud_filtered));
    pcl::PointCloud<pcl::Normal>::Ptr remaining_normals(new pcl::PointCloud<pcl::Normal>(*cloud_normals));

    pcl::PointCloud<PointT>::Ptr all_cylinders(new pcl::PointCloud<PointT>());

    // convert to pointcloud2, then to ROS2 message
    sensor_msgs::msg::PointCloud2 plane_out_msg;
    pcl::PCLPointCloud2::Ptr outcloud_plane(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud_filtered, *outcloud_plane);
    pcl_conversions::fromPCL(*outcloud_plane, plane_out_msg);
    planes_pub->publish(plane_out_msg);

    int marker_id = 0;
    int detected_cylinders = 0;

    while (detected_cylinders <= max_detected_cylinders) {

        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);

        seg.setInputCloud(remaining_cloud);
        seg.setInputNormals(remaining_normals);
        seg.segment(*inliers_cylinder, *coefficients_cylinder);

        if (coefficients_cylinder->values.empty() || inliers_cylinder->indices.empty()) {
            break;
        }

        float detected_radius = coefficients_cylinder->values[6];
        int cylinder_points_count = inliers_cylinder->indices.size();

        // Classify orientation from axis direction (values[3..5])
        float ax = coefficients_cylinder->values[3];
        float ay = coefficients_cylinder->values[4];
        float az = coefficients_cylinder->values[5];
        float axis_len = std::sqrt(ax*ax + ay*ay + az*az);
        bool is_vertical = (axis_len > 0) ? (std::abs(az / axis_len) > 0.5f) : true;

        // Extract cylinder
        pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers_cylinder);
        extract.setNegative(false);
        extract.filter(*cloud_cylinder);

        geometry_msgs::msg::PointStamped point_camera, point_map;
        visualization_msgs::msg::Marker marker;

        std::string toFrameRel = "map";
        std::string fromFrameRel = (*msg).header.frame_id;

        point_camera.header.frame_id = fromFrameRel;
        point_camera.header.stamp = now;
        point_camera.point.x = coefficients_cylinder->values[0];
        point_camera.point.y = coefficients_cylinder->values[1];
        point_camera.point.z = marker_height;

        try {
            auto tss = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, now);
            tf2::doTransform(point_camera, point_map, tss);
        } catch (tf2::TransformException& ex) {
            std::cout << ex.what() << std::endl;
            break;
        }

        // Reject flat objects: compute Z spread of inlier points in camera frame
        float z_min = std::numeric_limits<float>::max();
        float z_max = std::numeric_limits<float>::lowest();
        for (const auto& pt : *cloud_cylinder) {
            if (pt.z < z_min) z_min = pt.z;
            if (pt.z > z_max) z_max = pt.z;
        }
        float z_spread = z_max - z_min;

        // For horizontal detections, reject if axis center is at floor level (curbs, ledges)
        float axis_center_z = coefficients_cylinder->values[2];
        bool horizontal_height_ok = is_vertical || (axis_center_z >= min_horizontal_axis_z);

        // accept cylinders within margin
        if ((std::abs(detected_radius - target_radius) <= error_margin) && (cylinder_points_count>=min_cylinder_size) && (z_spread >= min_z_spread) && (z_spread <= max_z_spread) && horizontal_height_ok) {

            if (verbose) {
                std::cerr << "Cylinder radius: " << detected_radius << std::endl;
                std::cout << "Cylinder_points_count: " << cylinder_points_count << std::endl;
            }

            // Publish marker
            marker.header.frame_id = "map";
            marker.header.stamp = now;
            marker.ns = "cylinder";
            marker.id = marker_id++;

            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = point_map.point.x;
            marker.pose.position.y = point_map.point.y;
            marker.pose.position.z = marker_height/2;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = detected_radius * 2;
            marker.scale.y = detected_radius * 2;
            marker.scale.z = marker_height;

            // Compute average colour from cylinder point cloud
            float sum_r = 0.0f, sum_g = 0.0f, sum_b = 0.0f;
            for (const auto& pt : *cloud_cylinder) {
                sum_r += static_cast<float>(pt.r);
                sum_g += static_cast<float>(pt.g);
                sum_b += static_cast<float>(pt.b);
            }
            marker.color.r = sum_r / cylinder_points_count / 255.0f;
            marker.color.g = sum_g / cylinder_points_count / 255.0f;
            marker.color.b = sum_b / cylinder_points_count / 255.0f;
            marker.color.a = 1.0f;

            // Encode orientation in marker.text — read by cylinder_localizator
            marker.text = is_vertical ? "vertical" : "horizontal";

            marker.lifetime = rclcpp::Duration(0, 1);

            marker_pub->publish(marker);

            // Publish cylinder cloud
            sensor_msgs::msg::PointCloud2 cylinder_msg;
            pcl::PCLPointCloud2::Ptr pcl_out(new pcl::PCLPointCloud2());
            pcl::toPCLPointCloud2(*cloud_cylinder, *pcl_out);
            pcl_conversions::fromPCL(*pcl_out, cylinder_msg);
            *all_cylinders += *cloud_cylinder;
            detected_cylinders++;
        }

        // Remove extracted cylinder from cloud
        extract.setNegative(true);
        pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>());
        extract.filter(*temp_cloud);

        pcl::ExtractIndices<pcl::Normal> extract_normals_iter;
        extract_normals_iter.setInputCloud(remaining_normals);
        extract_normals_iter.setIndices(inliers_cylinder);
        extract_normals_iter.setNegative(true);

        pcl::PointCloud<pcl::Normal>::Ptr temp_normals(new pcl::PointCloud<pcl::Normal>());
        extract_normals_iter.filter(*temp_normals);

        remaining_cloud.swap(temp_cloud);
        remaining_normals.swap(temp_normals);
    }

    if (detected_cylinders > 0) {
        std::cout << "Detected " << detected_cylinders << " cylinders." << std::endl;
    }

    // publish cylinder-filtered point cloud
    if (!all_cylinders->empty()) {
        sensor_msgs::msg::PointCloud2 cylinder_msg;
        pcl::PCLPointCloud2::Ptr pcl_out(new pcl::PCLPointCloud2());

        pcl::toPCLPointCloud2(*all_cylinders, *pcl_out);
        pcl_conversions::fromPCL(*pcl_out, cylinder_msg);

        cylinder_msg.header = msg->header;  // preserve frame + timestamp
        cylinder_pub->publish(cylinder_msg);
    }    
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    std::cout << "cylinder_segmentation started" << std::endl;

    node = rclcpp::Node::make_shared("cylinder_segmentation");

    // create subscriber
    node->declare_parameter<std::string>("topic_pointcloud_in", "/oakd/rgb/preview/depth/points");
    std::string param_topic_pointcloud_in = node->get_parameter("topic_pointcloud_in").as_string();
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_in, 10, &cloud_cb);

    // setup tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // create publishers
    planes_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_point_cloud", 1);
    cylinder_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("cylinder_point_cloud", 1);
    marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("cylinder_markers", 1);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}