#include <ros/ros.h>

#include <continuous_tracking/ContinuousTrackingConfig.h>
#include <continuous_tracking/dataset.h>
#include <dynamic_reconfigure/server.h>
#include <list>
#include <object_instance_msgs/ObjectInstance3DArray.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

namespace continuous_tracking
{

struct LocalRangeImagePoint
{
    Eigen::Vector3f position{std::nanf(""), std::nanf(""), std::nanf("")};
    float distance{std::nanf("")};
};

struct ContinuousRangeImagePoint
{
    Eigen::Vector3f position{std::nanf(""), std::nanf(""), std::nanf("")};
    float distance{std::nanf("")};
    bool is_ground_point{false};
    bool is_ignored_for_clustering{false};
    int64_t cluster_id{0};
};

struct Line
{
    Eigen::Vector3f start_position;
    Eigen::Vector3f end_position;
    float length{0};
    int number_of_inliers{0};
};

class Centroid
{
  public:
    explicit Centroid(std::list<sensor_msgs::PointCloud2::ConstPtr> msgs = {}) : msgs(std::move(msgs))
    {
        update();
    }

    Eigen::Vector3d getPosition() const
    {
        return mean_position;
    }

    ros::Time getStamp() const
    {
        return mean_stamp;
    }

    ros::Time getMinStamp() const
    {
        return min_stamp;
    }

    ros::Time getMaxStamp() const
    {
        return max_stamp;
    }

    void merge(Centroid& other)
    {
        msgs.insert(msgs.end(), other.msgs.begin(), other.msgs.end());
        update();
    }

    const std::list<sensor_msgs::PointCloud2::ConstPtr>& getMessages() const
    {
        return msgs;
    }

    void update()
    {
        Eigen::Vector3d accumulated_position = {0., 0., 0.};
        int num_points = 0;
        min_stamp = ros::Time{std::numeric_limits<int32_t>::max(), 0};
        max_stamp = ros::Time{0, 0};

        for (const auto& msg : msgs)
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(*msg, "z");
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_time_sec_in(*msg, "time_sec");
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_time_nsec_in(*msg, "time_nsec");

            for (; iter_x_in != iter_x_in.end();
                 ++iter_x_in, ++iter_y_in, ++iter_z_in, ++iter_time_sec_in, ++iter_time_nsec_in)
            {
                Eigen::Vector3d p{*iter_x_in, *iter_y_in, *iter_z_in};
                accumulated_position += p;
                num_points++;

                ros::Time stamp{*iter_time_sec_in, *iter_time_nsec_in};
                if (stamp < min_stamp)
                    min_stamp = stamp;
                if (stamp > max_stamp)
                    max_stamp = stamp;
            }
        }
        mean_position = accumulated_position / num_points;
        mean_stamp = min_stamp + ros::Duration(max_stamp - min_stamp) * 0.5;
    }

    bool isInitialized() const
    {
        return !msgs.empty();
    }

  private:
    std::list<sensor_msgs::PointCloud2::ConstPtr> msgs;
    ros::Time mean_stamp{0, 0};
    ros::Time min_stamp{0, 0};
    ros::Time max_stamp{0, 0};
    Eigen::Vector3d mean_position;
};

struct LocalRangeImage
{
    std::vector<LocalRangeImagePoint> points;

    std::vector<int64_t> cluster_ids;
    int width{0};
    int height{0};
    int64_t min_global_column_index{0};
    int64_t max_global_column_index{0};
    uint16_t min_row_index{0};
    uint16_t max_row_index{0};
};

struct MessageWithLocalRangeImage
{
    sensor_msgs::PointCloud2::ConstPtr msg;
    std::shared_ptr<LocalRangeImage> local_range_image;
};

class Track
{
  public:
    int64_t id{0};
    uint16_t class_label{133};
    ros::Time last_update;
    ros::Time first_update;
    Centroid centroid;
    std::list<Centroid> centroid_history;
    Centroid initial_position;
    float yaw{std::nanf("")};
    float tentative_yaw{std::nanf("")};
    float maximum_seen_bounding_box_edge_length{0};
    friend bool operator==(const Track& t1, const Track& t2);
    friend bool operator!=(const Track& t1, const Track& t2);
};

bool operator==(const Track& t1, const Track& t2)
{
    return (t1.id == t2.id);
}

bool operator!=(const Track& t1, const Track& t2)
{
    return (t1.id == t2.id);
}

class ContinuousTrackingNode
{
  public:
    explicit ContinuousTrackingNode(ros::NodeHandle nh, const ros::NodeHandle& private_nh)
        : t(ros::TransportHints().tcp().tcpNoDelay(true)),
          reset_sub_(nh.subscribe("reset", 1, &ContinuousTrackingNode::callbackReset, this, t)),
          lidar_detections_sub_(
              nh.subscribe("lidar_detections", 10000, &ContinuousTrackingNode::callbackLidarDetections, this, t)),
          continuous_ground_point_segmentation_sub_(
              nh.subscribe("continuous_ground_point_segmentation",
                           10000,
                           &ContinuousTrackingNode::callbackContinuousGroundPointSegmentation,
                           this,
                           t)),
          tf_buffer_(),
          tf_listener_(tf_buffer_, nh, true, t)
    {
        tf_buffer_.setUsingDedicatedThread(true);
        pub_measurement_ = nh.advertise<visualization_msgs::Marker>("mean", 10000);
        pub_lines_ = nh.advertise<visualization_msgs::MarkerArray>("lines", 10000);
        pub_update_ = nh.advertise<object_instance_msgs::ObjectInstance3DArray>("update", 10000);
        pub_tracks_ = nh.advertise<visualization_msgs::Marker>("predict", 10000);

        reconfigure_server_.setCallback([this](ContinuousTrackingConfig& config, uint32_t level)
                                        { callbackReconfigure(config, level); });
    }

    void init()
    {
    }

    void reset()
    {
        tracks_.clear();
        track_id_counter_ = 0;
        deferred_msgs_.clear();
    }

  private:
    void callbackReset(const std_msgs::Empty::ConstPtr& msg)
    {
        ROS_WARN_STREAM("Reset because external reset message arrived.");
        reset();
    }

    inline bool isLocalRangeImageWithEdgeCompletelyInFullRangeImage(const std::shared_ptr<LocalRangeImage>& img) const
    {
        return img->max_global_column_index < full_range_image_global_column_index_end;
    }

    void callbackContinuousGroundPointSegmentation(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        double dt = (msg->header.stamp - last_message_stamp2).toSec();
        if (std::abs(dt) > 0.2)
        {
            // reset continuous range image
            full_range_image.resize(full_range_image_height * full_range_image_max_width, ContinuousRangeImagePoint());
            full_range_image_global_column_index_start = 0;
            full_range_image_global_column_index_end = -1;
            full_range_image_local_column_index_end = 0;
        }
        last_message_stamp2 = msg->header.stamp;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_d_in(*msg, "distance");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_label_in(*msg, "ground_point_label");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_ignored_in(*msg, "ignore_for_clustering");
        sensor_msgs::PointCloud2ConstIterator<double> iter_column_index_in(*msg, "global_column_index");

        for (int message_column_index = 0; message_column_index < msg->width; message_column_index++)
        {
            // get initial global column index
            if (full_range_image_global_column_index_end == -1)
            {
                full_range_image_global_column_index_start =
                    static_cast<int64_t>(*(iter_column_index_in + message_column_index));
                full_range_image_global_column_index_end = full_range_image_global_column_index_start;
                full_range_image_local_column_index_end = 0;
            }
            else if (full_range_image_global_column_index_end !=
                     static_cast<int64_t>(*(iter_column_index_in + message_column_index)))
            {
                throw std::runtime_error(
                    "Out-Of-Sequence Column detected (expected/actual): " +
                    std::to_string(full_range_image_global_column_index_end) + ", " +
                    std::to_string(static_cast<int64_t>(*(iter_column_index_in + message_column_index))));
            }

            for (int row_index = 0; row_index < msg->height; row_index++)
            {
                // by PointCloud2 definition the points are stored row-wise
                int data_index_message = row_index * static_cast<int>(msg->width) + message_column_index;

                // the continuous range image stores the range image column-wise (for efficient deletion)
                int data_index_continuous_range_image =
                    full_range_image_local_column_index_end * full_range_image_height + row_index;

                ContinuousRangeImagePoint& out = full_range_image[data_index_continuous_range_image];
                out.position.x() = *(iter_x_in + data_index_message);
                out.position.y() = *(iter_y_in + data_index_message);
                out.position.z() = *(iter_z_in + data_index_message);
                out.distance = *(iter_d_in + data_index_message);
                out.is_ground_point = (*(iter_label_in + data_index_message) == 54); // TODO: remove magic numbers
                out.is_ignored_for_clustering = (*(iter_ignored_in + data_index_message) == 9);
                out.cluster_id = 0;
            }

            full_range_image_global_column_index_end++;
            if (full_range_image_global_column_index_end - full_range_image_global_column_index_start >
                full_range_image_max_width)
                full_range_image_global_column_index_start++;

            full_range_image_local_column_index_end++;
            if (full_range_image_local_column_index_end == full_range_image_max_width)
                full_range_image_local_column_index_end = 0;
        }

        // check if we have deferred clusters which couldn't be processed so far because the continuous range image was
        // not far enough
        auto it = deferred_msgs_.begin();
        while (it != deferred_msgs_.end())
        {
            if (isLocalRangeImageWithEdgeCompletelyInFullRangeImage(it->local_range_image))
            {
                processCluster(*it);
                it = deferred_msgs_.erase(it);
            }
            else
                break;
        }
    }

    inline int getLocalColumnIndexFromGlobalColumnIndex(int64_t global_column_index) const
    {
        if (global_column_index < full_range_image_global_column_index_start ||
            global_column_index >= full_range_image_global_column_index_end)
            throw std::runtime_error("Index-Out-Of-Bounds error for continuous range image");

        int out = full_range_image_local_column_index_end -
                  static_cast<int>(full_range_image_global_column_index_end - global_column_index);
        if (out < 0)
            out += full_range_image_max_width;

        return out;
    }

    void callbackLidarDetections(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        double dt = (msg->header.stamp - last_message_stamp).toSec();
        if (std::abs(dt) > 0.2)
            reset();
        last_message_stamp = msg->header.stamp;

        // generate local range image of cluster (and get min/max indices)
        auto local_img = generateLocalRangeImageForCluster({msg});

        MessageWithLocalRangeImage message_with_range_image;
        message_with_range_image.local_range_image = local_img;
        message_with_range_image.msg = msg;

        // unfortunately detections can arrive even faster than a column of ground point segmentation. Therefore, we
        // check if the local range image is completely part of the continuous range image. If not we have to defer that
        // message.
        if (!isLocalRangeImageWithEdgeCompletelyInFullRangeImage(local_img))
        {
            auto it = std::lower_bound(deferred_msgs_.begin(),
                                       deferred_msgs_.end(),
                                       message_with_range_image,
                                       [](const MessageWithLocalRangeImage& a, const MessageWithLocalRangeImage& b) {
                                           return a.local_range_image->min_global_column_index <
                                                  b.local_range_image->min_global_column_index;
                                       });
            deferred_msgs_.insert(it, message_with_range_image);
        }
        else
            processCluster(message_with_range_image);
    }

    void processCluster(const MessageWithLocalRangeImage& message_with_range_image)
    {
        auto msg = message_with_range_image.msg;
        auto local_img = message_with_range_image.local_range_image;

        // get cluster id of this cluster
        auto cluster_id = static_cast<int32_t>(getClusterIdOfMessage(msg));

        // get centroid for new message
        Centroid current_detection({msg});

        // visualize detection
        visualizePoints({current_detection.getPosition()},
                        msg->header,
                        pub_measurement_,
                        cluster_id,
                        0xFFFF0000,
                        "detection_centroid",
                        .5f);

        // find straight lines in this local range image
        auto lines = findLinesInCluster(local_img->points,
                                        local_img->width,
                                        local_img->height,
                                        config_.row_window_size,
                                        config_.row_window_stride,
                                        config_.find_line_direction);
        visualizeLines(lines, msg->header, pub_lines_, cluster_id, 0xFFFF0000, "cluster_line");

        // also store local range image into full continuous range image
        insertClusterIdIntoContinuousRangeImage(msg);

        // delete tracks that are too old
        auto it = tracks_.begin();
        while (it != tracks_.end())
        {
            if ((msg->header.stamp - it->last_update).toSec() > config_.max_survival_duration_without_update)
            {
                // erase
                object_instance_msgs::ObjectInstance3DArray erase_msg;
                erase_msg.header = msg->header;
                erase_msg.instances.resize(1);
                erase_msg.instances[0].id = it->id;
                erase_msg.instances[0].bounding_box_pose.position.x = std::nan("");
                pub_update_.publish(erase_msg);
                it = tracks_.erase(it);
            }
            else
                it++;
        }

        // try to find best track
        double min_distance_squared = std::numeric_limits<float>::max();
        Track* best_track = nullptr;
        int track_relative_rotation_index = 0;
        for (auto& track : tracks_)
        {
            double time_diff = (msg->header.stamp - track.last_update).toSec();
            int relative_rotation_index = static_cast<int>(std::round(time_diff / 0.1)); // TODO: remove magic number
            if (relative_rotation_index < 0)
            {
                // throw std::runtime_error("Way to old detection arrived: " + std::to_string(time_diff));
                return;
            }

            Eigen::Vector3d predicted_position =
                predictMultipleTimePoints(track, {0.1 * relative_rotation_index}).front();

            double distance_squared =
                (current_detection.getPosition().head<2>() - predicted_position.head<2>()).squaredNorm();

            if (distance_squared < min_distance_squared)
            {
                min_distance_squared = distance_squared;
                best_track = &track;
                track_relative_rotation_index = relative_rotation_index;
            }
        }

        Track* updated_track;
        if (best_track != nullptr && min_distance_squared < config_max_centroid_distance_squared_)
        {
            if (track_relative_rotation_index == 0)
            {
                // update detection cluster mean
                best_track->centroid.merge(current_detection);
            }
            else
            {
                // we have a new rotation

                // if initial position is not set yet -> set it
                if (!best_track->initial_position.isInitialized())
                    best_track->initial_position = best_track->centroid;

                // take the final yaw obtained from the previous rotation (now we know that all clusters are
                // associated)
                if (!std::isnan(best_track->tentative_yaw))
                    best_track->yaw = best_track->tentative_yaw;

                // put previously updated position in history and erase old entries
                best_track->centroid_history.push_back(best_track->centroid);
                while ((current_detection.getStamp() - best_track->centroid_history.front().getStamp()).toSec() >
                           config_.max_history_age &&
                       best_track->centroid_history.size() > 1)
                    best_track->centroid_history.pop_front();

                best_track->centroid = current_detection;
            }

            best_track->last_update = msg->header.stamp;
            updated_track = best_track;
        }
        else
        {
            Track new_track;
            new_track.id = track_id_counter_++;
            new_track.class_label = getClassCount() - 1; // unknown
            new_track.centroid = current_detection;
            new_track.last_update = msg->header.stamp;
            new_track.first_update = msg->header.stamp;
            // new_track.initial_position is set when a full rotation is finished
            tracks_.push_back(new_track);
            updated_track = &tracks_.back();
        }

        // visualize updated centroid
        visualizePoints({updated_track->centroid.getPosition()},
                        msg->header,
                        pub_tracks_,
                        static_cast<int32_t>(updated_track->id),
                        0xFF00FFFF,
                        "updated_centroid",
                        .5f);

        // visualize previous centroids
        std::vector<Eigen::Vector3d> updated_centroid_history(0);
        updated_centroid_history.reserve(updated_track->centroid_history.size());
        for (const auto& centroid : updated_track->centroid_history)
            updated_centroid_history.push_back(centroid.getPosition());
        visualizePoints(updated_centroid_history,
                        msg->header,
                        pub_tracks_,
                        static_cast<int32_t>(updated_track->id),
                        0xFF000000,
                        "updated_centroid_history",
                        .3f);

        // visualize predictions
        auto predicted_centroids = predictMultipleTimePoints(*updated_track, {0.1, 0.2, 0.3, 0.4, 0.5});
        visualizePoints(predicted_centroids,
                        msg->header,
                        pub_tracks_,
                        static_cast<int32_t>(updated_track->id),
                        0xFFFFFFFF,
                        "predicted_centroids",
                        .3f);

        // declare bounding box parameters
        Eigen::Vector3f box_center, box_dimensions;

        float yaw_by_line = calculateYawByLongestLine(lines);
        float yaw_by_motion = std::nanf("");
        if (std::isnan(updated_track->yaw))
        {
            // try to fit a single frame bounding box as good as possible
            float yaw = !std::isnan(yaw_by_line) ? yaw_by_line : calculateYawByCovariance(msg);
            fitBoundingBox(box_center, box_dimensions, yaw, updated_track->centroid.getMessages());

            // keep track of the maximum edge length seen so far for this track
            updated_track->maximum_seen_bounding_box_edge_length = std::max(
                updated_track->maximum_seen_bounding_box_edge_length, std::max(box_dimensions.x(), box_dimensions.y()));

            // check if the centroid has moved enough to init yaw of track
            float threshold = static_cast<float>(std::max(config_.minimum_absolute_centroid_motion_for_init,
                                                          config_.minimum_relative_centroid_motion_for_init *
                                                              updated_track->maximum_seen_bounding_box_edge_length));
            yaw_by_motion = calculateYawByTrackMotion(*updated_track, threshold, true);
            if (!std::isnan(yaw_by_motion))
            {
                if (!std::isnan(yaw_by_line))
                {
                    // snap yaw_by_motion to the orientation of the longest line
                    updated_track->tentative_yaw = snapYawToLine(yaw_by_motion, yaw_by_line);
                }
                else
                {
                    // simply use yaw of initial motion
                    updated_track->tentative_yaw = yaw_by_motion;
                }
            }
        }
        else
        {
            // update yaw
            if (!std::isnan(yaw_by_line))
            {
                // snap track's yaw to the orientation of the longest line
                updated_track->tentative_yaw = snapYawToLine(updated_track->yaw, yaw_by_line);
            }
            else
            {
                // simply use recent track motion (but only if it fits to current track motion)
                yaw_by_motion = calculateYawByTrackMotion(
                    *updated_track, static_cast<float>(config_.minimum_centroid_motion_for_update), false);
                if (!std::isnan(yaw_by_motion))
                    updated_track->tentative_yaw = yaw_by_motion;
            }
        }

        bool bounding_box_enabled;
        switch (config_.bounding_box_enable_if)
        {
            case ContinuousTracking_Yaw_Initialized:
                bounding_box_enabled = !std::isnan(updated_track->yaw);
                break;
            case ContinuousTracking_Has_Line:
                bounding_box_enabled = !std::isnan(yaw_by_line);
                break;
            case ContinuousTracking_Had_Previous_Motion:
                bounding_box_enabled = !std::isnan(yaw_by_motion);
                break;
            case ContinuousTracking_Always:
                bounding_box_enabled = true;
                break;
            default:
                bounding_box_enabled = false;
                break;
        }

        // fit bounding box to yaw of track
        tf2::Quaternion q = tf2::Quaternion::getIdentity();
        if (bounding_box_enabled)
        {
            float bounding_box_yaw;
            switch (config_.bounding_box_yaw_by)
            {
                case ContinuousTracking_Track_Yaw:
                    bounding_box_yaw = updated_track->yaw;
                    break;
                case ContinuousTracking_Longest_Line:
                    bounding_box_yaw = yaw_by_line; // Todo: track based
                    break;
                case ContinuousTracking_Principal_Component:
                    bounding_box_yaw = calculateYawByCovariance(msg); // Todo: track based
                    break;
                case ContinuousTracking_Previous_Motion:
                    bounding_box_yaw = yaw_by_motion;
                    break;
                default:
                    bounding_box_yaw = 0;
                    break;
            }

            // due to combination of configs it can happen that bounding box is enabled but yaw is NaN -> set to
            // zero
            if (std::isnan(bounding_box_yaw))
                bounding_box_yaw = 0;

            fitBoundingBox(box_center,
                           box_dimensions,
                           bounding_box_yaw,
                           config_.bounding_box_by == ContinuousTracking_Cluster ?
                               std::list<sensor_msgs::PointCloud2::ConstPtr>({msg}) :
                               updated_track->centroid.getMessages());
            q.setRPY(0, 0, bounding_box_yaw);
        }

        // evaluate whether to hide track
        float longest_side = std::max(box_dimensions.x(), box_dimensions.y());
        bool hide_because_too_flat = box_dimensions.z() <= config_.hide_flat_track_max_height;
        bool hide_because_too_long = longest_side >= config_.hide_long_track_min_length;
        bool hide_because_too_large_base = box_dimensions.x() >= config_.hide_large_base_track_min_edge_length &&
                                           box_dimensions.y() >= config_.hide_large_base_track_min_edge_length;
        bool hide_because_too_long_and_high = longest_side >= config_.hide_long_and_high_track_min_length &&
                                              box_dimensions.z() >= config_.hide_long_and_high_track_min_height;
        bool hide_because_too_long_and_flat = longest_side >= config_.hide_long_and_flat_track_min_length &&
                                              box_dimensions.z() <= config_.hide_long_and_flat_track_max_height;
        bool hide = hide_because_too_flat | hide_because_too_long | hide_because_too_large_base |
                    hide_because_too_long_and_high | hide_because_too_long_and_flat;

        // visualize track
        object_instance_msgs::ObjectInstance3DArray instances_msg;
        instances_msg.header = msg->header;
        instances_msg.instances.resize(1);
        auto& instance_msg = instances_msg.instances[0];
        instance_msg.id = updated_track->id;
        instance_msg.class_index = updated_track->class_label;
        instance_msg.class_count = getClassCount();
        instance_msg.class_name = getClassName(updated_track->class_label);
        instance_msg.class_probabilities = {1.f};
        instance_msg.is_instance = !hide;
        instance_msg.point_cloud = *mergePointCloud2Messages(updated_track->centroid.getMessages());
        instance_msg.bounding_box_pose.position.x = box_center.x();
        instance_msg.bounding_box_pose.position.y = box_center.y();
        instance_msg.bounding_box_pose.position.z = box_center.z();
        if (bounding_box_enabled)
        {
            instance_msg.bounding_box_size.x = box_dimensions.x();
            instance_msg.bounding_box_size.y = box_dimensions.y();
            instance_msg.bounding_box_size.z = box_dimensions.z();
            instance_msg.bounding_box_pose.orientation = tf2::toMsg(q);
        }
        else
        {
            instance_msg.bounding_box_size.x = 0;
            instance_msg.bounding_box_size.y = 0;
            instance_msg.bounding_box_size.z = 0;
        }
        pub_update_.publish(instances_msg);
    }

    static std::vector<Eigen::Vector3d> predictMultipleTimePoints(const Track& track, std::vector<double> dt)
    {
        std::vector<Eigen::Vector3d> predictions(dt.size(), track.centroid.getPosition());
        if (!track.centroid_history.empty())
        {
            double time_diff = (track.centroid.getStamp() - track.centroid_history.front().getStamp()).toSec();
            if (time_diff <= 0)
                throw std::runtime_error("Unexpected negative/zero prediction: " + std::to_string(time_diff));
            Eigen::Vector3d new_position = track.centroid.getPosition();
            Eigen::Vector3d old_position = track.centroid_history.front().getPosition();

            Eigen::Vector3d direction = new_position - old_position;
            for (int i = 0; i < dt.size(); i++)
                predictions[i] = new_position + direction * (dt[i] / time_diff);
        }
        return predictions;
    }

    static float snapYawToLine(float yaw, float yaw_by_line)
    {
        return static_cast<float>(yaw_by_line + std::round((yaw - yaw_by_line) / (M_PI / 2)) * (M_PI / 2));
    }

    static float calculateYawByLongestLine(const std::vector<Line>& lines)
    {
        Line longest_line;
        if (!lines.empty())
        {
            for (auto& line : lines)
                if (line.length > longest_line.length)
                    longest_line = line;

            Eigen::Vector2f v = longest_line.end_position.head<2>() - longest_line.start_position.head<2>();
            return std::atan2(v.y(), v.x());
        }
        return std::nanf("");
    }

    static float
    calculateYawByTrackMotion(const Track& track, float minimum_centroid_travel_distance, bool initial = false)
    {
        if (track.initial_position.isInitialized())
        {
            Eigen::Vector3d new_position = track.centroid.getPosition();
            Eigen::Vector3d old_position =
                initial ? track.initial_position.getPosition() : track.centroid_history.front().getPosition();

            Eigen::Vector2f direction = (new_position - old_position).head<2>().cast<float>();

            if (direction.norm() >= minimum_centroid_travel_distance)
                return std::atan2(direction.y(), direction.x());
        }
        return std::nanf("");
    }

    static float calculateYawByCovariance(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        Eigen::MatrixXf mat;
        mat.resize(msg->width * msg->height, 2);
        sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*msg, "y");
        for (int i = 0; iter_x_in != iter_x_in.end(); ++iter_x_in, ++iter_y_in, ++i)
            mat.row(i) = Eigen::Vector2f{*iter_x_in, *iter_y_in};
        Eigen::MatrixXf centered = mat.rowwise() - mat.colwise().mean();
        Eigen::MatrixXf cov = (centered.adjoint() * centered) / double(mat.rows() - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(cov);
        if (eigen_solver.info() == Eigen::Success)
        {
            // Eigen::Vector2f eigen_values = eigen_solver.eigenvalues();
            const Eigen::Matrix2f& eigen_vectors = eigen_solver.eigenvectors();
            return Eigen::Rotation2Df(eigen_vectors).angle();
        }
        return std::nanf("");
    }

    static std::shared_ptr<LocalRangeImage>
    generateLocalRangeImageForCluster(const std::list<sensor_msgs::PointCloud2::ConstPtr>& msgs)
    {
        // init local range image
        std::shared_ptr<LocalRangeImage> img = std::make_shared<LocalRangeImage>();
        img->min_global_column_index = std::numeric_limits<int64_t>::max();
        img->max_global_column_index = 0;
        img->min_row_index = std::numeric_limits<uint16_t>::max();
        img->max_row_index = 0;

        // find min/max of row/column index to get size of local range image
        for (const auto& msg : msgs)
        {
            sensor_msgs::PointCloud2ConstIterator<double> iter_gc_in(*msg, "global_column_index");
            sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r_in(*msg, "row_index");
            for (; iter_gc_in != iter_gc_in.end(); ++iter_gc_in, ++iter_r_in)
            {
                auto global_column_index = static_cast<int64_t>(*iter_gc_in);
                uint16_t row_index = *iter_r_in;

                if (global_column_index > img->max_global_column_index)
                    img->max_global_column_index = global_column_index;
                if (global_column_index < img->min_global_column_index)
                    img->min_global_column_index = global_column_index;

                if (row_index > img->max_row_index)
                    img->max_row_index = row_index;
                if (row_index < img->min_row_index)
                    img->min_row_index = row_index;
            }
        }
        img->width = static_cast<int>(img->max_global_column_index - img->min_global_column_index) + 1;
        img->height = img->max_row_index - img->min_row_index + 1;

        // generate local range image by inserting all points of the cluster
        img->points.resize(img->width * img->height, LocalRangeImagePoint());
        for (const auto& msg : msgs)
        {
            sensor_msgs::PointCloud2ConstIterator<double> iter_gc_in{*msg, "global_column_index"};
            sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r_in{*msg, "row_index"};
            sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(*msg, "z");
            sensor_msgs::PointCloud2ConstIterator<float> iter_d_in(*msg, "distance");
            for (; iter_x_in != iter_x_in.end();
                 ++iter_gc_in, ++iter_r_in, ++iter_x_in, ++iter_y_in, ++iter_z_in, ++iter_d_in)
            {
                // get relative indices
                auto local_column_index =
                    static_cast<int>(static_cast<int64_t>(*iter_gc_in) - img->min_global_column_index);
                uint16_t row_index = *iter_r_in - img->min_row_index;

                // save the points in row-major order
                LocalRangeImagePoint& p = img->points[row_index * img->width + local_column_index];
                p.position.x() = *iter_x_in;
                p.position.y() = *iter_y_in;
                p.position.z() = *iter_z_in;
                p.distance = *iter_d_in;
            }
        }

        return img;
    }

    static int64_t getClusterIdOfMessage(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (msg->data.empty())
            throw std::runtime_error("Unable to obtain cluster ID because message is empty");
        sensor_msgs::PointCloud2ConstIterator<double> iter_id(*msg, "id");
        auto cluster_id = static_cast<int64_t>(*iter_id);
        if (cluster_id == 0)
            throw std::runtime_error("Unable to obtain cluster ID because first point has invalid cluster ID");
        return cluster_id;
    }

    void insertClusterIdIntoContinuousRangeImage(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        bool cluster_older_than_range_image_horizon = false;

        sensor_msgs::PointCloud2ConstIterator<double> iter_gc(*msg, "global_column_index");
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "row_index");
        sensor_msgs::PointCloud2ConstIterator<double> iter_id(*msg, "id");
        for (; iter_gc != iter_gc.end(); ++iter_gc, ++iter_r, ++iter_id)
        {
            // store it also into continuous range image (column major)
            auto global_column_index = static_cast<int64_t>(*iter_gc);
            uint16_t row_index = *iter_r;
            auto cluster_id = static_cast<int64_t>(*iter_id);
            if (global_column_index < full_range_image_global_column_index_start)
            {
                cluster_older_than_range_image_horizon = true;
                continue;
            }
            else if (global_column_index >= full_range_image_global_column_index_end)
            {
                std::cout << "Cluster contains points with global column index not in the continuous range image yet: "
                          << global_column_index << ", " << full_range_image_global_column_index_end << std::endl;
                continue;
            }
            int local_column_index = getLocalColumnIndexFromGlobalColumnIndex(global_column_index);
            ContinuousRangeImagePoint& p = full_range_image[local_column_index * full_range_image_height + row_index];
            p.cluster_id = cluster_id;
        }

        if (cluster_older_than_range_image_horizon)
            std::cout << "Warning: Cluster contains points which are older than continuous range image horizon!"
                      << std::endl;
    }

    std::vector<Line> findLinesInCluster(const std::vector<LocalRangeImagePoint>& cluster_as_range_image,
                                         int width,
                                         int height,
                                         int row_window_size,
                                         int row_window_stride,
                                         int find_line_direction = 1) const
    {
        if (row_window_size <= 0 || row_window_stride <= 0)
            throw std::runtime_error("row_window_size & row_window_stride are not allowed to be zero or negative");

        // first & last row index based on row window size (so we don't get an index-out-of-bounds exception)
        const int row_window_steps_up = row_window_size / 2;
        const int row_window_steps_down = (row_window_size - 1) / 2;
        const int first_row_index = row_window_steps_up;
        const int height_considering_row_window = height - row_window_steps_down;

        std::vector<Line> lines;
        for (int row_index = first_row_index; row_index < height_considering_row_window; row_index += row_window_stride)
        {
            int line_start_column_index = find_line_direction == 1 ? 0 : (width - 1);
            while (line_start_column_index >= 0 && line_start_column_index < width)
            {
                // skip if NaN or not obstacle
                const Eigen::Vector3f& line_start_point =
                    cluster_as_range_image[row_index * width + line_start_column_index].position;
                if (std::isnan(line_start_point.x()))
                {
                    line_start_column_index += find_line_direction;
                    continue;
                }

                // these variables store stats about the last line with current start column index
                float last_line_length = 0;
                int last_line_number_of_inliers = 0;
                int last_line_horizontal_step_count = 0;
                Eigen::Vector3f last_line_end_point;

                // do forward search for a straight line
                int line_end_nan_counter = 0;
                int windows_with_no_inliers_counter = 0;
                int line_end_column_index =
                    line_start_column_index + config_.line_end_min_columns_jump_ahead * find_line_direction;
                while (line_end_column_index >= 0 && line_end_column_index < width)
                {
                    // get end point of line
                    const Eigen::Vector3f& line_end_point =
                        cluster_as_range_image[row_index * width + line_end_column_index].position;
                    if (std::isnan(line_end_point.x()))
                    {
                        ++line_end_nan_counter;
                        // if (line_end_nan_counter == 3)
                        //     break; // N consecutive NaNs found -> give up and use next point as a start point
                        line_end_column_index += find_line_direction;
                        continue;
                    }
                    else
                        line_end_nan_counter = 0;

                    // get line between start and end
                    Eigen::Vector2f start_to_end_vec = line_end_point.head<2>() - line_start_point.head<2>();
                    float start_to_end_length = start_to_end_vec.norm(); // TODO: everywhere squared norm for efficiency

                    // only analyze lines longer than X meters -> if shorter increment end point position
                    if (start_to_end_length <= config_.min_line_length)
                    {
                        line_end_column_index += find_line_direction;
                        continue;
                    }

                    // track previous point in order to be able to limit the maximum distance between points
                    Eigen::Vector3f line_prev_inlier_point = line_start_point;

                    // iterate over points between line_start_column_index and line_end_column_index
                    int number_of_inliers = 0;
                    int line_cur_column_index = line_start_column_index;
                    while (line_cur_column_index != (line_end_column_index + find_line_direction))
                    {
                        // check that all points in window are either NaN or close enough to line
                        bool found_outlier = false;
                        Eigen::Vector3f last_inlier_in_window = Eigen::Vector3f::Constant(std::nanf(""));
                        for (int row_index_window = row_index - row_window_steps_up;
                             row_index_window <= row_index + row_window_steps_down;
                             row_index_window++)
                        {
                            const Eigen::Vector3f& line_cur_point =
                                cluster_as_range_image[row_index_window * width + line_cur_column_index].position;
                            if (std::isnan(line_cur_point.x()))
                                continue;

                            // do not evaluate line start and end points (they must be inliers)
                            if (row_index_window == row_index && (line_cur_column_index == line_start_column_index &&
                                                                  line_cur_column_index == line_end_column_index))
                            {
                                number_of_inliers++;
                                continue;
                            }

                            // check if this point is close to line between start and end
                            Eigen::Vector2f point_to_start_vec = line_start_point.head<2>() - line_cur_point.head<2>();
                            float distance = std::abs(start_to_end_vec.x() * point_to_start_vec.y() -
                                                      point_to_start_vec.x() * start_to_end_vec.y());
                            distance /= start_to_end_length;
                            if (distance > config_.max_distance_from_line)
                            {
                                found_outlier = true;
                                break;
                            }
                            else
                            {
                                last_inlier_in_window = line_cur_point;
                                number_of_inliers++;
                            }
                        }

                        // stop current line if an outlier was found
                        if (found_outlier)
                            break;

                        // found no outlier in window but also no inlier
                        if (std::isnan(last_inlier_in_window.x()))
                        {
                            windows_with_no_inliers_counter++;
                            if (windows_with_no_inliers_counter > config_.max_windows_with_no_inliers)
                                break;
                            else
                            {
                                line_cur_column_index += find_line_direction;
                                continue;
                            }
                        }
                        else
                            windows_with_no_inliers_counter = 0;

                        // ensure that the gap to a point from previous horizontal step is not too large
                        if ((last_inlier_in_window.head<2>() - line_prev_inlier_point.head<2>()).norm() >
                            config_.max_distance_between_points)
                            break;

                        line_cur_column_index += find_line_direction;
                        line_prev_inlier_point = last_inlier_in_window;
                    }

                    // check if line is still valid
                    bool last_gap_too_large = (line_end_point.head<2>() - line_prev_inlier_point.head<2>()).norm() >
                                              config_.max_distance_between_points;
                    bool line_does_match_all_criteria =
                        line_cur_column_index == (line_end_column_index + find_line_direction) && !last_gap_too_large;
                    if (!line_does_match_all_criteria)
                        break;

                    // line matches all criteria so far -> save it and continue with next end point
                    last_line_length = start_to_end_length;
                    last_line_number_of_inliers = number_of_inliers;
                    last_line_horizontal_step_count = std::abs(line_end_column_index - line_start_column_index) + 1;
                    last_line_end_point = line_end_point;

                    // if it is already long enough we stop (even when it could be continued) to save CPU cycles
                    if (last_line_number_of_inliers >= config_.min_number_of_inliers &&
                        last_line_length >= config_.max_line_length)
                        break;

                    line_end_column_index += find_line_direction;
                }

                // check if the line, which matched all criteria, is long enough and has enough inliers
                if (last_line_length > config_.min_line_length &&
                    last_line_number_of_inliers >= config_.min_number_of_inliers)
                {
                    // save line
                    Line line;
                    line.start_position = line_start_point;
                    line.end_position = last_line_end_point;
                    line.length = last_line_length;
                    line.number_of_inliers = last_line_number_of_inliers;
                    lines.push_back(line);

                    line_start_column_index += last_line_horizontal_step_count * find_line_direction;
                }
                else
                {
                    line_start_column_index += find_line_direction;
                }
            }
        }

        return lines;
    }

    static void fitBoundingBox(Eigen::Vector3f& box_center,
                               Eigen::Vector3f& box_dimensions,
                               float box_yaw,
                               const std::list<sensor_msgs::PointCloud2::ConstPtr>& instance_msgs)
    {
        float sin_yaw = std::sin(box_yaw);
        float cos_yaw = std::cos(box_yaw);

        // get dimensions of oriented bounding box
        Eigen::Vector3f corner_min = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
        Eigen::Vector3f corner_max = Eigen::Vector3f::Constant(-std::numeric_limits<float>::max());

        for (const auto& instance_msg : instance_msgs)
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*instance_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*instance_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(*instance_msg, "z");
            for (; iter_x_in != iter_x_in.end(); ++iter_x_in, ++iter_y_in, ++iter_z_in)
            {
                Eigen::Vector3f p{*iter_x_in, *iter_y_in, *iter_z_in};
                Eigen::Vector3f p_wrt_box{cos_yaw * p.x() + sin_yaw * p.y(), -sin_yaw * p.x() + cos_yaw * p.y(), p.z()};
                corner_min = corner_min.cwiseMin(p_wrt_box);
                corner_max = corner_max.cwiseMax(p_wrt_box);
            }
        }

        box_dimensions = corner_max - corner_min;
        Eigen::Vector3f box_rotated_center = corner_min + box_dimensions / 2;
        box_center = {cos_yaw * box_rotated_center.x() - sin_yaw * box_rotated_center.y(),
                      sin_yaw * box_rotated_center.x() + cos_yaw * box_rotated_center.y(),
                      box_rotated_center.z()};
    }

    static void visualizePoints(const std::vector<Eigen::Vector3d>& points,
                                const std_msgs::Header& header,
                                ros::Publisher& pub,
                                int id,
                                uint32_t color,
                                const std::string& ns = "none",
                                float diameter = 1.f,
                                int32_t action = visualization_msgs::Marker::ADD)
    {
        if (pub.getNumSubscribers() == 0)
            return;

        visualization_msgs::Marker marker_msg;
        marker_msg.header.frame_id = header.frame_id;
        marker_msg.header.stamp = header.stamp;
        marker_msg.type = visualization_msgs::Marker::SPHERE_LIST;
        marker_msg.action = action;
        marker_msg.id = id;
        marker_msg.ns = ns;
        marker_msg.color.a = static_cast<float>((color >> 24) & 0xFF) / 255;
        marker_msg.color.r = static_cast<float>((color >> 16) & 0xFF) / 255;
        marker_msg.color.g = static_cast<float>((color >> 8) & 0xFF) / 255;
        marker_msg.color.b = static_cast<float>((color >> 0) & 0xFF) / 255;
        marker_msg.pose.orientation.w = 1.0f;
        marker_msg.scale.x = diameter;
        marker_msg.scale.y = diameter;
        marker_msg.scale.z = diameter;
        marker_msg.lifetime = ros::Duration(.1);
        marker_msg.pose.position.x = 0;
        marker_msg.pose.position.y = 0;
        marker_msg.pose.position.z = 0;
        marker_msg.pose.orientation.x = 0;
        marker_msg.pose.orientation.y = 0;
        marker_msg.pose.orientation.z = 0;
        marker_msg.pose.orientation.w = 1;

        for (const auto& point : points)
        {
            geometry_msgs::Point point_msg;
            point_msg.x = point.x();
            point_msg.y = point.y();
            point_msg.z = point.z();
            marker_msg.points.push_back(point_msg);
        }

        if (!points.empty())
            pub.publish(marker_msg);
    }

    static void visualizeLines(const std::vector<Line>& lines,
                               const std_msgs::Header& header,
                               ros::Publisher& pub,
                               int id,
                               uint32_t color,
                               const std::string& ns = "none",
                               float line_width = 0.02f,
                               int32_t action = visualization_msgs::Marker::ADD)
    {
        if (pub.getNumSubscribers() == 0)
            return;

        visualization_msgs::MarkerArray msg;

        // visualize lines
        msg.markers.resize(2);
        auto& line_marker = msg.markers[0];
        line_marker.header = header;
        line_marker.ns = ns + "_lines";
        line_marker.id = id;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.lifetime = ros::Duration(0.1f);
        line_marker.action = action;
        line_marker.pose.position.x = 0;
        line_marker.pose.position.y = 0;
        line_marker.pose.position.z = 0;
        line_marker.pose.orientation.x = 0.0;
        line_marker.pose.orientation.y = 0.0;
        line_marker.pose.orientation.z = 0.0;
        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = 0.02f;
        line_marker.color.a = static_cast<float>((color >> 24) & 0xFF) / 255;
        line_marker.color.r = static_cast<float>((color >> 16) & 0xFF) / 255;
        line_marker.color.g = static_cast<float>((color >> 8) & 0xFF) / 255;
        line_marker.color.b = static_cast<float>((color >> 0) & 0xFF) / 255;

        // visualize ends
        auto& end_marker = msg.markers[1];
        end_marker.header = header;
        end_marker.ns = ns + "_ends";
        end_marker.id = id;
        end_marker.type = visualization_msgs::Marker::POINTS;
        end_marker.lifetime = ros::Duration(0.1f);
        end_marker.action = action;
        end_marker.pose.position.x = 0;
        end_marker.pose.position.y = 0;
        end_marker.pose.position.z = 0;
        end_marker.pose.orientation.x = 0.0;
        end_marker.pose.orientation.y = 0.0;
        end_marker.pose.orientation.z = 0.0;
        end_marker.pose.orientation.w = 1.0;
        end_marker.scale.x = 0.1f;
        end_marker.scale.y = 0.1f;
        end_marker.scale.z = 0.1f;
        end_marker.color.r = 0.0f;
        end_marker.color.g = 0.0f;
        end_marker.color.b = 0.0f;
        end_marker.color.a = 1.0f;

        for (const auto& line : lines)
        {
            geometry_msgs::Point vector_start;
            vector_start.x = line.start_position.x();
            vector_start.y = line.start_position.y();
            vector_start.z = line.start_position.z();
            line_marker.points.push_back(vector_start);

            geometry_msgs::Point vector_end;
            vector_end.x = line.end_position.x();
            vector_end.y = line.end_position.y();
            vector_end.z = line.end_position.z();
            line_marker.points.push_back(vector_end);

            end_marker.points.push_back(vector_end);
        }

        if (!lines.empty())
            pub.publish(msg);
    }

    static sensor_msgs::PointCloud2::ConstPtr
    mergePointCloud2Messages(std::list<sensor_msgs::PointCloud2::ConstPtr> msgs)
    {
        if (msgs.empty())
            return {};

        if (msgs.size() == 1)
            return msgs.front();

        sensor_msgs::PointCloud2::Ptr output_msg(new sensor_msgs::PointCloud2());
        output_msg->header = msgs.front()->header;
        output_msg->height = 1;
        output_msg->width = 0;
        output_msg->is_bigendian = msgs.front()->is_bigendian;
        output_msg->is_dense = msgs.front()->is_dense;
        output_msg->fields = msgs.front()->fields;
        output_msg->point_step = msgs.front()->point_step;
        for (const auto& msg : msgs)
            output_msg->width += msg->width;
        output_msg->row_step = output_msg->width * output_msg->point_step;
        output_msg->data.clear();
        output_msg->data.reserve(output_msg->width * output_msg->height * output_msg->point_step);
        for (const auto& msg : msgs)
            output_msg->data.insert(output_msg->data.end(), msg->data.begin(), msg->data.end());

        return output_msg;
    }

    void callbackReconfigure(ContinuousTrackingConfig& config, uint32_t level)
    {
        ContinuousTrackingConfig old_config = config_;

        config_ = config;
        config_max_centroid_distance_squared_ = config.max_centroid_distance * config.max_centroid_distance;
    }

  private:
    dynamic_reconfigure::Server<ContinuousTrackingConfig> reconfigure_server_;
    ContinuousTrackingConfig config_;
    double config_max_centroid_distance_squared_{};
    ros::TransportHints t;
    ros::Subscriber reset_sub_;
    ros::Subscriber lidar_detections_sub_;
    ros::Subscriber continuous_ground_point_segmentation_sub_;
    ros::Publisher pub_measurement_;
    ros::Publisher pub_update_;
    ros::Publisher pub_lines_;
    ros::Publisher pub_tracks_;
    std::list<MessageWithLocalRangeImage> deferred_msgs_;
    int64_t track_id_counter_{0};
    std::list<Track> tracks_;
    ros::Time last_message_stamp{0};
    ros::Time last_message_stamp2{0};
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;

    // full continuous range image
    std::vector<ContinuousRangeImagePoint> full_range_image; // stores points column-wise
    int64_t full_range_image_global_column_index_start{0};
    int64_t full_range_image_global_column_index_end{-1};
    int full_range_image_local_column_index_end{0};
    int full_range_image_height{128};
    int full_range_image_max_width{17000}; // TODO: remove magic number (more than 2 rotations)
};

} // namespace continuous_tracking

int main(int argc, char** argv)
{
    ros::init(argc, argv, "continuous_tracking");
    continuous_tracking::ContinuousTrackingNode node(ros::NodeHandle(), ros::NodeHandle("~"));
    node.init();
    ros::spin();

    return 0;
}