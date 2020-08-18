/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <fastcdr/Cdr.h>
#include <condition_variable>
#include <queue>

#include "microRTPS_timesync.h"

#include "adc_report_Publisher.h"
#include "airspeed_Publisher.h"
#include "battery_status_Publisher.h"
#include "cpuload_Publisher.h"
#include "distance_sensor_Publisher.h"
#include "estimator_status_Publisher.h"
#include "home_position_Publisher.h"
#include "iridiumsbd_status_Publisher.h"
#include "radio_status_Publisher.h"
#include "satellite_info_Publisher.h"
#include "sensor_baro_Publisher.h"
#include "sensor_combined_Publisher.h"
#include "sensor_selection_Publisher.h"
#include "timesync_Publisher.h"
#include "vehicle_attitude_Publisher.h"
#include "vehicle_odometry_Publisher.h"
#include "vtol_vehicle_status_Publisher.h"
#include "wind_estimate_Publisher.h"
#include "collision_constraints_Publisher.h"
#include "camera_capture_Subscriber.h"
#include "camera_trigger_Subscriber.h"
#include "collision_report_Subscriber.h"
#include "debug_array_Subscriber.h"
#include "debug_key_value_Subscriber.h"
#include "debug_value_Subscriber.h"
#include "debug_vect_Subscriber.h"
#include "obstacle_distance_Subscriber.h"
#include "optical_flow_Subscriber.h"
#include "position_setpoint_Subscriber.h"
#include "position_setpoint_triplet_Subscriber.h"
#include "timesync_Subscriber.h"
#include "trajectory_waypoint_Subscriber.h"
#include "vehicle_trajectory_waypoint_Subscriber.h"
#include "onboard_computer_status_Subscriber.h"
#include "vehicle_mocap_odometry_Subscriber.h"
#include "vehicle_visual_odometry_Subscriber.h"


using camera_capture_msg_t = camera_capture;
using camera_trigger_msg_t = camera_trigger;
using collision_report_msg_t = collision_report;
using debug_array_msg_t = debug_array;
using debug_key_value_msg_t = debug_key_value;
using debug_value_msg_t = debug_value;
using debug_vect_msg_t = debug_vect;
using obstacle_distance_msg_t = obstacle_distance;
using optical_flow_msg_t = optical_flow;
using position_setpoint_msg_t = position_setpoint;
using position_setpoint_triplet_msg_t = position_setpoint_triplet;
using timesync_msg_t = timesync;
using trajectory_waypoint_msg_t = trajectory_waypoint;
using vehicle_trajectory_waypoint_msg_t = vehicle_trajectory_waypoint;
using onboard_computer_status_msg_t = onboard_computer_status;
using vehicle_mocap_odometry_msg_t = vehicle_mocap_odometry;
using vehicle_visual_odometry_msg_t = vehicle_visual_odometry;
using adc_report_msg_t = adc_report;
using airspeed_msg_t = airspeed;
using battery_status_msg_t = battery_status;
using cpuload_msg_t = cpuload;
using distance_sensor_msg_t = distance_sensor;
using estimator_status_msg_t = estimator_status;
using home_position_msg_t = home_position;
using iridiumsbd_status_msg_t = iridiumsbd_status;
using radio_status_msg_t = radio_status;
using satellite_info_msg_t = satellite_info;
using sensor_baro_msg_t = sensor_baro;
using sensor_combined_msg_t = sensor_combined;
using sensor_selection_msg_t = sensor_selection;
using timesync_msg_t = timesync;
using vehicle_attitude_msg_t = vehicle_attitude;
using vehicle_odometry_msg_t = vehicle_odometry;
using vtol_vehicle_status_msg_t = vtol_vehicle_status;
using wind_estimate_msg_t = wind_estimate;
using collision_constraints_msg_t = collision_constraints;

class RtpsTopics {
public:
    bool init(std::condition_variable* t_send_queue_cv, std::mutex* t_send_queue_mutex, std::queue<uint8_t>* t_send_queue);
    void set_timesync(const std::shared_ptr<TimeSync>& timesync) { _timesync = timesync; };
    void publish(uint8_t topic_ID, char data_buffer[], size_t len);
    bool getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr);

private:
    /** Publishers **/
    adc_report_Publisher _adc_report_pub;
    airspeed_Publisher _airspeed_pub;
    battery_status_Publisher _battery_status_pub;
    cpuload_Publisher _cpuload_pub;
    distance_sensor_Publisher _distance_sensor_pub;
    estimator_status_Publisher _estimator_status_pub;
    home_position_Publisher _home_position_pub;
    iridiumsbd_status_Publisher _iridiumsbd_status_pub;
    radio_status_Publisher _radio_status_pub;
    satellite_info_Publisher _satellite_info_pub;
    sensor_baro_Publisher _sensor_baro_pub;
    sensor_combined_Publisher _sensor_combined_pub;
    sensor_selection_Publisher _sensor_selection_pub;
    timesync_Publisher _timesync_pub;
    vehicle_attitude_Publisher _vehicle_attitude_pub;
    vehicle_odometry_Publisher _vehicle_odometry_pub;
    vtol_vehicle_status_Publisher _vtol_vehicle_status_pub;
    wind_estimate_Publisher _wind_estimate_pub;
    collision_constraints_Publisher _collision_constraints_pub;

    /** Subscribers **/
    camera_capture_Subscriber _camera_capture_sub;
    camera_trigger_Subscriber _camera_trigger_sub;
    collision_report_Subscriber _collision_report_sub;
    debug_array_Subscriber _debug_array_sub;
    debug_key_value_Subscriber _debug_key_value_sub;
    debug_value_Subscriber _debug_value_sub;
    debug_vect_Subscriber _debug_vect_sub;
    obstacle_distance_Subscriber _obstacle_distance_sub;
    optical_flow_Subscriber _optical_flow_sub;
    position_setpoint_Subscriber _position_setpoint_sub;
    position_setpoint_triplet_Subscriber _position_setpoint_triplet_sub;
    timesync_Subscriber _timesync_sub;
    trajectory_waypoint_Subscriber _trajectory_waypoint_sub;
    vehicle_trajectory_waypoint_Subscriber _vehicle_trajectory_waypoint_sub;
    onboard_computer_status_Subscriber _onboard_computer_status_sub;
    vehicle_mocap_odometry_Subscriber _vehicle_mocap_odometry_sub;
    vehicle_visual_odometry_Subscriber _vehicle_visual_odometry_sub;

    /** Msg metada Getters **/
    template <class T>
    inline uint64_t getMsgTimestamp(const T* msg) { return msg->timestamp_(); }

    template <class T>
    inline uint8_t getMsgSysID(const T* msg) { return msg->sys_id_(); }

    template <class T>
    inline uint8_t getMsgSeq(const T* msg) { return msg->seq_(); }

    /** Msg metadata Setters **/
    template <class T>
    inline void setMsgTimestamp(T* msg, const uint64_t& timestamp) { msg->timestamp_() = timestamp; }

    template <class T>
    inline void setMsgSysID(T* msg, const uint8_t& sys_id) { msg->sys_id_() = sys_id; }

    template <class T>
    inline void setMsgSeq(T* msg, const uint8_t& seq) { msg->seq_() = seq; }

    /**
     * @brief Timesync object ptr.
     *         This object is used to compuyte and apply the time offsets to the
     *         messages timestamps.
     */
    std::shared_ptr<TimeSync> _timesync;
};
