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

#include "RtpsTopics.h"

bool RtpsTopics::init(std::condition_variable* t_send_queue_cv, std::mutex* t_send_queue_mutex, std::queue<uint8_t>* t_send_queue)
{
    // Initialise subscribers
    std::cout << "\033[0;36m---   Subscribers   ---\033[0m" << std::endl;
    if (_camera_capture_sub.init(7, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- camera_capture subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting camera_capture subscriber" << std::endl;
        return false;
    }
    if (_camera_trigger_sub.init(8, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- camera_trigger subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting camera_trigger subscriber" << std::endl;
        return false;
    }
    if (_collision_report_sub.init(9, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- collision_report subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting collision_report subscriber" << std::endl;
        return false;
    }
    if (_debug_array_sub.init(12, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- debug_array subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting debug_array subscriber" << std::endl;
        return false;
    }
    if (_debug_key_value_sub.init(13, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- debug_key_value subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting debug_key_value subscriber" << std::endl;
        return false;
    }
    if (_debug_value_sub.init(14, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- debug_value subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting debug_value subscriber" << std::endl;
        return false;
    }
    if (_debug_vect_sub.init(15, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- debug_vect subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting debug_vect subscriber" << std::endl;
        return false;
    }
    if (_obstacle_distance_sub.init(43, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- obstacle_distance subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting obstacle_distance subscriber" << std::endl;
        return false;
    }
    if (_optical_flow_sub.init(45, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- optical_flow subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting optical_flow subscriber" << std::endl;
        return false;
    }
    if (_position_setpoint_sub.init(50, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- position_setpoint subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting position_setpoint subscriber" << std::endl;
        return false;
    }
    if (_position_setpoint_triplet_sub.init(51, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- position_setpoint_triplet subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting position_setpoint_triplet subscriber" << std::endl;
        return false;
    }
    if (_timesync_sub.init(78, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- timesync subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting timesync subscriber" << std::endl;
        return false;
    }
    if (_trajectory_waypoint_sub.init(79, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- trajectory_waypoint subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting trajectory_waypoint subscriber" << std::endl;
        return false;
    }
    if (_vehicle_trajectory_waypoint_sub.init(104, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- vehicle_trajectory_waypoint subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting vehicle_trajectory_waypoint subscriber" << std::endl;
        return false;
    }
    if (_onboard_computer_status_sub.init(116, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- onboard_computer_status subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting onboard_computer_status subscriber" << std::endl;
        return false;
    }
    if (_vehicle_mocap_odometry_sub.init(161, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- vehicle_mocap_odometry subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting vehicle_mocap_odometry subscriber" << std::endl;
        return false;
    }
    if (_vehicle_visual_odometry_sub.init(162, t_send_queue_cv, t_send_queue_mutex, t_send_queue)) {
        std::cout << "- vehicle_visual_odometry subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting vehicle_visual_odometry subscriber" << std::endl;
        return false;
    }
    std::cout << "\033[0;36m-----------------------\033[0m" << std::endl << std::endl;
    // Initialise publishers
    std::cout << "\033[0;36m----   Publishers  ----\033[0m" << std::endl;
    if (_adc_report_pub.init()) {
        std::cout << "- adc_report publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting adc_report publisher" << std::endl;
        return false;
    }
    if (_airspeed_pub.init()) {
        std::cout << "- airspeed publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting airspeed publisher" << std::endl;
        return false;
    }
    if (_battery_status_pub.init()) {
        std::cout << "- battery_status publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting battery_status publisher" << std::endl;
        return false;
    }
    if (_cpuload_pub.init()) {
        std::cout << "- cpuload publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting cpuload publisher" << std::endl;
        return false;
    }
    if (_distance_sensor_pub.init()) {
        std::cout << "- distance_sensor publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting distance_sensor publisher" << std::endl;
        return false;
    }
    if (_estimator_status_pub.init()) {
        std::cout << "- estimator_status publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting estimator_status publisher" << std::endl;
        return false;
    }
    if (_home_position_pub.init()) {
        std::cout << "- home_position publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting home_position publisher" << std::endl;
        return false;
    }
    if (_iridiumsbd_status_pub.init()) {
        std::cout << "- iridiumsbd_status publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting iridiumsbd_status publisher" << std::endl;
        return false;
    }
    if (_radio_status_pub.init()) {
        std::cout << "- radio_status publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting radio_status publisher" << std::endl;
        return false;
    }
    if (_satellite_info_pub.init()) {
        std::cout << "- satellite_info publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting satellite_info publisher" << std::endl;
        return false;
    }
    if (_sensor_baro_pub.init()) {
        std::cout << "- sensor_baro publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting sensor_baro publisher" << std::endl;
        return false;
    }
    if (_sensor_combined_pub.init()) {
        std::cout << "- sensor_combined publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting sensor_combined publisher" << std::endl;
        return false;
    }
    if (_sensor_selection_pub.init()) {
        std::cout << "- sensor_selection publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting sensor_selection publisher" << std::endl;
        return false;
    }
    if (_timesync_pub.init()) {
        std::cout << "- timesync publisher started" << std::endl;
        _timesync->start(&_timesync_pub);
    } else {
        std::cerr << "ERROR starting timesync publisher" << std::endl;
        return false;
    }
    if (_vehicle_attitude_pub.init()) {
        std::cout << "- vehicle_attitude publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting vehicle_attitude publisher" << std::endl;
        return false;
    }
    if (_vehicle_odometry_pub.init()) {
        std::cout << "- vehicle_odometry publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting vehicle_odometry publisher" << std::endl;
        return false;
    }
    if (_vtol_vehicle_status_pub.init()) {
        std::cout << "- vtol_vehicle_status publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting vtol_vehicle_status publisher" << std::endl;
        return false;
    }
    if (_wind_estimate_pub.init()) {
        std::cout << "- wind_estimate publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting wind_estimate publisher" << std::endl;
        return false;
    }
    if (_collision_constraints_pub.init()) {
        std::cout << "- collision_constraints publisher started" << std::endl;
    } else {
        std::cerr << "ERROR starting collision_constraints publisher" << std::endl;
        return false;
    }
    std::cout << "\033[0;36m-----------------------\033[0m" << std::endl;
    return true;
}

void RtpsTopics::publish(uint8_t topic_ID, char data_buffer[], size_t len)
{
    switch (topic_ID)
    {
        case 4: // adc_report
        {
            adc_report_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _adc_report_pub.publish(&st);
        }
        break;
        case 5: // airspeed
        {
            airspeed_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _airspeed_pub.publish(&st);
        }
        break;
        case 6: // battery_status
        {
            battery_status_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _battery_status_pub.publish(&st);
        }
        break;
        case 11: // cpuload
        {
            cpuload_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _cpuload_pub.publish(&st);
        }
        break;
        case 17: // distance_sensor
        {
            distance_sensor_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _distance_sensor_pub.publish(&st);
        }
        break;
        case 24: // estimator_status
        {
            estimator_status_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _estimator_status_pub.publish(&st);
        }
        break;
        case 29: // home_position
        {
            home_position_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _home_position_pub.publish(&st);
        }
        break;
        case 31: // iridiumsbd_status
        {
            iridiumsbd_status_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _iridiumsbd_status_pub.publish(&st);
        }
        break;
        case 56: // radio_status
        {
            radio_status_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _radio_status_pub.publish(&st);
        }
        break;
        case 61: // satellite_info
        {
            satellite_info_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _satellite_info_pub.publish(&st);
        }
        break;
        case 63: // sensor_baro
        {
            sensor_baro_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _sensor_baro_pub.publish(&st);
        }
        break;
        case 65: // sensor_combined
        {
            sensor_combined_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _sensor_combined_pub.publish(&st);
        }
        break;
        case 70: // sensor_selection
        {
            sensor_selection_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _sensor_selection_pub.publish(&st);
        }
        break;
        case 78: // timesync
        {
            timesync_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            _timesync->processTimesyncMsg(&st);

            if (getMsgSysID(&st) == 1) {
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _timesync_pub.publish(&st);
            }
        }
        break;
        case 87: // vehicle_attitude
        {
            vehicle_attitude_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _vehicle_attitude_pub.publish(&st);
        }
        break;
        case 99: // vehicle_odometry
        {
            vehicle_odometry_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _vehicle_odometry_pub.publish(&st);
        }
        break;
        case 105: // vtol_vehicle_status
        {
            vtol_vehicle_status_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _vtol_vehicle_status_pub.publish(&st);
        }
        break;
        case 106: // wind_estimate
        {
            wind_estimate_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _wind_estimate_pub.publish(&st);
        }
        break;
        case 107: // collision_constraints
        {
            collision_constraints_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _collision_constraints_pub.publish(&st);
        }
        break;
        default:
            printf("\033[1;33m[   micrortps_agent   ]\tUnexpected topic ID to publish\033[0m\n");
        break;
    }
}

bool RtpsTopics::getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr)
{
    bool ret = false;
    switch (topic_ID)
    {
        case 7: // camera_capture
            if (_camera_capture_sub.hasMsg())
            {
                camera_capture_msg_t msg = _camera_capture_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _camera_capture_sub.unlockMsg();
            }
        break;
        case 8: // camera_trigger
            if (_camera_trigger_sub.hasMsg())
            {
                camera_trigger_msg_t msg = _camera_trigger_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _camera_trigger_sub.unlockMsg();
            }
        break;
        case 9: // collision_report
            if (_collision_report_sub.hasMsg())
            {
                collision_report_msg_t msg = _collision_report_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _collision_report_sub.unlockMsg();
            }
        break;
        case 12: // debug_array
            if (_debug_array_sub.hasMsg())
            {
                debug_array_msg_t msg = _debug_array_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _debug_array_sub.unlockMsg();
            }
        break;
        case 13: // debug_key_value
            if (_debug_key_value_sub.hasMsg())
            {
                debug_key_value_msg_t msg = _debug_key_value_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _debug_key_value_sub.unlockMsg();
            }
        break;
        case 14: // debug_value
            if (_debug_value_sub.hasMsg())
            {
                debug_value_msg_t msg = _debug_value_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _debug_value_sub.unlockMsg();
            }
        break;
        case 15: // debug_vect
            if (_debug_vect_sub.hasMsg())
            {
                debug_vect_msg_t msg = _debug_vect_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _debug_vect_sub.unlockMsg();
            }
        break;
        case 43: // obstacle_distance
            if (_obstacle_distance_sub.hasMsg())
            {
                obstacle_distance_msg_t msg = _obstacle_distance_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _obstacle_distance_sub.unlockMsg();
            }
        break;
        case 45: // optical_flow
            if (_optical_flow_sub.hasMsg())
            {
                optical_flow_msg_t msg = _optical_flow_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _optical_flow_sub.unlockMsg();
            }
        break;
        case 50: // position_setpoint
            if (_position_setpoint_sub.hasMsg())
            {
                position_setpoint_msg_t msg = _position_setpoint_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _position_setpoint_sub.unlockMsg();
            }
        break;
        case 51: // position_setpoint_triplet
            if (_position_setpoint_triplet_sub.hasMsg())
            {
                position_setpoint_triplet_msg_t msg = _position_setpoint_triplet_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _position_setpoint_triplet_sub.unlockMsg();
            }
        break;
        case 78: // timesync
            if (_timesync_sub.hasMsg())
            {
                timesync_msg_t msg = _timesync_sub.getMsg();
                if (getMsgSysID(&msg) == 0) {
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                }
                _timesync_sub.unlockMsg();
            }
        break;
        case 79: // trajectory_waypoint
            if (_trajectory_waypoint_sub.hasMsg())
            {
                trajectory_waypoint_msg_t msg = _trajectory_waypoint_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _trajectory_waypoint_sub.unlockMsg();
            }
        break;
        case 104: // vehicle_trajectory_waypoint
            if (_vehicle_trajectory_waypoint_sub.hasMsg())
            {
                vehicle_trajectory_waypoint_msg_t msg = _vehicle_trajectory_waypoint_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _vehicle_trajectory_waypoint_sub.unlockMsg();
            }
        break;
        case 116: // onboard_computer_status
            if (_onboard_computer_status_sub.hasMsg())
            {
                onboard_computer_status_msg_t msg = _onboard_computer_status_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _onboard_computer_status_sub.unlockMsg();
            }
        break;
        case 161: // vehicle_mocap_odometry
            if (_vehicle_mocap_odometry_sub.hasMsg())
            {
                vehicle_mocap_odometry_msg_t msg = _vehicle_mocap_odometry_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _vehicle_mocap_odometry_sub.unlockMsg();
            }
        break;
        case 162: // vehicle_visual_odometry
            if (_vehicle_visual_odometry_sub.hasMsg())
            {
                vehicle_visual_odometry_msg_t msg = _vehicle_visual_odometry_sub.getMsg();
                // apply timestamp offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                msg.serialize(scdr);
                ret = true;
                _vehicle_visual_odometry_sub.unlockMsg();
            }
        break;
        default:
            printf("\033[1;33m[   micrortps_agent   ]\tUnexpected topic ID '%hhu' to getMsg\033[0m\n", topic_ID);
        break;
    }

    return ret;
}
