// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <time.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#ifdef _WIN32
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#endif

#include "an_packet_protocol.h"
#include "subsonus_packets.h"

#define RADIANS_TO_DEGREES (180.0/M_PI)


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;


/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
uint32_t parseIPV4string(char* ipAddress)
{
	unsigned int ipbytes[4];
	sscanf(ipAddress, "%uhh.%uhh.%uhh.%uhh", &ipbytes[3], &ipbytes[2], &ipbytes[1], &ipbytes[0]);
	return ipbytes[0] | ipbytes[1] << 8 | ipbytes[2] << 16 | ipbytes[3] << 24;
}

int an_packet_transmit(an_packet_t *an_packet)
{
	an_packet_encode(an_packet);
	return 0; //SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

/*
 * This is an example of sending a configuration packet to Spatial.
 *
 * 1. First declare the structure for the packet, in this case filter_options_packet_t.
 * 2. Set all the fields of the packet structure
 * 3. Encode the packet structure into an an_packet_t using the appropriate helper function
 * 4. Send the packet
 * 5. Free the packet
 */

void set_network_options()
{
	an_packet_t *an_packet = an_packet_allocate(30, packet_id_network_settings);

	network_settings_packet_t network_settings_packet;

	// initialise the structure by setting all the fields to zero
	memset(&network_settings_packet, 0, sizeof(network_settings_packet_t));

	network_settings_packet.dhcp_mode_flags.b.dhcp_enabled = 1;
	network_settings_packet.dhcp_mode_flags.b.automatic_dns = 1;
	network_settings_packet.dhcp_mode_flags.b.link_mode = link_auto;
	network_settings_packet.permanent = 1;

	network_settings_packet.static_dns_server = (uint32_t) parseIPV4string("0.0.0.0");  // usually the network modem: e.g. 192.168.1.1
	network_settings_packet.static_gateway = (uint32_t) parseIPV4string("0.0.0.0");     // usually the network modem: e.g. 192.168.1.1
	network_settings_packet.static_ip_address = (uint32_t) parseIPV4string("0.0.0.0");  // e.g. 192.168.1.20
	network_settings_packet.static_netmask = (uint32_t) parseIPV4string("0.0.0.0");     // e.g. 255.255.255.0

	encode_network_settings_packet(an_packet, &network_settings_packet);
	an_packet_encode(an_packet);

	an_packet_transmit(an_packet);

	an_packet_free(&an_packet);
}


int tcp_socket;
struct sockaddr_in serveraddr;
struct hostent *server;


class MinimalPublisher : public rclcpp::Node
{
public:
  
  clock_t begin = clock();
  int id=0;
  geometry_msgs::msg::PoseStamped message = geometry_msgs::msg::PoseStamped();
  geometry_msgs::msg::PoseStamped message_sub = geometry_msgs::msg::PoseStamped();
  geometry_msgs::msg::TransformStamped ts;
  geometry_msgs::msg::TransformStamped ts_sub;
  geometry_msgs::msg::TransformStamped ts_elec;
  visualization_msgs::msg::Marker marker;
  tf2::Quaternion q_elec;
  float alpha;
  float beta;
  float gamma;
  
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    
    ts.header.frame_id = "map";
    ts.child_frame_id = "surface";

    ts_sub.header.frame_id = "surface";
    ts_sub.child_frame_id = "ROV";

    ts_elec.header.frame_id = "ROV";
    ts_elec.child_frame_id = "elec";

    message.header.frame_id="map";
    message_sub.header.frame_id="surface";

    marker.header.frame_id="ROV";
    marker.mesh_resource = "file:///home/neo/workspaceRos2/src/subsonus_pkg/meshes/submarine.obj";
    marker.mesh_use_embedded_materials = true;
    marker.ns = "";

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_broadcaster_sub = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_broadcaster_elec = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    //remplir ici la transformée entre le repère USBL fond et mesure elec
    q_elec.setRPY(0,0, -M_PI/4);
    q_elec.normalize();
    ts_elec.transform.rotation.z =q_elec.z();
    ts_elec.transform.rotation.w =q_elec.w();
    ts_elec.transform.translation.x =1.;

    unsigned int bytes_received = 0;
    char *hostname;
    hostname = "192.168.2.100";
    int port = 19000;
    // int port = 16718;


    tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(tcp_socket < 0)
    {
      printf("Could not open TCP socket\n");
      exit(EXIT_FAILURE);
    }

    // Find the address of the host
    server = gethostbyname("192.168.2.100");
    if(server == NULL)
    {
      printf("Could not find host %s\n", hostname);
      exit(EXIT_FAILURE);
    }
    else{
      printf("ok host\n");
    }

    // Set the server's address
    memset((char *) &serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    memcpy((char *) &serveraddr.sin_addr.s_addr, (char *) server->h_addr_list[0], server->h_length);
    serveraddr.sin_port = htons(port);
    if(connect(tcp_socket, (struct sockaddr *) &serveraddr, sizeof(serveraddr)) < 0)
    {
      printf("Could not connect to server\n");
    }else{
      printf("ok socket\n");
    }

    {
      int flush_length = 0;
      while(1)
      {
        struct timeval t;
        fd_set readfds;
        t.tv_sec = 0;
        t.tv_usec = 50000;
        unsigned char buf[1024];
        FD_ZERO(&readfds);
        FD_SET(tcp_socket, &readfds);
        select(tcp_socket + 1, &readfds, NULL, NULL, &t);
        if(FD_ISSET(tcp_socket, &readfds))
        {
          flush_length = recv(tcp_socket, buf, sizeof(buf), 0);
          if(flush_length < 100)
          {
            break;
          }
        }
        else
        {
          break;
        }
      }
    }

    printf("Encode Network Settings Packet:\n");
    set_network_options();

    publisher_rotation = this->create_publisher<std_msgs::msg::Float64MultiArray>("rotation_matrix", 1000);
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1000);
    publisher_sub = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_sub", 1000);
    publisher_marker = this->create_publisher<visualization_msgs::msg::Marker>("marker", 1000);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {  
    ts.header.stamp = this->get_clock()->now();
    ts_sub.header.stamp = this->get_clock()->now();
    ts_elec.header.stamp = this->get_clock()->now();
    message.header.stamp=this->get_clock()->now();
    message_sub.header.stamp=this->get_clock()->now();

    marker.header.stamp =this->get_clock()->now();
    

    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.2;
    marker.color.g = 0.2;
    marker.color.b = 0.9;
    marker.color.a = 0.8;

    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = -0.2;
    tf2::Quaternion q_mark;
    q_mark.setRPY(-M_PI/2 ,M_PI ,-M_PI/2);
    q_mark.normalize();
    marker.pose.orientation.x = q_mark.x();
    marker.pose.orientation.y = q_mark.y();
    marker.pose.orientation.z = q_mark.z();
    marker.pose.orientation.w = q_mark.w();
    publisher_marker->publish(marker);
    id++;



    unsigned int bytes_received = 0;
    struct timeval t;
    fd_set readfds;
    t.tv_sec = 0;
    t.tv_usec = 10000;

    an_decoder_t an_decoder;
    an_packet_t *an_packet;

    an_decoder_initialise(&an_decoder);

    subsonus_system_state_packet_t system_state_packet;
    subsonus_track_packet_t subsonus_track_packet;
    subsonus_remote_system_state_packet_t subsonus_remote_system_state_packet;
    FD_ZERO(&readfds);
		FD_SET(tcp_socket, &readfds);
		select(tcp_socket + 1, &readfds, NULL, NULL, &t);
		if(FD_ISSET(tcp_socket, &readfds))
		{

#ifdef _WIN32
			bytes_received = recv(tcp_socket,an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder),0);
#else
			bytes_received = recv(tcp_socket, an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder), MSG_DONTWAIT);
#endif

			if(bytes_received > 0)
			{
				/* increment the decode buffer length by the number of bytes received */
				an_decoder_increment(&an_decoder, bytes_received);

				/* decode all the packets in the buffer */
				while((an_packet = an_packet_decode_dynamic(&an_decoder)) != NULL)
				{
          // printf("test %u\n",an_packet->id);
					if(an_packet->id == packet_id_subsonus_system_state) /* system state packet */
					{
						// printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
						/* copy all the binary data into the typedef struct for the packet */
						/* this allows easy access to all the different values             */
						if(decode_subsonus_system_state_packet(&system_state_packet, an_packet) == 0)
						{
              // cout<<system_state_packet.longitude<<endl;
              message.pose.position.x=0.;
              message.pose.position.y=0.;
              message.pose.position.z= 0.;
              tf2::Quaternion q;
              alpha=system_state_packet.orientation[0];
              beta=system_state_packet.orientation[1];
              gamma=system_state_packet.orientation[2];
              q.setRPY(alpha,beta, gamma );
							q.normalize();
              message.pose.orientation.x=q.x();
              message.pose.orientation.y=q.y();
              message.pose.orientation.z=q.z();
              message.pose.orientation.w=q.w();
              ts.transform.translation.x = 0.0;
              ts.transform.translation.y = 0.0;
              ts.transform.translation.z = 0.0;
              ts.transform.rotation.x = q.x();
              ts.transform.rotation.y = q.y();
              ts.transform.rotation.z = q.z();
              ts.transform.rotation.w = q.w();
              tf_broadcaster_->sendTransform(ts);
              
              // printf("Subsonus System State Packet:\n");
							// printf("\tLatitude = %f, Longitude = %f, Height = %f\n", system_state_packet.latitude * RADIANS_TO_DEGREES, system_state_packet.longitude * RADIANS_TO_DEGREES, system_state_packet.height);
							// printf("\tRoll = %f, Pitch = %f, Heading = %f\n", system_state_packet.orientation[0] * RADIANS_TO_DEGREES, system_state_packet.orientation[1] * RADIANS_TO_DEGREES, system_state_packet.orientation[2] * RADIANS_TO_DEGREES);
						}
					}
					else if(an_packet->id == packet_id_subsonus_track) /* subsonus track packet */
					{
						// printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
						/* copy all the binary data into the typedef struct for the packet */
						/* this allows easy access to all the different values             */
						if(decode_subsonus_track_packet(&subsonus_track_packet, an_packet) == 0)
						{
              
              
              message_sub.pose.position.x=subsonus_track_packet.corrected_position[0];
              message_sub.pose.position.y=subsonus_track_packet.corrected_position[1];
              message_sub.pose.position.z=subsonus_track_packet.corrected_position[2];
              publisher_sub->publish(message_sub);
              
              
              ts_sub.transform.translation.x =subsonus_track_packet.corrected_position[0];
              ts_sub.transform.translation.y = subsonus_track_packet.corrected_position[1];
              ts_sub.transform.translation.z = subsonus_track_packet.corrected_position[2];
              
              tf_broadcaster_sub->sendTransform(ts_sub);
							printf("Remote Track Packet:\n");
							// printf("\tLatitude = %f, Longitude = %f, Height = %f\n", subsonus_track_packet.latitude * RADIANS_TO_DEGREES, subsonus_track_packet.longitude * RADIANS_TO_DEGREES, subsonus_track_packet.height);
							// printf("\tRange = %f, Azimuth = %f, Elevation = %f\n", subsonus_track_packet.range, subsonus_track_packet.azimuth * RADIANS_TO_DEGREES, subsonus_track_packet.elevation * RADIANS_TO_DEGREES);
						}
					}
					else if(an_packet->id == packet_id_subsonus_remote_system_state)
					{
            // printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
						// printf("Remote System State Packet:\n");
            if(decode_subsonus_remote_system_state_packet(&subsonus_remote_system_state_packet, an_packet) == 0)
						{
            tf2::Quaternion q_sub;
            if (subsonus_remote_system_state_packet.orientation[0]!=0 || subsonus_remote_system_state_packet.orientation[1]!=0 || subsonus_remote_system_state_packet.orientation[2]!=0){
            float alphap=subsonus_remote_system_state_packet.orientation[0];
            float betap=subsonus_remote_system_state_packet.orientation[1];
            float gammap=subsonus_remote_system_state_packet.orientation[2];

            tf2::Quaternion q_map_to_sub;
            tf2::Quaternion q_map_to_surf;

            tf2::fromMsg(message.pose.orientation, q_map_to_surf);
            q_map_to_sub.setRPY(alphap,betap,gammap);

            tf2::Quaternion q_surf_to_map;
            q_surf_to_map=q_map_to_surf;
            q_surf_to_map[3]=-q_surf_to_map[3];

            q_sub=q_map_to_sub*q_surf_to_map;


            q_sub.normalize();
            message_sub.pose.orientation.x=q_sub.x();
            message_sub.pose.orientation.y=q_sub.y();
            message_sub.pose.orientation.z=q_sub.z();
            message_sub.pose.orientation.w=q_sub.w();
            ts_sub.transform.rotation.x = q_sub.x();
            ts_sub.transform.rotation.y = q_sub.y();
            ts_sub.transform.rotation.z = q_sub.z();
            ts_sub.transform.rotation.w = q_sub.w();


            }
            else{
              printf("Message de State vide: %u ",subsonus_remote_system_state_packet.data_valid.r);
            }
            tf_broadcaster_sub->sendTransform(ts_sub);
            publisher_sub->publish(message_sub);
            

            // printf("Remote System State Packet:\n");
            }
					}
					else
					{
						// printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
					}

					/* Ensure that you free the an_packet when your done with it or you will leak memory */
					an_packet_free(&an_packet);
				}
			}
		}

    // ts_sub.transform.translation.x =5.; //a commenter
    // message_sub.pose.position.x=5.; //a commenter
    // ts_sub.transform.translation.z =5.; //a commenter
    // message_sub.pose.position.z=5.; //a commenter 
    // tf2::Quaternion q_sub;
    // q_sub.setRPY(M_PI,0,0);
    // q_sub.normalize();
    // ts_sub.transform.rotation.x =q_sub.x(); //a commenter
    // message_sub.pose.orientation.x=q_sub.x(); //a commenter
    // ts_sub.transform.rotation.y =q_sub.y(); //a commenter
    // message_sub.pose.orientation.y=q_sub.y(); //a commenter
    // ts_sub.transform.rotation.z =q_sub.z(); //a commenter
    // message_sub.pose.orientation.z=q_sub.z(); //a commenter
    // ts_sub.transform.rotation.w =q_sub.w(); //a commenter
    // message_sub.pose.orientation.w=q_sub.w(); //a commenter

    tf_broadcaster_sub->sendTransform(ts_sub);
    tf_broadcaster_elec->sendTransform(ts_elec);
    publisher_->publish(message);
    publisher_sub->publish(message_sub);

    tf2::Quaternion quat_tf;
    tf2::fromMsg(message.pose.orientation, quat_tf);
    tf2::Matrix3x3 m(quat_tf);
    m.getRotation(quat_tf);

    tf2::Quaternion quat_tf_sub;
    tf2::fromMsg(message_sub.pose.orientation, quat_tf_sub);
    tf2::Matrix3x3 m_sub(quat_tf_sub);
    m_sub.getRotation(quat_tf_sub);

    // tf2::Quaternion quat_tf_elec;
    // tf2::fromMsg(message_sub.pose.orientation, quat_tf_elec);
    tf2::Matrix3x3 m_elec(q_elec);
    m_elec.getRotation(q_elec);


    m*=m_sub;
    m*=m_elec;
    //ici m est la matrice de rotation du repere map (bateau) vers le repere mesure elec
    // for (int i=0;i<3;i++){
    //   for (int j=0;j<3;j++){
    //     cout<<m[i][j]<<" ";
    //   }
    //   cout<<endl;
    // }
    // cout<<endl;
    // cout<<endl;
    auto message_rotation = std_msgs::msg::Float64MultiArray();
    std::vector<float> vec1 = {m[0][0],m[0][1],m[0][2],m[1][0],m[1][1],m[1][2],m[2][0],m[2][1],m[2][2]};
    message_rotation.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    message_rotation.layout.dim[0].size = 3;
    message_rotation.layout.dim[0].label = "ligne";
    message_rotation.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    message_rotation.layout.dim[1].size = 3;
    message_rotation.layout.dim[1].label = "colonne";
    message_rotation.data.clear();
    message_rotation.data.insert(message_rotation.data.end(), vec1.begin(), vec1.end());
    publisher_rotation->publish(message_rotation);

    
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_sub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_marker;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_rotation;
  size_t count_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_sub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_elec;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
