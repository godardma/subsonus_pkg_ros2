/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*          C Language Subsonus SDK, Version 2.4       		*/
/*              Copyright 2020, Advanced Navigation             */
/*                                                              */
/****************************************************************/
/*
 * Copyright (C) 2020 Advanced Navigation
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdint.h>
#include <string.h>
#include "an_packet_protocol.h"
#include "subsonus_packets.h"
#include "anpp_helper_macro.h"

/*
 * This file contains functions to decode and encode packets
 *
 * Decode functions take an an_packet_t and turn it into a type specific
 * to that packet so that the fields can be conveniently accessed. Decode
 * functions return 0 for success and 1 for failure. Decode functions are
 * used when receiving packets.
 *
 * Example decode
 *
 * an_packet_t an_packet
 * acknowledge_packet_t acknowledge_packet
 * ...
 * decode_acknowledge_packet(&acknowledge_packet, &an_packet);
 * printf("acknowledge id %d with result %d\n", acknowledge_packet.packet_id, acknowledge_packet.acknowledge_result);
 *
 * Encode functions take a type specific structure and turn it into an
 * an_packet_t. Encode functions are used when sending packets.
 *
 * Example encode
 *
 * an_packet_t an_packet;
 * boot_mode_packet_t boot_mode_packet;
 * ...
 * boot_mode_packet.boot_mode = boot_mode_bootloader;
 * encode_boot_mode_packet(&an_packet, &boot_mode_packet);
 * serial_port_transmit(an_packet_pointer(&an_packet), an_packet_size(&an_packet));
 *
 */

int decode_acknowledge_packet(acknowledge_packet_t *acknowledge_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_acknowledge && an_packet->length == 4)
	{
		acknowledge_packet->packet_id = an_packet->data[0];
		memcpy(&acknowledge_packet->packet_crc, &an_packet->data[1], sizeof(uint16_t));
		acknowledge_packet->acknowledge_result = an_packet->data[3];
		return 0;
	}
	else return 1;
}

void encode_request_packet(an_packet_t *an_packet, uint8_t requested_packet_id)
{
	an_packet->id = packet_id_request;
	an_packet->length = 1;
	an_packet->data[0] = requested_packet_id;
}

int decode_device_information_packet(device_information_packet_t *device_information_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_device_information && an_packet->length == 24)
	{
		memcpy(&device_information_packet->software_version, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&device_information_packet->device_id, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&device_information_packet->hardware_revision, &an_packet->data[8], sizeof(uint32_t));
		memcpy(&device_information_packet->serial_number[0], &an_packet->data[12], 3 * sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

void encode_restore_factory_settings_packet(an_packet_t *an_packet, int surface_mode)
{
	uint32_t verification = 0x85429E1C; // Surface Mode
	if(surface_mode == 0){
		verification = 0xDEFA8581; // Subsea Mode
	}
	an_packet->id = packet_id_restore_factory_settings;
	an_packet->length = 4;
	memcpy(&an_packet->data[0], &verification, sizeof(uint32_t));
}

void encode_reset_packet(an_packet_t *an_packet)
{
	uint32_t verification = 0x21057A7E;
	an_packet->id = packet_id_reset;
	an_packet->length = 4;
	memcpy(&an_packet->data[0], &verification, sizeof(uint32_t));
}


void encode_network_settings_packet(an_packet_t *an_packet,network_settings_packet_t *network_settings_packet){
	an_packet->id = packet_id_network_settings;
	an_packet->length = 18;
	memcpy(&an_packet->data[0], &network_settings_packet->permanent, sizeof(uint8_t));
	memcpy(&an_packet->data[1], &network_settings_packet->dhcp_mode_flags, sizeof(uint8_t));
	memcpy(&an_packet->data[2], &network_settings_packet->static_ip_address, sizeof(uint32_t));
	memcpy(&an_packet->data[6], &network_settings_packet->static_netmask, sizeof(uint32_t));
	memcpy(&an_packet->data[10], &network_settings_packet->static_gateway, sizeof(uint32_t));
	memcpy(&an_packet->data[14], &network_settings_packet->static_dns_server, sizeof(uint32_t));
}

void encode_extended_network_settings_packet(an_packet_t *an_packet,network_settings_packet_t *network_settings_packet){
	encode_network_settings_packet(an_packet,network_settings_packet);
	an_packet->length = 30;
	memcpy(&an_packet->data[18], &network_settings_packet->serial_number[0], sizeof(uint32_t)*3);
}

int decode_network_settings_packet(network_settings_packet_t *network_settings_packet, an_packet_t *an_packet){
	if(an_packet->id == packet_id_network_settings && (an_packet->length == 18 || an_packet->length == 30))
	{
		memcpy(&network_settings_packet->permanent, &an_packet->data[0], sizeof(uint8_t));
		memcpy(&network_settings_packet->dhcp_mode_flags, &an_packet->data[1], sizeof(uint8_t));
		memcpy(&network_settings_packet->static_ip_address, &an_packet->data[2], sizeof(uint32_t));
		memcpy(&network_settings_packet->static_netmask, &an_packet->data[6], sizeof(uint32_t));
		memcpy(&network_settings_packet->static_gateway, &an_packet->data[10], sizeof(uint32_t));
		memcpy(&network_settings_packet->static_dns_server, &an_packet->data[14], sizeof(uint32_t));
		if(an_packet->length == 30){
			memcpy(&network_settings_packet->serial_number, &an_packet->data[18], sizeof(uint32_t)*3);
		}
		else{
			network_settings_packet->serial_number[0] = 0;
			network_settings_packet->serial_number[1] = 0;
			network_settings_packet->serial_number[2] = 0;
		}
		return 0;
	}
	else return 1;
}

int decode_subsonus_hostname_packet(subsonus_hostname_packet_t * subsonus_hostname_packet, an_packet_t * an_packet){
    if(an_packet->id == packet_id_subsonus_hostname && an_packet->length == 16){
        memcpy(&subsonus_hostname_packet->hostname, &an_packet->data[0],  16);
        return 0;
    }
    else{ return 1; }
}

void encode_subsonus_hostname_packet(an_packet_t * an_packet, subsonus_hostname_packet_t * subsonus_hostname_packet){
    an_packet->id = packet_id_subsonus_hostname;
    an_packet->length = 16;
    memcpy(&an_packet->data[0], &subsonus_hostname_packet->hostname, 16);
}

int decode_subsonus_system_state_packet(subsonus_system_state_packet_t * system_state_packet, an_packet_t * an_packet){
    if(an_packet->id == packet_id_subsonus_system_state && an_packet->length == 116){
        int count = 0;
        COPY_OUT_OF_AN_PACKET(system_state_packet->system_status.r)
        COPY_OUT_OF_AN_PACKET(system_state_packet->filter_status.r)
        COPY_OUT_OF_AN_PACKET(system_state_packet->unix_time_seconds)
        COPY_OUT_OF_AN_PACKET(system_state_packet->microseconds)
        COPY_OUT_OF_AN_PACKET(system_state_packet->latitude)
        COPY_OUT_OF_AN_PACKET(system_state_packet->longitude)
        COPY_OUT_OF_AN_PACKET(system_state_packet->height)
        COPY_OUT_OF_AN_PACKET_A(system_state_packet->velocity[0],3)
        COPY_OUT_OF_AN_PACKET_A(system_state_packet->body_acceleration[0],3)
        COPY_OUT_OF_AN_PACKET(system_state_packet->g_force)
        COPY_OUT_OF_AN_PACKET_A(system_state_packet->orientation[0], 3)
        COPY_OUT_OF_AN_PACKET_A(system_state_packet->angular_velocity[0], 3)
        COPY_OUT_OF_AN_PACKET_A(system_state_packet->position_standard_deviation[0], 3)
        COPY_OUT_OF_AN_PACKET_A(system_state_packet->orientation_standard_deviation[0], 3)
        return 0;
    }
    else{ return 1; }
}

int decode_unix_time_packet(unix_time_packet_t *unix_time_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_unix_time && an_packet->length == 8)
	{
		memcpy(&unix_time_packet->unix_time_seconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&unix_time_packet->microseconds, &an_packet->data[4], sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

int decode_formatted_time_packet(formatted_time_packet_t *formatted_time_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_formatted_time && an_packet->length == 14)
	{
		memcpy(&formatted_time_packet->microseconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&formatted_time_packet->year, &an_packet->data[4], sizeof(uint16_t));
		memcpy(&formatted_time_packet->year_day, &an_packet->data[6], sizeof(uint16_t));
		formatted_time_packet->month = an_packet->data[8];
		formatted_time_packet->month_day = an_packet->data[9];
		formatted_time_packet->week_day = an_packet->data[10];
		formatted_time_packet->hour = an_packet->data[11];
		formatted_time_packet->minute = an_packet->data[12];
		formatted_time_packet->second = an_packet->data[13];
		return 0;
	}
	else return 1;
}

int decode_subsonus_status_packet(subsonus_status_packet_t * status_packet, an_packet_t * an_packet){
    if(an_packet->id == packet_id_subsonus_status && an_packet->length == 8){
        int count = 0;
        COPY_OUT_OF_AN_PACKET(status_packet->system_status.r)
        COPY_OUT_OF_AN_PACKET(status_packet->filter_status.r)
        return 0;
    }
    else{ return 1; }
}

int decode_subsonus_track_packet(subsonus_track_packet_t * remote_unit_tracking_packet, an_packet_t * an_packet){
    if(an_packet->id == packet_id_subsonus_track && an_packet->length == 211){
        int count = 0;
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->device_address)  //utile en multi-tracking
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->tracking_status.r)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->observer_system_status.r)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->observer_filter_status.r)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->data_valid.r)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->observer_unix_time_seconds)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->observer_microseconds)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->observer_latitude)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->observer_longitude)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->observer_height)
        COPY_OUT_OF_AN_PACKET_A(remote_unit_tracking_packet->observer_velocity[0],3)
        COPY_OUT_OF_AN_PACKET_A(remote_unit_tracking_packet->observer_orientation[0],3) //RPY local (surface) (rd)
        COPY_OUT_OF_AN_PACKET_A(remote_unit_tracking_packet->observer_position_standard_deviation[0],3)
        COPY_OUT_OF_AN_PACKET_A(remote_unit_tracking_packet->observer_orientation_standard_deviation[0],3)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->observer_depth)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->age_microseconds)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->range)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->azimuth)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->elevation)
        COPY_OUT_OF_AN_PACKET_A(remote_unit_tracking_packet->raw_position[0],3)
        COPY_OUT_OF_AN_PACKET_A(remote_unit_tracking_packet->corrected_position[0],3)   //à utiliser
        COPY_OUT_OF_AN_PACKET_A(remote_unit_tracking_packet->ned_position[0],3)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->latitude)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->longitude)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->height)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->range_standard_deviation)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->azimuth_standard_deviation)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->elevation_standard_deviation)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->latitude_standard_deviation)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->longitude_standard_deviation)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->height_standard_deviation)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->depth)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->signal_level)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->signal_to_noise_ratio)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->signal_correlation_ratio)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->signal_correlation_interference)
        COPY_OUT_OF_AN_PACKET(remote_unit_tracking_packet->reserved)
        return 0;
    }
    else{ return 1; }
}

int decode_subsonus_remote_system_state_packet(subsonus_remote_system_state_packet_t * remote_system_state_packet, an_packet_t * an_packet){
    if(an_packet->id == packet_id_subsonus_remote_system_state && an_packet->length == 130){
        int count = 0;
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->device_address)
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->system_status_valid.r)
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->filter_status_valid.r)
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->system_status.r)
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->filter_status.r)
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->data_valid.r)
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->unix_time_seconds)
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->microseconds)
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->latitude)
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->longitude)
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->height)
        COPY_OUT_OF_AN_PACKET_A(remote_system_state_packet->velocity[0],3)
        COPY_OUT_OF_AN_PACKET_A(remote_system_state_packet->body_acceleration[0],3)
        COPY_OUT_OF_AN_PACKET(remote_system_state_packet->g_force)
        COPY_OUT_OF_AN_PACKET_A(remote_system_state_packet->orientation[0], 3)
        COPY_OUT_OF_AN_PACKET_A(remote_system_state_packet->angular_velocity[0], 3)
        COPY_OUT_OF_AN_PACKET_A(remote_system_state_packet->position_standard_deviation[0], 3)
        COPY_OUT_OF_AN_PACKET_A(remote_system_state_packet->orientation_standard_deviation[0], 3)
        return 0;
    }
    else{ return 1; }
}

void encode_subsonus_remote_system_state_packet(an_packet_t * an_packet, subsonus_remote_system_state_packet_t * remote_system_state_packet){
    an_packet->id = packet_id_subsonus_remote_system_state;
    int count = 0;
    COPY_INTO_AN_PACKET(remote_system_state_packet->device_address)
    COPY_INTO_AN_PACKET(remote_system_state_packet->system_status_valid.r)
    COPY_INTO_AN_PACKET(remote_system_state_packet->filter_status_valid.r)
    COPY_INTO_AN_PACKET(remote_system_state_packet->system_status.r)
    COPY_INTO_AN_PACKET(remote_system_state_packet->filter_status.r)
    COPY_INTO_AN_PACKET(remote_system_state_packet->data_valid.r)
    COPY_INTO_AN_PACKET(remote_system_state_packet->unix_time_seconds)
    COPY_INTO_AN_PACKET(remote_system_state_packet->microseconds)
    COPY_INTO_AN_PACKET(remote_system_state_packet->latitude)
    COPY_INTO_AN_PACKET(remote_system_state_packet->longitude)
    COPY_INTO_AN_PACKET(remote_system_state_packet->height)
    COPY_INTO_AN_PACKET_A(remote_system_state_packet->velocity[0],3)
    COPY_INTO_AN_PACKET_A(remote_system_state_packet->body_acceleration[0],3)
    COPY_INTO_AN_PACKET(remote_system_state_packet->g_force)
    COPY_INTO_AN_PACKET_A(remote_system_state_packet->orientation[0], 3)
    COPY_INTO_AN_PACKET_A(remote_system_state_packet->angular_velocity[0], 3)
    COPY_INTO_AN_PACKET_A(remote_system_state_packet->position_standard_deviation[0], 3)
    COPY_INTO_AN_PACKET_A(remote_system_state_packet->orientation_standard_deviation[0], 3)
    an_packet->length = (uint8_t) count;
}

int decode_subsonus_raw_sensors_packet(subsonus_raw_sensors_packet_t * subsonus_raw_sensors_packet, an_packet_t * an_packet){
    if(an_packet->id == packet_id_subsonus_raw_sensors_packet && an_packet->length == 52){
        int count = 0;
        COPY_OUT_OF_AN_PACKET_A(subsonus_raw_sensors_packet->accelerometers[0],3)
        COPY_OUT_OF_AN_PACKET_A(subsonus_raw_sensors_packet->gyroscopes[0],3)
        COPY_OUT_OF_AN_PACKET_A(subsonus_raw_sensors_packet->magnetometers[0],3)
        COPY_OUT_OF_AN_PACKET(subsonus_raw_sensors_packet->internal_temperature)
        COPY_OUT_OF_AN_PACKET(subsonus_raw_sensors_packet->pressure_depth)
        COPY_OUT_OF_AN_PACKET(subsonus_raw_sensors_packet->water_temperature)
        COPY_OUT_OF_AN_PACKET(subsonus_raw_sensors_packet->velocity_of_sound)
        return 0;
    }
    else{ return 1; }
}

int decode_subsonus_remote_raw_sensors_packet(subsonus_remote_raw_sensors_packet_t * subsonus_remote_raw_sensors_packet, an_packet_t * an_packet){
    if(an_packet->id == packet_id_subsonus_remote_raw_sensors_packet && an_packet->length == 20){
        int count = 0;
        COPY_OUT_OF_AN_PACKET(subsonus_remote_raw_sensors_packet->device_id)
        COPY_OUT_OF_AN_PACKET(subsonus_remote_raw_sensors_packet->data_valid.r)
        COPY_OUT_OF_AN_PACKET(subsonus_remote_raw_sensors_packet->pressure_depth)
        COPY_OUT_OF_AN_PACKET(subsonus_remote_raw_sensors_packet->water_temperature)
        COPY_OUT_OF_AN_PACKET(subsonus_remote_raw_sensors_packet->velocity_of_sound)
        COPY_OUT_OF_AN_PACKET(subsonus_remote_raw_sensors_packet->internal_temperature)
        return 0;
    }
    else{ return 1; }
}

int decode_external_position_velocity_packet(external_position_velocity_packet_t *external_position_velocity_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_external_position_velocity && an_packet->length == 60)
	{
		memcpy(&external_position_velocity_packet->position[0], &an_packet->data[0], 3 * sizeof(double));
		memcpy(&external_position_velocity_packet->velocity[0], &an_packet->data[24], 3 * sizeof(float));
		memcpy(&external_position_velocity_packet->position_standard_deviation[0], &an_packet->data[36], 3 * sizeof(float));
		memcpy(&external_position_velocity_packet->velocity_standard_deviation[0], &an_packet->data[48], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_position_velocity_packet(an_packet_t *an_packet, external_position_velocity_packet_t *external_position_velocity_packet)
{
	an_packet->id = packet_id_external_position_velocity;
	an_packet->length = 60;
	memcpy(&an_packet->data[0], &external_position_velocity_packet->position[0], 3 * sizeof(double));
	memcpy(&an_packet->data[24], &external_position_velocity_packet->velocity[0], 3 * sizeof(float));
	memcpy(&an_packet->data[36], &external_position_velocity_packet->position_standard_deviation[0], 3 * sizeof(float));
	memcpy(&an_packet->data[48], &external_position_velocity_packet->velocity_standard_deviation[0], 3 * sizeof(float));
}

int decode_external_position_packet(external_position_packet_t *external_position_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_external_position && an_packet->length == 36)
	{
		memcpy(&external_position_packet->position[0], &an_packet->data[0], 3 * sizeof(double));
		memcpy(&external_position_packet->standard_deviation[0], &an_packet->data[24], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_position_packet(an_packet_t *an_packet, external_position_packet_t *external_position_packet)
{
	an_packet->id = packet_id_external_position;
	an_packet->length = 36;
	memcpy(&an_packet->data[0], &external_position_packet->position[0], 3 * sizeof(double));
	memcpy(&an_packet->data[24], &external_position_packet->standard_deviation[0], 3 * sizeof(float));
}

int decode_external_velocity_packet(external_velocity_packet_t *external_velocity_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_external_velocity && an_packet->length == 24)
	{
		memcpy(&external_velocity_packet->velocity[0], &an_packet->data[0], 3 * sizeof(float));
		memcpy(&external_velocity_packet->standard_deviation[0], &an_packet->data[12], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_velocity_packet(an_packet_t *an_packet, external_velocity_packet_t *external_velocity_packet)
{
	an_packet->id = packet_id_external_velocity;
	an_packet->length = 24;
	memcpy(&an_packet->data[0], &external_velocity_packet->velocity[0], 3 * sizeof(float));
	memcpy(&an_packet->data[12], &external_velocity_packet->standard_deviation[0], 3 * sizeof(float));
}

int decode_external_body_velocity_packet(external_body_velocity_packet_t *external_body_velocity_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_external_body_velocity && an_packet->length == 16)
	{
		memcpy(&external_body_velocity_packet->velocity, &an_packet->data[0], 3 * sizeof(float));
		memcpy(&external_body_velocity_packet->standard_deviation, &an_packet->data[12], sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_body_velocity_packet(an_packet_t *an_packet, external_body_velocity_packet_t *external_body_velocity_packet)
{
	an_packet->id = packet_id_external_body_velocity;
	an_packet->length = 16;
	memcpy(&an_packet->data[0], &external_body_velocity_packet->velocity[0], 3 * sizeof(float));
	memcpy(&an_packet->data[12], &external_body_velocity_packet->standard_deviation, sizeof(float));
}

int decode_external_heading_packet(external_heading_packet_t *external_heading_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_external_heading && an_packet->length == 8)
	{
		memcpy(&external_heading_packet->heading, &an_packet->data[0], sizeof(float));
		memcpy(&external_heading_packet->standard_deviation, &an_packet->data[4], sizeof(float));
		return 0;
	}
	else return 1;
}

void encode_external_heading_packet(an_packet_t *an_packet, external_heading_packet_t *external_heading_packet)
{
	an_packet->id = packet_id_external_heading;
	an_packet->length = 8;
	memcpy(&an_packet->data[0], &external_heading_packet->heading, sizeof(float));
	memcpy(&an_packet->data[4], &external_heading_packet->standard_deviation, sizeof(float));
}

int decode_subsonus_running_time_packet(subsonus_running_time_packet_t * subsonus_running_time_packet, an_packet_t * an_packet){
    if(an_packet->id == packet_id_subsonus_running_time && an_packet->length == 8){
        int count = 0;
        COPY_OUT_OF_AN_PACKET(subsonus_running_time_packet->seconds)
        COPY_OUT_OF_AN_PACKET(subsonus_running_time_packet->microseconds)
        return 0;
    }
    else{ return 1; }
}

int decode_subsonus_modem_status_packet(subsonus_modem_status_packet_t * status, an_packet_t * an_packet){
    if(an_packet->id == packet_id_subsonus_modem_status && an_packet->length == 26){
        int count = 0;
        COPY_OUT_OF_AN_PACKET(status->unix_time_seconds)
        COPY_OUT_OF_AN_PACKET(status->microseconds)
        COPY_OUT_OF_AN_PACKET(status->data_chunk_id)
        COPY_OUT_OF_AN_PACKET(status->modem_data_description.r)
        COPY_OUT_OF_AN_PACKET(status->link_status.r)
        COPY_OUT_OF_AN_PACKET(status->modem_event.r)
        COPY_OUT_OF_AN_PACKET(status->num_acoustic_transmissions)
        COPY_OUT_OF_AN_PACKET(status->num_acoustic_packets_lost)
        COPY_OUT_OF_AN_PACKET(status->num_bit_errors_detected)
        COPY_OUT_OF_AN_PACKET(status->num_bit_errors_corrected)
        COPY_OUT_OF_AN_PACKET(status->latency)
        return 0;
    }
    else{ return 1; }
}

int decode_subsonus_modem_data_packet(subsonus_modem_data_packet_t * modem_data, an_packet_t * an_packet){
    if(an_packet->id == packet_id_subsonus_modem_data && an_packet->length >= 27){
        int count = 0;
        COPY_OUT_OF_AN_PACKET(modem_data->unix_time_seconds)
        COPY_OUT_OF_AN_PACKET(modem_data->microseconds)
        COPY_OUT_OF_AN_PACKET(modem_data->serial_number[0])
        COPY_OUT_OF_AN_PACKET(modem_data->serial_number[1])
        COPY_OUT_OF_AN_PACKET(modem_data->serial_number[2])
        COPY_OUT_OF_AN_PACKET(modem_data->num_packets_in_sequence)
        COPY_OUT_OF_AN_PACKET(modem_data->packet_sequence_number)
        COPY_OUT_OF_AN_PACKET(modem_data->modem_data_description.r)
        COPY_OUT_OF_AN_PACKET(modem_data->data_size)
        if(an_packet->length != 27 + modem_data->data_size){
            return 1;
        }
        memcpy(&modem_data->data, &an_packet->data[count], modem_data->data_size);
        return 0;
    }
    else{ return 1; }
}

/* Start of configuration packet functions */

int decode_subsonus_packet_periods_packet(subsonus_packet_periods_packet_t *packet_periods_packet, an_packet_t *an_packet){
    if(an_packet->id == packet_id_subsonus_packet_periods && (an_packet->length - 1) % 3 == 0 && an_packet->length > 1){
        int i;
        const int packet_periods_count = (an_packet->length - 1) / 3;
        packet_periods_packet->clear_existing_packets = an_packet->data[0];
        for(i = 0; i < MAXIMUM_PACKET_PERIODS; i++){
            if(i < packet_periods_count){
                packet_periods_packet->packet_periods[i].port_id = an_packet->data[1 + 3*i];
                packet_periods_packet->packet_periods[i].packet_id = an_packet->data[1 + 3*i + 1];
                packet_periods_packet->packet_periods[i].packet_rate = an_packet->data[1 + 3*i + 2];
            }
            else memset(&packet_periods_packet->packet_periods[i], 0, sizeof(uint8_t)*3);
        }
        return 0;
    }
    else return 1;
}

void encode_subsonus_packet_periods_packet(an_packet_t *an_packet, subsonus_packet_periods_packet_t *packet_periods_packet){
    int i;
    an_packet->id = packet_id_subsonus_packet_periods;
    an_packet->data[0] = packet_periods_packet->clear_existing_packets;
    for(i = 0; i < MAXIMUM_PACKET_PERIODS; i++){
        if(packet_periods_packet->packet_periods[i].packet_id){
            an_packet->data[1 + 3*i] = packet_periods_packet->packet_periods[i].port_id;
            an_packet->data[1 + 3*i + 1] = packet_periods_packet->packet_periods[i].packet_id;
            an_packet->data[1 + 3*i + 2] = packet_periods_packet->packet_periods[i].packet_rate;
        }
        else break;
    }
    an_packet->length = 1 + 3*i;
}
int decode_subsonus_installation_alignment_packet(subsonus_installation_alignment_packet_t *subsonus_installation_alignment_packet, an_packet_t *an_packet)
{
    if(an_packet->id == packet_id_installation_alignment && an_packet->length == 73) // Includes Reserved space to match format of spatial packet
    {
        subsonus_installation_alignment_packet->reserved = an_packet->data[0];
        memcpy(&subsonus_installation_alignment_packet->alignment_dcm[0][0], &an_packet->data[1], 9*sizeof(float));
        subsonus_installation_alignment_packet->subsonus_orientation = an_packet->data[37];
        return 0;
    }
    else return 1;
}

void encode_subsonus_installation_alignment_packet(an_packet_t *an_packet, subsonus_installation_alignment_packet_t *subsonus_installation_alignment_packet)
{
    an_packet->id = packet_id_installation_alignment;
    an_packet->length = 73;
    an_packet->data[0] = subsonus_installation_alignment_packet->reserved;
    memcpy(&an_packet->data[1], &subsonus_installation_alignment_packet->alignment_dcm[0][0], 9*sizeof(float));
    an_packet->data[37] = subsonus_installation_alignment_packet->subsonus_orientation;
    memset(&an_packet->data[38],0,35);
}

int decode_subsonus_operation_mode_packet(subsonus_operation_mode_packet_t *subsonus_operation_mode_packet, an_packet_t *an_packet){
    if(an_packet->id == packet_id_subsonus_operation_mode && an_packet->length == 12){
        subsonus_operation_mode_packet->track_mode = an_packet->data[0];
        subsonus_operation_mode_packet->vehicle_profile = an_packet->data[1];
        subsonus_operation_mode_packet->enable_pressure_depth_aiding = an_packet->data[2];
        memcpy(&subsonus_operation_mode_packet->pressure_depth_offset, &an_packet->data[3], sizeof(float));
        subsonus_operation_mode_packet->force_fixed_velocity_of_sound = an_packet->data[7];
        memcpy(&subsonus_operation_mode_packet->fixed_velocity_of_sound, &an_packet->data[8], sizeof(float));
        return 0;
    }
    else return 1;
}

void encode_subsonus_operation_mode_packet(an_packet_t *an_packet, subsonus_operation_mode_packet_t *subsonus_operation_mode_packet){
    an_packet->id = packet_id_subsonus_operation_mode;
    an_packet->length = 12;
    an_packet->data[0] = subsonus_operation_mode_packet->track_mode;
    an_packet->data[1] = subsonus_operation_mode_packet->vehicle_profile;
    an_packet->data[2] = subsonus_operation_mode_packet->enable_pressure_depth_aiding;
    memcpy(&an_packet->data[3], &subsonus_operation_mode_packet->pressure_depth_offset, sizeof(float));
    an_packet->data[7] = subsonus_operation_mode_packet->force_fixed_velocity_of_sound;
    memcpy(&an_packet->data[8], &subsonus_operation_mode_packet->fixed_velocity_of_sound, sizeof(float));
}

int decode_subsonus_fixed_position_packet(subsonus_fixed_position_packet_t *subsonus_fixed_position_packet, an_packet_t *an_packet){
    if(an_packet->id == packet_id_subsonus_fixed_position && an_packet->length == 38){
        subsonus_fixed_position_packet->enable_fixed_position = an_packet->data[0];
        memcpy(&subsonus_fixed_position_packet->fixed_position_flags, &an_packet->data[1], sizeof(uint8_t));
        memcpy(&subsonus_fixed_position_packet->fixed_latitude, &an_packet->data[2], sizeof(double));
        memcpy(&subsonus_fixed_position_packet->fixed_longitude, &an_packet->data[10], sizeof(double));
        memcpy(&subsonus_fixed_position_packet->fixed_height, &an_packet->data[18], sizeof(double));
        memcpy(&subsonus_fixed_position_packet->fixed_roll, &an_packet->data[26], sizeof(float));
        memcpy(&subsonus_fixed_position_packet->fixed_pitch, &an_packet->data[30], sizeof(float));
        memcpy(&subsonus_fixed_position_packet->fixed_heading, &an_packet->data[34], sizeof(float));
        return 0;
    }
    else return 1;
}

void encode_subsonus_fixed_position_packet(an_packet_t *an_packet, subsonus_fixed_position_packet_t *subsonus_fixed_position_packet){
    an_packet->id = packet_id_subsonus_fixed_position;
    an_packet->length = 38;
    an_packet->data[0] = subsonus_fixed_position_packet->enable_fixed_position;
    memcpy(&an_packet->data[1], &subsonus_fixed_position_packet->fixed_position_flags, sizeof(uint8_t));
    memcpy(&an_packet->data[2], &subsonus_fixed_position_packet->fixed_latitude, sizeof(double));
    memcpy(&an_packet->data[10], &subsonus_fixed_position_packet->fixed_longitude, sizeof(double));
    memcpy(&an_packet->data[18], &subsonus_fixed_position_packet->fixed_height, sizeof(double));
    memcpy(&an_packet->data[26], &subsonus_fixed_position_packet->fixed_roll, sizeof(float));
    memcpy(&an_packet->data[30], &subsonus_fixed_position_packet->fixed_pitch, sizeof(float));
    memcpy(&an_packet->data[34], &subsonus_fixed_position_packet->fixed_heading, sizeof(float));
}

int decode_subsonus_device_address_packet(subsonus_device_address_packet_t *subsonus_device_address_packet, an_packet_t *an_packet){
    if(an_packet->id == packet_id_subsonus_device_address && an_packet->length == 2){
        int count = 0;
        COPY_OUT_OF_AN_PACKET(subsonus_device_address_packet->device_address)
        return 0;
    }
    else return 1;
}

void encode_subsonus_device_address_packet(an_packet_t *an_packet, subsonus_device_address_packet_t *subsonus_device_address_packet){
    an_packet->id = packet_id_subsonus_device_address;
    int count = 0;
    COPY_INTO_AN_PACKET(subsonus_device_address_packet->device_address)
    an_packet->length = count;
}

int decode_subsonus_time_configuration_packet(subsonus_time_configuration_packet_t *subsonus_time_configuration_packet, an_packet_t *an_packet){
	if(an_packet->id == packet_id_subsonus_time_configuration && an_packet->length == 194){
		int count = 0;
        COPY_OUT_OF_AN_PACKET(subsonus_time_configuration_packet->time_source)
        COPY_OUT_OF_AN_PACKET(subsonus_time_configuration_packet->time_config_flags.r)
        COPY_OUT_OF_AN_PACKET_A(subsonus_time_configuration_packet->ntp_server_1[0],NTP_SERVER_NAME_SIZE)
        COPY_OUT_OF_AN_PACKET_A(subsonus_time_configuration_packet->ntp_server_2[0],NTP_SERVER_NAME_SIZE)
        COPY_OUT_OF_AN_PACKET_A(subsonus_time_configuration_packet->ntp_server_3[0],NTP_SERVER_NAME_SIZE)
        return 0;
    }
    else return 1;
}

void encode_subsonus_time_configuration_packet(an_packet_t *an_packet, subsonus_time_configuration_packet_t *subsonus_time_configuration_packet){
		an_packet->id = packet_id_subsonus_time_configuration;
		int count = 0;
		COPY_INTO_AN_PACKET(subsonus_time_configuration_packet->time_source)
        COPY_INTO_AN_PACKET(subsonus_time_configuration_packet->time_config_flags.r)
        COPY_INTO_AN_PACKET_A(subsonus_time_configuration_packet->ntp_server_1[0],NTP_SERVER_NAME_SIZE)
        COPY_INTO_AN_PACKET_A(subsonus_time_configuration_packet->ntp_server_2[0],NTP_SERVER_NAME_SIZE)
        COPY_INTO_AN_PACKET_A(subsonus_time_configuration_packet->ntp_server_3[0],NTP_SERVER_NAME_SIZE)
		an_packet->length = count;
}


int decode_subsonus_nmea_packet_periods_packet(subsonus_nmea_packet_periods_packet_t *nmea_packet_periods_packet, an_packet_t *an_packet){
    if(an_packet->id == packet_id_subsonus_nmea_packet_periods && (an_packet->length - 2) % 3 == 0 && an_packet->length > 2){
        int i;
        const int packet_periods_count = (an_packet->length - 2) / 3;
        nmea_packet_periods_packet->nmea_fix_behaviour = an_packet->data[0];
        nmea_packet_periods_packet->clear_existing_packets = an_packet->data[1];
        for(i = 0; i < packet_periods_count; ++i){
            nmea_packet_periods_packet->packet_periods[i].port_id = an_packet->data[2 + 3*i];
            nmea_packet_periods_packet->packet_periods[i].packet_id = an_packet->data[2 + 3*i + 1];
            nmea_packet_periods_packet->packet_periods[i].packet_rate = an_packet->data[2 + 3*i + 2];
        }
        for(i = packet_periods_count; i < MAXIMUM_PACKET_PERIODS; ++i){
            memset(&nmea_packet_periods_packet->packet_periods[i], 0, sizeof(subsonus_packet_period_t));
        }
        return 0;
    }
    else return 1;
}

void encode_subsonus_nmea_packet_periods_packet(an_packet_t *an_packet, subsonus_nmea_packet_periods_packet_t *nmea_packet_periods_packet){
    int i;
    an_packet->id = packet_id_subsonus_nmea_packet_periods;
    an_packet->data[0] = nmea_packet_periods_packet->nmea_fix_behaviour;
    an_packet->data[1] = nmea_packet_periods_packet->clear_existing_packets;
    for(i = 0; i < MAXIMUM_PACKET_PERIODS; i++){
        if(nmea_packet_periods_packet->packet_periods[i].packet_id){
            an_packet->data[2 + 3*i] = nmea_packet_periods_packet->packet_periods[i].port_id;
            an_packet->data[2 + 3*i + 1] = nmea_packet_periods_packet->packet_periods[i].packet_id;
            an_packet->data[2 + 3*i + 2] = nmea_packet_periods_packet->packet_periods[i].packet_rate;
        }
        else break;
    }
    an_packet->length = 2 + 3*i;
}

void encode_subsonus_modem_configuration_packet(an_packet_t * an_packet, subsonus_modem_configuration_packet_t * config){
    an_packet->id = packet_id_subsonus_modem_configuration;
    int count = 0;
    COPY_INTO_AN_PACKET(config->reserved[0])
    COPY_INTO_AN_PACKET(config->reserved[1])
    COPY_INTO_AN_PACKET(config->unit_select)
    COPY_INTO_AN_PACKET(config->buffer_size)
    COPY_INTO_AN_PACKET(config->buffer_timeout)
    COPY_INTO_AN_PACKET(config->data_chunk_size)
    COPY_INTO_AN_PACKET(config->data_chunk_timeout)
    COPY_INTO_AN_PACKET(config->reserved_1)
    COPY_INTO_AN_PACKET(config->configuration_flags.r)
    COPY_INTO_AN_PACKET(config->reserved_2[0])
    COPY_INTO_AN_PACKET(config->reserved_2[1])
    COPY_INTO_AN_PACKET(config->reserved_2[2])
    an_packet->length = (uint8_t) count;
}

int decode_subsonus_modem_configuration_packet(subsonus_modem_configuration_packet_t * config, an_packet_t * an_packet){
    if(an_packet->id == packet_id_subsonus_modem_configuration && an_packet->length == 40){
        int count = 0;
        COPY_OUT_OF_AN_PACKET(config->reserved[0])
        COPY_OUT_OF_AN_PACKET(config->reserved[1])
        COPY_OUT_OF_AN_PACKET(config->unit_select)
        COPY_OUT_OF_AN_PACKET(config->buffer_size)
        COPY_OUT_OF_AN_PACKET(config->buffer_timeout)
        COPY_OUT_OF_AN_PACKET(config->data_chunk_size)
        COPY_OUT_OF_AN_PACKET(config->data_chunk_timeout)
        COPY_OUT_OF_AN_PACKET(config->reserved_1)
        COPY_OUT_OF_AN_PACKET(config->configuration_flags.r)
        COPY_OUT_OF_AN_PACKET(config->reserved_2[0])
        COPY_OUT_OF_AN_PACKET(config->reserved_2[1])
        COPY_OUT_OF_AN_PACKET(config->reserved_2[2])
        return 0;
    }
    else{ return 1; }
}

int decode_subsonus_sensor_ranges_packet(subsonus_sensor_ranges_packet_t *sensor_ranges_packet, an_packet_t *an_packet){
    if(an_packet->id == packet_id_subsonus_sensor_ranges && an_packet->length == 4){
        int count = 0;
        uint8_t reserved_value = 0;
        COPY_OUT_OF_AN_PACKET(reserved_value)
        COPY_OUT_OF_AN_PACKET(sensor_ranges_packet->accelerometers_range)
        COPY_OUT_OF_AN_PACKET(reserved_value)
        COPY_OUT_OF_AN_PACKET(sensor_ranges_packet->magnetometers_range)
        return 0;
    }
    else return 1;
}

void encode_subsonus_sensor_ranges_packet(an_packet_t *an_packet, const subsonus_sensor_ranges_packet_t *sensor_ranges_packet){
    an_packet->id = packet_id_subsonus_sensor_ranges;
    int count = 0;
    const uint8_t reserved_value = 0;
    COPY_INTO_AN_PACKET(reserved_value)
    COPY_INTO_AN_PACKET(sensor_ranges_packet->accelerometers_range)
    COPY_INTO_AN_PACKET(reserved_value)
    COPY_INTO_AN_PACKET(sensor_ranges_packet->magnetometers_range)
    an_packet->length = count;
}

int decode_subsonus_acoustic_data_priority_packet(subsonus_acoustic_data_priority_packet_t *priorities_packet, an_packet_t *an_packet){
    if(an_packet->id == packet_id_subsonus_acoustic_data_priority && (an_packet->length - 1) % 2 == 0 && an_packet->length > 1){
        int i;
        const int priorities_count = (an_packet->length - 1) / 2;
        priorities_packet->clear_existing_priorities = an_packet->data[0];
        for(i = 0; i < SUBSONUS_MAXIMUM_ACOUSTIC_DATA_PRIORITIES; i++){
            if(i < priorities_count){
                priorities_packet->priorities[i].acoustic_data_id = an_packet->data[1 + 2*i];
                priorities_packet->priorities[i].acoustic_data_priority = an_packet->data[1 + 2*i + 1];
            }
            else memset(&priorities_packet->priorities[i], 0, sizeof(uint8_t)*2);
        }
        return 0;
    }
    else return 1;
}

void encode_subsonus_acoustic_data_priority_packet(an_packet_t *an_packet, const subsonus_acoustic_data_priority_packet_t *priorities_packet){
    int i;
    an_packet->id = packet_id_subsonus_acoustic_data_priority;
    an_packet->data[0] = priorities_packet->clear_existing_priorities;
    for(i = 0; i < SUBSONUS_MAXIMUM_ACOUSTIC_DATA_PRIORITIES; i++){
        if(priorities_packet->priorities[i].acoustic_data_id){
            an_packet->data[1 + 2*i] = priorities_packet->priorities[i].acoustic_data_id;
            an_packet->data[1 + 2*i + 1] = priorities_packet->priorities[i].acoustic_data_priority;
        }
        else break;
    }
    an_packet->length = 1 + 2*i;
}

int decode_subsonus_port_configuration_packet(subsonus_port_configuration_packet_t *port_configuration_packet, an_packet_t *an_packet){
    if(an_packet->id == packet_id_subsonus_port_configuration && (an_packet->length == 9 + SERVER_ADDRESS_SIZE)){
        int count = 0;
        COPY_OUT_OF_AN_PACKET(port_configuration_packet->port_control.r)
        COPY_OUT_OF_AN_PACKET(port_configuration_packet->port_id)
        COPY_OUT_OF_AN_PACKET(port_configuration_packet->output_type)
        COPY_OUT_OF_AN_PACKET(port_configuration_packet->device_address)
        COPY_OUT_OF_AN_PACKET(port_configuration_packet->port_type)
        COPY_OUT_OF_AN_PACKET(port_configuration_packet->enable)
        COPY_OUT_OF_AN_PACKET(port_configuration_packet->port_number)
        COPY_OUT_OF_AN_PACKET_A(port_configuration_packet->server_address[0],SERVER_ADDRESS_SIZE)
        return 0;
    }
    return 1;
}

void encode_subsonus_port_configuration_packet(an_packet_t *an_packet, const subsonus_port_configuration_packet_t *port_configuration_packet){
    an_packet->id = packet_id_subsonus_port_configuration;
    int count = 0;
    COPY_INTO_AN_PACKET(port_configuration_packet->port_control.r)
    COPY_INTO_AN_PACKET(port_configuration_packet->port_id)
    COPY_INTO_AN_PACKET(port_configuration_packet->output_type)
    COPY_INTO_AN_PACKET(port_configuration_packet->device_address)
    COPY_INTO_AN_PACKET(port_configuration_packet->port_type)
    COPY_INTO_AN_PACKET(port_configuration_packet->enable)
    COPY_INTO_AN_PACKET(port_configuration_packet->port_number)
    COPY_INTO_AN_PACKET_A(port_configuration_packet->server_address[0],SERVER_ADDRESS_SIZE)
    an_packet->length = (uint8_t) count;
}
