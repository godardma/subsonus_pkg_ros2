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

#ifndef SUBSONUS_DEVICE_PACKETS_H
#define SUBSONUS_DEVICE_PACKETS_H

#include "an_packet_protocol.h"


#ifdef __cplusplus
extern "C"
{
#endif

typedef enum{
    Null_Device = 0,
    Advanced_Navigation_Subsonus = 1,
    Advanced_Navigation_Tag = 2,
    Advanced_Navigation_GNSS_Compass = 3,
    Advanced_Navigation_INS = 4,
    Fixed_Tone_Pinger_Generic = 5,
    Fixed_Tone_Transponder_Generic = 6,
    Fixed_Tone_Transponder_Benthowave = 7,
    Passive_Device = 8,
    Fixed_Tone_Pinger_Emergency = 9,
    Hemisphere_GNSS_Compass = 10,
    WB2_Transponder = 11,
    Generic_NMEA_GNSS_Compass = 12,
    Micron_Transponder = 13,

    Advanced_Navigation_Spatial = 14,
    Advanced_Navigation_FOG = 15,
    Advanced_Navigation_Dual = 16,
    Advanced_Navigation_FOG_Dual = 17,

    Generic_NMEA_Depth_Sensor = 18,
    Generic_ANPP = 19,

    Invalid_Device // this marks the end of the enumeration for device types
} device_types_e;

typedef struct{
    uint16_t device_type;
} subsonus_device_configuration_packet_t;

extern const int device_configuration_header_size;

device_types_e get_device_type_from_configuration_packet(an_packet_t *an_packet);
int get_device_id_from_configuration_packet(an_packet_t *an_packet);
int get_device_address_from_configuration_packet(an_packet_t *an_packet);
void encode_subsonus_device_configuration_packet(an_packet_t *an_packet, const uint16_t device_type);

#define DISPLAY_NAME_SIZE 16
#define HOST_NAME_SIZE 32
typedef struct{
    uint16_t device_id;
    uint16_t device_address;                // Read Only overwritten by network discovery
    char display_name[DISPLAY_NAME_SIZE];   // Read Only overwritten by network discovery 
    uint8_t active;

    uint16_t port;                          // Read Only overwritten by network discovery
    char hostname[HOST_NAME_SIZE];          // Read Only overwritten by network discovery

    uint8_t tracked;

    int operation_mode;                     // Read Only overwritten by network discovery subsonus_orientation_e ??

    uint32_t serial_number[3];              // Read Only overwritten by network discovery

    float interrogation_period;
} subsonus_device_config_t;

void encode_subsonus_device_config_packet(an_packet_t *an_packet, const subsonus_device_config_t *device_config);
int decode_subsonus_device_config_packet(subsonus_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;                // Read Only overwritten by network discovery
    char display_name[DISPLAY_NAME_SIZE];   // Read Only overwritten by network discovery 
    uint8_t active;

    uint8_t tracked;

    uint32_t serial_number[3];              // Read Only overwritten by network discovery

    float interrogation_period;
} tag_device_config_t;

void encode_tag_device_config_packet(an_packet_t *an_packet, const tag_device_config_t *device_config);
int decode_tag_device_config_packet(tag_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint16_t port;
    char hostname[HOST_NAME_SIZE];

    uint32_t serial_number[3];              // Read Only overwritten by network discovery
    uint8_t enable;
    float offset_x;
    float offset_y;
    float offset_z;
} advanced_navigation_gnss_compass_device_config_t;

void encode_advanced_navigation_gnss_compass_device_config_packet(an_packet_t *an_packet, const advanced_navigation_gnss_compass_device_config_t *device_config);
int decode_advanced_navigation_gnss_compass_device_config_packet(advanced_navigation_gnss_compass_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint16_t port;
    char hostname[HOST_NAME_SIZE];

    uint32_t serial_number[3];              // Read Only overwritten by network discovery

    uint8_t external_ins;
    uint8_t enable;
    float offset_x;
    float offset_y;
    float offset_z;
} advanced_navigation_ins_device_config_t;

void encode_advanced_navigation_ins_device_config_packet(an_packet_t *an_packet, const advanced_navigation_ins_device_config_t *device_config);
int decode_advanced_navigation_ins_device_config_packet(advanced_navigation_ins_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint8_t tracked;

    float transmit_frequency;
    float transmit_duration;
} fixed_tone_pinger_generic_device_config_t;

void encode_fixed_tone_pinger_generic_device_config_packet(an_packet_t *an_packet, fixed_tone_pinger_generic_device_config_t *device_config);
int decode_fixed_tone_pinger_generic_device_config_packet(fixed_tone_pinger_generic_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint8_t tracked;

    float interrogation_period;
    float interrogation_duration;
    float interrogation_frequency;
    float transmit_frequency;
    float transmit_duration;
    float reply_delay;

} fixed_tone_transponder_generic_device_config_t;

void encode_fixed_tone_transponder_generic_device_config_packet(an_packet_t *an_packet, fixed_tone_transponder_generic_device_config_t *device_config);
int decode_fixed_tone_transponder_generic_device_config_packet(fixed_tone_transponder_generic_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint8_t tracked;

    float interrogation_period;
    float interrogation_duration;
    float interrogation_frequency;
    float transmit_frequency;
    float transmit_duration;
    float reply_delay;
    uint8_t one_way_delay;

} fixed_tone_transponder_benthowave_device_config_t;

void encode_fixed_tone_transponder_benthowave_device_config_packet(an_packet_t *an_packet, const fixed_tone_transponder_benthowave_device_config_t *device_config);
int decode_fixed_tone_transponder_benthowave_device_config_packet(fixed_tone_transponder_benthowave_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;
    double latitude;
    double longitude;
    double height;

} passive_device_config_t;

void encode_passive_device_config_packet(an_packet_t *an_packet, const passive_device_config_t *device_config);
int decode_passive_device_config_packet(passive_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint8_t tracked;

} fixed_tone_pinger_emergency_device_config_t;

void encode_fixed_tone_pinger_emergency_device_config_packet(an_packet_t *an_packet, const fixed_tone_pinger_emergency_device_config_t *device_config);
int decode_fixed_tone_pinger_emergency_device_config_packet(fixed_tone_pinger_emergency_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint16_t port;
    char hostname[HOST_NAME_SIZE];

    uint8_t enable;
    float offset_x;
    float offset_y;
    float offset_z;

} hemisphere_gnss_compass_device_config_t;

void encode_hemisphere_gnss_compass_device_config_packet(an_packet_t *an_packet, const hemisphere_gnss_compass_device_config_t *device_config);
int decode_hemisphere_gnss_compass_device_config_packet(hemisphere_gnss_compass_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint8_t tracked;

    uint8_t wake_enable;
    uint16_t reply_code;
    uint16_t interrogation_code;
    float interrogation_period;
    float reply_delay;

} wb2_transponder_device_config_t;

void encode_wb2_transponder_device_config_packet(an_packet_t *an_packet, const wb2_transponder_device_config_t *device_config);
int decode_wb2_transponder_device_config_packet(wb2_transponder_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint16_t port;
    char hostname[HOST_NAME_SIZE];

    uint8_t enable;
    float offset_x;
    float offset_y;
    float offset_z;

} generic_nmea_gnss_compass_device_config_t;

void encode_generic_nmea_gnss_compass_device_config_packet(an_packet_t *an_packet, generic_nmea_gnss_compass_device_config_t *device_config);
int decode_generic_nmea_gnss_compass_device_config_packet(generic_nmea_gnss_compass_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint8_t tracked;

    uint16_t interrogation_code;
    float interrogation_period;

} micron_transponder_device_config_t;

void encode_micron_transponder_device_config_packet(an_packet_t *an_packet, const micron_transponder_device_config_t *device_config);
int decode_micron_transponder_device_config_packet(micron_transponder_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint16_t port;
    char hostname[HOST_NAME_SIZE];

    uint32_t serial_number[3];              // Read Only overwritten by network discovery

    uint8_t external_ins;
    float offset_x;
    float offset_y;
    float offset_z;
} advanced_navigation_spatial_device_config_t;

void encode_advanced_navigation_spatial_device_config_packet(an_packet_t *an_packet, const advanced_navigation_spatial_device_config_t *device_config);
int decode_advanced_navigation_spatial_device_config_packet(advanced_navigation_spatial_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint16_t port;
    char hostname[HOST_NAME_SIZE];

    uint32_t serial_number[3];              // Read Only overwritten by network discovery

    uint8_t external_ins;
    float offset_x;
    float offset_y;
    float offset_z;
} advanced_navigation_fog_device_config_t;

void encode_advanced_navigation_fog_device_config_packet(an_packet_t *an_packet, const advanced_navigation_fog_device_config_t *device_config);
int decode_advanced_navigation_fog_device_config_packet(advanced_navigation_fog_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint16_t port;
    char hostname[HOST_NAME_SIZE];

    uint32_t serial_number[3];              // Read Only overwritten by network discovery

    uint8_t external_ins;
    uint8_t enable;
    float offset_x;
    float offset_y;
    float offset_z;
} advanced_navigation_dual_device_config_t;

void encode_advanced_navigation_dual_device_config_packet(an_packet_t *an_packet, const advanced_navigation_dual_device_config_t *device_config);
int decode_advanced_navigation_dual_device_config_packet(advanced_navigation_dual_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint16_t port;
    char hostname[HOST_NAME_SIZE];

    uint32_t serial_number[3];              // Read Only overwritten by network discovery

    uint8_t external_ins;
    uint8_t enable;
    float offset_x;
    float offset_y;
    float offset_z;
} advanced_navigation_fog_dual_device_config_t;

void encode_advanced_navigation_fog_dual_device_config_packet(an_packet_t *an_packet, const advanced_navigation_fog_dual_device_config_t *device_config);
int decode_advanced_navigation_fog_dual_device_config_packet(advanced_navigation_fog_dual_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint16_t port;
    char hostname[HOST_NAME_SIZE];

    float offset_x;
    float offset_y;
    float offset_z;

} generic_nmea_depth_sensor_device_config_t;

void encode_generic_nmea_depth_sensor_device_config_packet(an_packet_t *an_packet, const generic_nmea_depth_sensor_device_config_t *device_config);
int decode_generic_nmea_depth_sensor_device_config_packet(generic_nmea_depth_sensor_device_config_t *device_config, an_packet_t *an_packet);

typedef struct{
    uint16_t device_id;
    uint16_t device_address;
    char display_name[DISPLAY_NAME_SIZE];
    uint8_t active;

    uint16_t port;
    char hostname[HOST_NAME_SIZE];

    uint8_t enable;
    float offset_x;
    float offset_y;
    float offset_z;

    float alignment_x;
    float alignment_y;
    float alignment_z;

} generic_anpp_device_config_t;

void encode_generic_anpp_device_config_packet(an_packet_t *an_packet, const generic_anpp_device_config_t *device_config);
int decode_generic_anpp_device_config_packet(generic_anpp_device_config_t *device_config, an_packet_t *an_packet);

#ifdef __cplusplus
}
#endif

#endif
