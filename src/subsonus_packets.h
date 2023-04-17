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

#ifdef __cplusplus
extern "C"
{
#endif

#define MAXIMUM_PACKET_PERIODS (50)
#define MAXIMUM_DETAILED_SATELLITES (32)
#define NTP_SERVER_NAME_SIZE (64)
#define SERVER_ADDRESS_SIZE (32)
#define DISPLAY_NAME_SIZE (16)
#define HOST_NAME_SIZE (32)
#define SUBSONUS_MAXIMUM_ACOUSTIC_DATA_PRIORITIES (127)

#define START_SYSTEM_PACKETS 0
#define START_STATE_PACKETS 20
#define START_CONFIGURATION_PACKETS 180

typedef enum
{
	/*00*/packet_id_acknowledge,
	/*01*/packet_id_request,
	/*03*/packet_id_device_information = 3,
	/*04*/packet_id_restore_factory_settings,
	/*05*/packet_id_reset,
	/*11*/packet_id_network_settings = 11,
	/*12*/packet_id_subsonus_hostname,
	end_system_packets,

	/*20*/packet_id_subsonus_system_state = START_STATE_PACKETS,
	/*21*/packet_id_unix_time,
	/*22*/packet_id_formatted_time,
	/*23*/packet_id_subsonus_status,
	/*24*/packet_id_subsonus_track,
	/*25*/packet_id_subsonus_remote_system_state,

	/*28*/ packet_id_subsonus_raw_sensors_packet = 28,
	/*29*/ packet_id_subsonus_remote_raw_sensors_packet,

	/*44*/packet_id_external_position_velocity = 44,
	/*45*/packet_id_external_position,
	/*46*/packet_id_external_velocity,
	/*47*/packet_id_external_body_velocity,
	/*48*/packet_id_external_heading,
	/*49*/packet_id_subsonus_running_time,

	/*66*/ packet_id_subsonus_modem_status = 66,
	/*68*/ packet_id_subsonus_modem_data = 68,

	end_state_packets,

	/*181*/packet_id_subsonus_packet_periods = 181,
	/*184*/ packet_id_subsonus_sensor_ranges = 184,
	/*185*/packet_id_installation_alignment = 185,
	/*186*/packet_id_subsonus_operation_mode = 186,
	/*187*/ packet_id_subsonus_fixed_position = 187,
	/*192*/ packet_id_subsonus_device_address = 192,
	/*193*/ packet_id_subsonus_time_configuration = 193,
	/*195*/ packet_id_subsonus_nmea_packet_periods = 195,
	/*196*/ packet_id_subsonus_device_configuration = 196,
	/*197*/ packet_id_subsonus_port_configuration = 197,
	/*198*/ packet_id_subsonus_modem_configuration = 198,
    /*200*/ packet_id_subsonus_acoustic_data_priority = 200,

	end_configuration_packets
} packet_id_e;

/* start of system packets typedef structs */

typedef enum
{
	acknowledge_success,
	acknowledge_failure_crc,
	acknowledge_failure_length,
	acknowledge_failure_range,
	acknowledge_failure_flash,
	acknowledge_failure_not_ready,
	acknowledge_failure_unknown_packet
} acknowledge_result_e;

typedef struct
{
	uint8_t packet_id;
	uint16_t packet_crc;
	uint8_t acknowledge_result;
} acknowledge_packet_t;

typedef struct
{
	uint32_t software_version;
	uint32_t device_id;
	uint32_t hardware_revision;
	uint32_t serial_number[3];
} device_information_packet_t;

typedef enum
{
    link_auto,       /** Auto negotiation  */
    link_100mb_full, /** Force 100 Mb full duplex */
    link_100mb_half, /** Force 100 Mb half duplex */
    link_10mb_full,  /** Force 10 Mb full duplex */
    link_10mb_half  /** Force 10 Mb half duplex */
} network_link_mode_e;

typedef struct
{
	uint8_t permanent;
	union {
		uint8_t r;
		struct
		{
			unsigned int dhcp_enabled :1;
			unsigned int automatic_dns :1;
            unsigned int link_mode: 3;
			unsigned int reserved :3;
		} b;
	} dhcp_mode_flags;
	uint32_t static_ip_address;
	uint32_t static_netmask;
	uint32_t static_gateway;
	uint32_t static_dns_server;
	uint32_t serial_number[3];
} network_settings_packet_t;

typedef struct{
    char hostname[16];
} subsonus_hostname_packet_t;

/* start of state packets typedef structs */

typedef enum
{
	gnss_fix_none,
	gnss_fix_2d,
	gnss_fix_3d,
	gnss_fix_sbas,
	gnss_fix_differential,
	gnss_fix_omnistar,
	gnss_fix_rtk_float,
	gnss_fix_rtk_fixed
} gnss_fix_type_e;

typedef union{
    uint32_t r;

    struct{
        unsigned int system_failure : 1;
        unsigned int accelerometer_sensor_failure : 1;
        unsigned int gyroscope_sensor_failure : 1;
        unsigned int magnetometer_sensor_failure : 1;
        unsigned int pressure_sensor_failure : 1;
        unsigned int gnss_failure : 1;
        unsigned int accelerometer_over_range : 1;
        unsigned int gyroscope_over_range : 1;
        unsigned int magnetometer_over_range : 1;
        unsigned int pressure_over_range : 1;
        unsigned int maximum_temperature_shutdown_alarm : 1;
        unsigned int out_of_water_alarm : 1;
        unsigned int pressure_depth_offset_alarm : 1;
        unsigned int logging_memory_failure : 1;
        unsigned int filter_aiding_communication_failure : 1;
        unsigned int fixed_position_gnss_aiding_alarm : 1;
        unsigned int fixed_position_acoustic_aiding_alarm : 1;
        unsigned int multiple_aiding_source_alarm : 1;
        unsigned int aided_external_ins_alarm : 1;
        unsigned int aiding_gnss_time_sync_alarm : 1;
        unsigned int multiple_master_alarm : 1;
        unsigned int modem_data_disabled_alarm : 1;
        unsigned int brownout_alarm : 1;
        unsigned int aiding_alignment_alarm : 1;
        unsigned int reserved : 8;
    } b;
} subsonus_system_status_t;

typedef union{
    uint32_t r;

    struct{
        unsigned int orientation_filter_initialised : 1;
        unsigned int position_filter_initialised : 1;
        unsigned int heading_initialised : 1;
        unsigned int utc_time_initialised : 1;
        unsigned int gnss_fix_type : 3;
        unsigned int acoustic_heading_active : 1;
        unsigned int acoustic_position_active : 1;
        unsigned int internal_gnss_enabled : 1;
        unsigned int magnetic_heading_enabled : 1;
        unsigned int velocity_heading_enabled : 1;
        unsigned int pressure_depth_position_active : 1;
        unsigned int external_position_active : 1;
        unsigned int external_velocity_active : 1;
        unsigned int external_heading_active : 1;
        unsigned int dual_antenna_heading_active : 1;
        unsigned int fixed_position_active : 1;
        unsigned int fixed_heading_active : 1;
        unsigned int fixed_roll_pitch_active : 1;
        unsigned int pressure_depth_height_active : 1;
        unsigned int external_gnss_active : 1;
        unsigned int pressure_depth_initialised : 1;
        unsigned int internal_vos_initialised : 1;
        unsigned int usbl_tracking_active : 1;
        unsigned int reserved : 7;
    } b;
} subsonus_filter_status_t;

typedef struct{
    subsonus_system_status_t system_status;
    subsonus_filter_status_t filter_status;
    uint32_t unix_time_seconds;
    uint32_t microseconds;
    double latitude;
    double longitude;
    double height;
    float velocity[3];
    float body_acceleration[3];
    float g_force;
    float orientation[3];
    float angular_velocity[3];
    float position_standard_deviation[3];
    float orientation_standard_deviation[3];
} subsonus_system_state_packet_t;

typedef struct
{
	uint32_t unix_time_seconds;
	uint32_t microseconds;
} unix_time_packet_t;

typedef struct
{
	uint32_t microseconds;
	uint16_t year;
	uint16_t year_day;
	uint8_t month;
	uint8_t month_day;
	uint8_t week_day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} formatted_time_packet_t;

typedef struct{
    subsonus_system_status_t system_status;
    subsonus_filter_status_t filter_status;
} subsonus_status_packet_t;


typedef  union{
    uint8_t r;
    struct{
        unsigned int data_connection_active : 1;
        unsigned int depth_correction_applied : 1;
        unsigned int reserved : 6;
    } b;
} subsonus_tracking_status_t;

typedef union{
    uint32_t r;

    struct{
        unsigned int observer_time_valid : 1;
        unsigned int observer_position_valid : 1;
        unsigned int observer_velocity_valid : 1;
        unsigned int observer_orientation_valid : 1;
        unsigned int observer_position_standard_deviation_valid : 1;
        unsigned int observer_orientation_standard_deviation_valid : 1;
        unsigned int observer_depth_valid : 1;
        unsigned int age_valid : 1;
        unsigned int range_valid : 1;
        unsigned int azimuth_valid : 1;
        unsigned int elevation_valid : 1;
        unsigned int raw_position_valid : 1;
        unsigned int corrected_position_valid : 1;
        unsigned int ned_position_valid : 1;
        unsigned int geodetic_position_valid : 1;
        unsigned int range_standard_deviation_valid : 1;
        unsigned int azimuth_standard_deviation_valid : 1;
        unsigned int elevation_standard_deviation_valid : 1;
        unsigned int position_standard_deviation_valid : 1;
        unsigned int depth_valid : 1;
        unsigned int signal_level_valid : 1;
        unsigned int signal_to_noise_ratio_valid : 1;
        unsigned int signal_correlation_ratio_valid : 1;
        unsigned int signal_correlation_interference_valid : 1;
        unsigned int reserved : 8;
    } b;
} subsonus_track_data_valid_flags_t;


typedef struct{
    uint16_t device_address;
    subsonus_tracking_status_t tracking_status;
    subsonus_system_status_t observer_system_status;
    subsonus_filter_status_t observer_filter_status;
    subsonus_track_data_valid_flags_t data_valid;
    uint32_t observer_unix_time_seconds;
    uint32_t observer_microseconds;
    double observer_latitude;
    double observer_longitude;
    double observer_height;
    float observer_velocity[3];
    float observer_orientation[3];
    float observer_position_standard_deviation[3];
    float observer_orientation_standard_deviation[3];
    float observer_depth;
    uint32_t age_microseconds;
    float range;
    float azimuth;
    float elevation;
    float raw_position[3];
    float corrected_position[3];
    float ned_position[3];
    double latitude;
    double longitude;
    double height;
    float range_standard_deviation;
    float azimuth_standard_deviation;
    float elevation_standard_deviation;
    float latitude_standard_deviation;
    float longitude_standard_deviation;
    float height_standard_deviation;
    float depth;
    uint8_t signal_level;
    int8_t signal_to_noise_ratio;
    uint8_t signal_correlation_ratio;
    uint8_t signal_correlation_interference;
    uint32_t reserved;
} subsonus_track_packet_t;

typedef union{
    uint32_t r;

    struct{
        unsigned int time_seconds_valid : 1;
        unsigned int microseconds_valid : 1;
        unsigned int latitude_valid : 1;
        unsigned int longitude_valid : 1;
        unsigned int height_valid : 1;
        unsigned int velocity_north_valid : 1;
        unsigned int velocity_east_valid : 1;
        unsigned int velocity_down_valid : 1;
        unsigned int body_acceleration_X_valid : 1;
        unsigned int body_acceleration_Y_valid : 1;
        unsigned int body_acceleration_Z_valid : 1;
        unsigned int g_force_valid : 1;
        unsigned int roll_valid : 1;
        unsigned int pitch_valid : 1;
        unsigned int heading_valid : 1;
        unsigned int angular_velocity_X_valid : 1;
        unsigned int angular_velocity_Y_valid : 1;
        unsigned int angular_velocity_Z_valid : 1;
        unsigned int latitude_stdev_valid : 1;
        unsigned int longitude_stdev_valid : 1;
        unsigned int height_stdev_valid : 1;
        unsigned int roll_stdev_valid : 1;
        unsigned int pitch_stdev_valid : 1;
        unsigned int heading_stdev_valid : 1;
        unsigned int reserved : 8;
    } b;
} subsonus_state_data_valid_flags_t;

typedef struct{
    uint16_t device_address;
    subsonus_system_status_t system_status_valid;
    subsonus_filter_status_t filter_status_valid;
    subsonus_system_status_t system_status;
    subsonus_filter_status_t filter_status;
    subsonus_state_data_valid_flags_t data_valid;
    uint32_t unix_time_seconds;
    uint32_t microseconds;
    double latitude;
    double longitude;
    double height;
    float velocity[3];
    float body_acceleration[3];
    float g_force;
    float orientation[3];
    float angular_velocity[3];
    float position_standard_deviation[3];
    float orientation_standard_deviation[3];
} subsonus_remote_system_state_packet_t;

typedef struct{
    float accelerometers[3];
    float gyroscopes[3];
    float magnetometers[3];
    float internal_temperature;
    float pressure_depth;
    float water_temperature;
    float velocity_of_sound;
} subsonus_raw_sensors_packet_t;

typedef union{
    uint16_t r;

    struct{
        unsigned int pressure_depth_valid : 1;
        unsigned int water_temperature_valid : 1;
        unsigned int velocity_of_sound_valid : 1;
        unsigned int internal_temperature_valid : 1;
        unsigned int reserved : 12;
    } b;
} subsonus_raw_sensors_data_valid_flags_t;

typedef struct{
    uint16_t device_id;
    subsonus_raw_sensors_data_valid_flags_t data_valid;
    float pressure_depth;
    float water_temperature;
    float velocity_of_sound;
    float internal_temperature;
} subsonus_remote_raw_sensors_packet_t;

typedef struct
{
	double position[3];
	float velocity[3];
	float position_standard_deviation[3];
	float velocity_standard_deviation[3];
} external_position_velocity_packet_t;

typedef struct
{
	double position[3];
	float standard_deviation[3];
} external_position_packet_t;

typedef struct
{
	float velocity[3];
	float standard_deviation[3];
} external_velocity_packet_t;

typedef struct
{
	float velocity[3];
	float standard_deviation;
} external_body_velocity_packet_t;

typedef struct
{
	float heading;
	float standard_deviation;
} external_heading_packet_t;

typedef struct
{
	uint32_t seconds;
	uint32_t microseconds;
} subsonus_running_time_packet_t;


typedef union {
    uint16_t r;
    struct
    {
        unsigned int error_detection_enabled :1;
        unsigned int error_correction_enabled :1;
        unsigned int data_was_discarded :1;
        unsigned int data_timed_out :1;
        unsigned int data_was_corrupt :1;
        unsigned int data_was_corrected :1;
        unsigned int reserved :10;
    } b;
} subsonus_modem_data_description_t;

typedef union {
    uint16_t r;
    struct
    {
        unsigned int no_medium_available :1;
        unsigned int network_utilized :1;
        unsigned int acoustic_30k_utilized: 1;
        unsigned int reserved :13;
    } b;
} subsonus_modem_link_status_t;

typedef enum
{
    data_chunk_created_from_anpp,
    data_chunk_created_from_timeout,
    data_chunk_created_from_size,
    data_chunk_discard_buffer_overflow,
    data_chunk_discard_buffer_timeout,
    data_chunk_discard_preemptive_buffer_timeout,
    data_chunk_discard_buffer_contention,
    data_chunk_discard_out_of_sequence,
    data_chunk_discard_sender,
    data_chunk_pending_discard_buffer_timeout,
    sent_message,
    message_partial_acknowledgement,
    message_acknowledged,
    received_message
} subsonus_modem_event_type_e;

typedef union {
    uint16_t r;
    struct
    {
        unsigned int event_type :5;
        unsigned int reserved :11;
    } b;
} subsonus_modem_event_t;

typedef struct {
    uint32_t unix_time_seconds;
    uint32_t microseconds;
    uint16_t data_chunk_id;
    subsonus_modem_data_description_t modem_data_description;
    subsonus_modem_link_status_t link_status;
    subsonus_modem_event_t modem_event;
    uint16_t num_acoustic_transmissions;
    uint16_t num_acoustic_packets_lost;
    uint16_t num_bit_errors_detected;
    uint16_t num_bit_errors_corrected;
    uint16_t latency;
} subsonus_modem_status_packet_t;

#define SUBSONUS_MODEM_DATA_PACKET_MAX_SIZE 228
#define SUBSONUS_MODEM_FIRST_SEQUENCE_NUMBER 0

typedef struct {
    uint32_t unix_time_seconds;
    uint32_t microseconds;
    uint32_t serial_number[3];
    uint16_t num_packets_in_sequence;
    uint16_t packet_sequence_number;
    subsonus_modem_data_description_t modem_data_description;
    uint8_t data_size;
    uint8_t data[SUBSONUS_MODEM_DATA_PACKET_MAX_SIZE];
} subsonus_modem_data_packet_t;

/* start of configuration packets typedef structs */

typedef enum{
    DISABLED = 0,
    PERIOD_0_1_HZ,
    PERIOD_0_2_HZ,
    PERIOD_0_5_HZ,
    PERIOD_1_0_HZ,
    PERIOD_2_0_HZ,
    PERIOD_5_0_HZ,
    PERIOD_10_0_HZ,
    PERIOD_25_0_HZ,
    PERIOD_50_0_HZ,
} packet_rates_e;

typedef struct
{
    uint8_t port_id;
    uint8_t packet_id;
    uint8_t packet_rate;
} subsonus_packet_period_t;

typedef struct
{
    uint8_t clear_existing_packets;
    subsonus_packet_period_t packet_periods[MAXIMUM_PACKET_PERIODS];
} subsonus_packet_periods_packet_t;

typedef struct
{
    uint8_t reserved_1;
    uint8_t accelerometers_range;
    uint8_t reserved_2;
    uint8_t magnetometers_range;
} subsonus_sensor_ranges_packet_t;

typedef enum{
    subsonus_acoustic_data_id_none = 0,
    subsonus_acoustic_data_id_north,
    subsonus_acoustic_data_id_east,
    subsonus_acoustic_data_id_down,
    subsonus_acoustic_data_id_roll,
    subsonus_acoustic_data_id_pitch,
    subsonus_acoustic_data_id_heading,
    subsonus_acoustic_data_id_depth,
    subsonus_acoustic_data_id_vos,
    subsonus_acoustic_data_id_system_status,
    subsonus_acoustic_data_id_filter_status,
    subsonus_acoustic_data_id_modem_data,
    subsonus_acoustic_data_id_time,
} subsonus_acoustic_data_id_e;

typedef enum{
    subsonus_acoustic_data_priority_off = 0,
    subsonus_acoustic_data_priority_low,
    subsonus_acoustic_data_priority_medium,
    subsonus_acoustic_data_priority_high,
    subsonus_acoustic_data_priority_default,
} subsonus_acoustic_data_priority_e;

typedef struct
{
    uint8_t acoustic_data_id;
    uint8_t acoustic_data_priority;
} subsonus_acoustic_data_priority_t;

typedef struct
{
    uint8_t clear_existing_priorities;
    subsonus_acoustic_data_priority_t priorities[SUBSONUS_MAXIMUM_ACOUSTIC_DATA_PRIORITIES];
} subsonus_acoustic_data_priority_packet_t;

typedef enum{
    Orientation_Unchanged = 0,
    Orientation_Surface,
    Orientation_Subsea,
} subsonus_orientation_e;

typedef struct
{
    uint8_t reserved;
    float alignment_dcm[3][3];
    uint8_t subsonus_orientation;
} subsonus_installation_alignment_packet_t;

typedef enum{
    Operation_Unchanged = 0,
    Operation_Subsonus_Master,
    Operation_Subsonus_Slave,
    Operation_Subsonus_Listen_Only
} subsonus_track_mode_e;

typedef enum{
    Profile_Unchanged = 0,
    Profile_Unlimited,
    Profile_Stationary,
    Profile_Buoy,
    Profile_Diver,
    Profile_3D_Observation_Class,
    Profile_3D_Work_Class,
    Profile_Towfish,
    Profile_Glider,
    Profile_Small_Boat,
    Profile_Large_Ship,
} subsonus_vehicle_profile_e;

typedef struct
{
    uint8_t track_mode;
    uint8_t vehicle_profile;
    uint8_t enable_pressure_depth_aiding;
    float pressure_depth_offset;
    uint8_t force_fixed_velocity_of_sound;
    float fixed_velocity_of_sound;
} subsonus_operation_mode_packet_t;

typedef struct
{
    uint8_t enable_fixed_position;
    union{
        uint8_t r;
        struct{
            unsigned int internal_roll_pitch : 1;
            unsigned int reserved : 7;
        } b;
    } fixed_position_flags;
    double fixed_latitude;
    double fixed_longitude;
    double fixed_height;
    float fixed_roll;
    float fixed_pitch;
    float fixed_heading;
} subsonus_fixed_position_packet_t;

typedef struct
{
    uint16_t device_address;
} subsonus_device_address_packet_t;

typedef struct
{
    uint8_t time_source;
    union{
        uint8_t r;
        struct{
            unsigned int serve_ptp : 1;
            unsigned int serve_ntp : 1;
            unsigned int update_external : 1;
            unsigned int reserved : 5;
        } b;
    } time_config_flags;
    char ntp_server_1[NTP_SERVER_NAME_SIZE];
    char ntp_server_2[NTP_SERVER_NAME_SIZE];
    char ntp_server_3[NTP_SERVER_NAME_SIZE];
} subsonus_time_configuration_packet_t;

typedef struct
{
    uint8_t nmea_fix_behaviour;
    uint8_t clear_existing_packets;
    subsonus_packet_period_t packet_periods[MAXIMUM_PACKET_PERIODS];
} subsonus_nmea_packet_periods_packet_t;

/*Subsonus Port Configuration Structs*/
typedef union {
    uint8_t r;
    struct
    {
        unsigned int delete_port :1;
        unsigned int device_address_mask :1;
        unsigned int reserved :6;
    } b;
} subsonus_port_control_t;

typedef enum{
    TCP_server = 0,
    TCP_client = 1,
    UDP_server = 2,
    UDP_client = 3,
} subsonus_port_type_e;

typedef struct
{
    subsonus_port_control_t port_control;
    uint8_t port_id;
    uint8_t output_type;
    uint16_t device_address;
    uint8_t port_type;
    uint8_t enable;
    uint16_t port_number;
    char server_address[SERVER_ADDRESS_SIZE];
} subsonus_port_configuration_packet_t;

/*Subsonus Modem Configuration Structs*/
typedef union {
    uint16_t r;
    struct
    {
        unsigned int disable_tcp_aiding: 1;
        unsigned int disable_data_chunk_rules_for_anpp: 1;
        unsigned int reserved :11;
    } b;
} subsonus_modem_configuration_flags_t;

typedef struct {
    uint32_t reserved[2];
    uint32_t unit_select;
    uint16_t buffer_size;
    float buffer_timeout;
    uint16_t data_chunk_size;
    float data_chunk_timeout;
    uint16_t reserved_1;
    subsonus_modem_configuration_flags_t configuration_flags;
    uint32_t reserved_2[3];
} subsonus_modem_configuration_packet_t;

int decode_acknowledge_packet(acknowledge_packet_t *acknowledge_packet, an_packet_t *an_packet);
void encode_request_packet(an_packet_t *an_packet, uint8_t requested_packet_id);
int decode_device_information_packet(device_information_packet_t *device_information_packet, an_packet_t *an_packet);
void encode_restore_factory_settings_packet(an_packet_t *an_packet, int surface_mode);
void encode_reset_packet(an_packet_t *an_packet);
void encode_network_settings_packet(an_packet_t *an_packet,network_settings_packet_t *network_settings_packet);
void encode_extended_network_settings_packet(an_packet_t *an_packet,network_settings_packet_t *network_settings_packet);
int decode_network_settings_packet(network_settings_packet_t *network_settings_packet, an_packet_t *an_packet);
int decode_subsonus_hostname_packet(subsonus_hostname_packet_t * subsonus_hostname_packet, an_packet_t * an_packet);
void encode_subsonus_hostname_packet(an_packet_t * an_packet, subsonus_hostname_packet_t * subsonus_hostname_packet);

int decode_subsonus_system_state_packet(subsonus_system_state_packet_t * system_state_packet, an_packet_t * an_packet);
int decode_unix_time_packet(unix_time_packet_t *unix_time_packet, an_packet_t *an_packet);
int decode_formatted_time_packet(formatted_time_packet_t *formatted_time_packet, an_packet_t *an_packet);
int decode_subsonus_status_packet(subsonus_status_packet_t * status_packet, an_packet_t * an_packet);
int decode_subsonus_track_packet(subsonus_track_packet_t * system_state_packet, an_packet_t * an_packet);
int decode_subsonus_remote_system_state_packet(subsonus_remote_system_state_packet_t * remote_system_state_packet, an_packet_t * an_packet);
int decode_external_position_velocity_packet(external_position_velocity_packet_t *external_position_velocity_packet, an_packet_t *an_packet);
void encode_external_position_velocity_packet(an_packet_t *an_packet, external_position_velocity_packet_t *external_position_velocity_packet);
int decode_external_position_packet(external_position_packet_t *external_position_packet, an_packet_t *an_packet);
void encode_external_position_packet(an_packet_t *an_packet, external_position_packet_t *external_position_packet);
int decode_external_velocity_packet(external_velocity_packet_t *external_velocity_packet, an_packet_t *an_packet);
void encode_external_velocity_packet(an_packet_t *an_packet, external_velocity_packet_t *external_velocity_packet);
int decode_external_body_velocity_packet(external_body_velocity_packet_t *external_body_velocity_packet, an_packet_t *an_packet);
void encode_external_body_velocity_packet(an_packet_t *an_packet, external_body_velocity_packet_t *external_body_velocity_packet);
int decode_external_heading_packet(external_heading_packet_t *external_heading_packet, an_packet_t *an_packet);
void encode_external_heading_packet(an_packet_t *an_packet, external_heading_packet_t *external_heading_packet);
int decode_subsonus_running_time_packet(subsonus_running_time_packet_t * subsonus_running_time_packet, an_packet_t * an_packet);
int decode_subsonus_modem_status_packet(subsonus_modem_status_packet_t * status, an_packet_t * an_packet);
int decode_subsonus_modem_data_packet(subsonus_modem_data_packet_t * modem_data, an_packet_t * an_packet);

int decode_subsonus_packet_periods_packet(subsonus_packet_periods_packet_t *packet_periods_packet, an_packet_t *an_packet);
void encode_subsonus_packet_periods_packet(an_packet_t *an_packet, subsonus_packet_periods_packet_t *packet_periods_packet);
int decode_subsonus_sensor_ranges_packet(subsonus_sensor_ranges_packet_t *sensor_ranges_packet, an_packet_t *an_packet);
void encode_subsonus_sensor_ranges_packet(an_packet_t *an_packet, const subsonus_sensor_ranges_packet_t *sensor_ranges_packet);
int decode_subsonus_installation_alignment_packet(subsonus_installation_alignment_packet_t *installation_alignment_packet, an_packet_t *an_packet);
void encode_subsonus_installation_alignment_packet(an_packet_t *an_packet, subsonus_installation_alignment_packet_t *installation_alignment_packet);
int decode_subsonus_operation_mode_packet(subsonus_operation_mode_packet_t *subsonus_operation_mode_packet, an_packet_t *an_packet);
void encode_subsonus_operation_mode_packet(an_packet_t *an_packet, subsonus_operation_mode_packet_t *subsonus_operation_mode_packet);
int decode_subsonus_fixed_position_packet(subsonus_fixed_position_packet_t *subsonus_fixed_position_packet, an_packet_t *an_packet);
void encode_subsonus_fixed_position_packet(an_packet_t *an_packet, subsonus_fixed_position_packet_t *subsonus_fixed_position_packet);
int decode_subsonus_device_address_packet(subsonus_device_address_packet_t *subsonus_device_address_packet, an_packet_t *an_packet);
void encode_subsonus_device_address_packet(an_packet_t *an_packet, subsonus_device_address_packet_t *subsonus_device_address_packet);
int decode_subsonus_time_configuration_packet(subsonus_time_configuration_packet_t *subsonus_time_configuration_packet, an_packet_t *an_packet);
void encode_subsonus_time_configuration_packet(an_packet_t *an_packet, subsonus_time_configuration_packet_t *subsonus_time_configuration_packet);
int decode_subsonus_nmea_packet_periods_packet(subsonus_nmea_packet_periods_packet_t *nmea_packet_periods_packet, an_packet_t *an_packet);
void encode_subsonus_nmea_packet_periods_packet(an_packet_t *an_packet, subsonus_nmea_packet_periods_packet_t *nmea_packet_periods_packet);
void encode_subsonus_modem_configuration_packet(an_packet_t * an_packet, subsonus_modem_configuration_packet_t * config);
int decode_subsonus_modem_configuration_packet(subsonus_modem_configuration_packet_t * config, an_packet_t * an_packet);
int decode_subsonus_acoustic_data_priority_packet(subsonus_acoustic_data_priority_packet_t *priorities_packet, an_packet_t *an_packet);
void encode_subsonus_acoustic_data_priority_packet(an_packet_t *an_packet, const subsonus_acoustic_data_priority_packet_t *priorities_packet);
int decode_subsonus_port_configuration_packet(subsonus_port_configuration_packet_t *port_configuration_packet, an_packet_t *an_packet);
void encode_subsonus_port_configuration_packet(an_packet_t *an_packet, const subsonus_port_configuration_packet_t *port_configuration_packet);

#ifdef __cplusplus
}
#endif
