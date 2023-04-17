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

#include "subsonus_device_packets.h"

#include <string.h>

#include "subsonus_packets.h"
#include "anpp_helper_macro.h"

const int device_configuration_header_size = 2;

device_types_e get_device_type_from_configuration_packet(an_packet_t *an_packet){
    if(an_packet->id == packet_id_subsonus_device_configuration && an_packet->length >= device_configuration_header_size){
        uint16_t device_type = 0;
        int count = 0;
        COPY_OUT_OF_AN_PACKET(device_type)
        return (device_types_e)device_type;
    }
    return Invalid_Device;
}

int get_device_id_from_configuration_packet(an_packet_t *an_packet){
    if(an_packet->id == packet_id_subsonus_device_configuration && an_packet->length >= device_configuration_header_size + 2){
        uint16_t device_type = 0, device_id = 0;
        int count = 0;
        COPY_OUT_OF_AN_PACKET(device_type)
        COPY_OUT_OF_AN_PACKET(device_id)
        return device_id;
    }
    return -1;
}

int get_device_address_from_configuration_packet(an_packet_t *an_packet){
    if(an_packet->id == packet_id_subsonus_device_configuration && an_packet->length >= device_configuration_header_size + 4){
        uint16_t device_type = 0, device_id = 0, device_address = 0;
        int count = 0;
        COPY_OUT_OF_AN_PACKET(device_type)
        COPY_OUT_OF_AN_PACKET(device_id)
        COPY_OUT_OF_AN_PACKET(device_address)
        return device_address;
    }
    return -1;
}

void encode_subsonus_device_configuration_packet(an_packet_t *an_packet, const uint16_t device_type){
    int count = 0;
    an_packet->id = packet_id_subsonus_device_configuration;
    COPY_INTO_AN_PACKET(device_type)
    an_packet->length = count;
}


void encode_subsonus_device_config_packet(an_packet_t *an_packet, const subsonus_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Advanced_Navigation_Subsonus);
    int count = an_packet->length;

    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->port)
    COPY_INTO_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

    COPY_INTO_AN_PACKET(device_config->tracked)

    COPY_INTO_AN_PACKET(device_config->operation_mode)

    COPY_INTO_AN_PACKET_A(device_config->serial_number[0],3)

    COPY_INTO_AN_PACKET(device_config->interrogation_period)
    an_packet->length = count;
}

int decode_subsonus_device_config_packet(subsonus_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Advanced_Navigation_Subsonus){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 2 + HOST_NAME_SIZE + 1 + 4 + 3*sizeof(uint32_t) + 4){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->port)
            COPY_OUT_OF_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

            COPY_OUT_OF_AN_PACKET(device_config->tracked)

            COPY_OUT_OF_AN_PACKET(device_config->operation_mode)

            COPY_OUT_OF_AN_PACKET_A(device_config->serial_number[0],3)

            COPY_OUT_OF_AN_PACKET(device_config->interrogation_period)
            return 0;
        }
    }
    return 1;
}

void encode_tag_device_config_packet(an_packet_t *an_packet, const tag_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Advanced_Navigation_Tag);
    int count = an_packet->length;

    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->tracked)

    COPY_INTO_AN_PACKET_A(device_config->serial_number[0],3)

    COPY_INTO_AN_PACKET(device_config->interrogation_period)
    an_packet->length = count;
}

int decode_tag_device_config_packet(tag_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Advanced_Navigation_Tag){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 1 + 3*sizeof(uint32_t) + 4){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->tracked)

            COPY_OUT_OF_AN_PACKET_A(device_config->serial_number[0],3)

            COPY_OUT_OF_AN_PACKET(device_config->interrogation_period)
            return 0;
        }
    }
    return 1;
}

void encode_advanced_navigation_gnss_compass_device_config_packet(an_packet_t *an_packet, const advanced_navigation_gnss_compass_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Advanced_Navigation_GNSS_Compass);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->port)
    COPY_INTO_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

    COPY_INTO_AN_PACKET_A(device_config->serial_number[0],3)

    COPY_INTO_AN_PACKET(device_config->enable)
    COPY_INTO_AN_PACKET(device_config->offset_x)
    COPY_INTO_AN_PACKET(device_config->offset_y)
    COPY_INTO_AN_PACKET(device_config->offset_z)
    an_packet->length = count;
}

int decode_advanced_navigation_gnss_compass_device_config_packet(advanced_navigation_gnss_compass_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Advanced_Navigation_GNSS_Compass){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 2 + HOST_NAME_SIZE + 3*sizeof(uint32_t) + 1 + 3*sizeof(float)){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->port)
            COPY_OUT_OF_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

            COPY_OUT_OF_AN_PACKET_A(device_config->serial_number[0],3)

            COPY_OUT_OF_AN_PACKET(device_config->enable)
            COPY_OUT_OF_AN_PACKET(device_config->offset_x)
            COPY_OUT_OF_AN_PACKET(device_config->offset_y)
            COPY_OUT_OF_AN_PACKET(device_config->offset_z)
            return 0;
        }
    }
    return 1;
}

void encode_advanced_navigation_ins_device_config_packet(an_packet_t *an_packet, const advanced_navigation_ins_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Advanced_Navigation_INS);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->port)
    COPY_INTO_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

    COPY_INTO_AN_PACKET_A(device_config->serial_number[0],3)

    COPY_INTO_AN_PACKET(device_config->external_ins)
    COPY_INTO_AN_PACKET(device_config->enable)
    COPY_INTO_AN_PACKET(device_config->offset_x)
    COPY_INTO_AN_PACKET(device_config->offset_y)
    COPY_INTO_AN_PACKET(device_config->offset_z)
    an_packet->length = count;
}

int decode_advanced_navigation_ins_device_config_packet(advanced_navigation_ins_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Advanced_Navigation_INS){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 2 + HOST_NAME_SIZE  + 3*sizeof(uint32_t) + 2 + 3*sizeof(float)){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->port)
            COPY_OUT_OF_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

            COPY_OUT_OF_AN_PACKET_A(device_config->serial_number[0],3)

            COPY_OUT_OF_AN_PACKET(device_config->external_ins)
            COPY_OUT_OF_AN_PACKET(device_config->enable)
            COPY_OUT_OF_AN_PACKET(device_config->offset_x)
            COPY_OUT_OF_AN_PACKET(device_config->offset_y)
            COPY_OUT_OF_AN_PACKET(device_config->offset_z)
            return 0;
        }
    }
    return 1;
}

void encode_fixed_tone_pinger_generic_device_config_packet(an_packet_t *an_packet, const fixed_tone_pinger_generic_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Fixed_Tone_Pinger_Generic);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->tracked)

    COPY_INTO_AN_PACKET(device_config->transmit_frequency)
    COPY_INTO_AN_PACKET(device_config->transmit_duration)
    an_packet->length = count;
}

int decode_fixed_tone_pinger_generic_device_config_packet(fixed_tone_pinger_generic_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Fixed_Tone_Pinger_Generic){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 1 + 8){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->tracked)

            COPY_OUT_OF_AN_PACKET(device_config->transmit_frequency)
            COPY_OUT_OF_AN_PACKET(device_config->transmit_duration)
            return 0;
        }
    }
    return 1;
}

void encode_fixed_tone_transponder_generic_device_config_packet(an_packet_t *an_packet, const fixed_tone_transponder_generic_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Fixed_Tone_Transponder_Generic);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->tracked)

    COPY_INTO_AN_PACKET(device_config->interrogation_period)
    COPY_INTO_AN_PACKET(device_config->interrogation_duration)
    COPY_INTO_AN_PACKET(device_config->interrogation_frequency)
    COPY_INTO_AN_PACKET(device_config->transmit_frequency)
    COPY_INTO_AN_PACKET(device_config->transmit_duration)
    COPY_INTO_AN_PACKET(device_config->reply_delay)
    an_packet->length = count;
}

int decode_fixed_tone_transponder_generic_device_config_packet(fixed_tone_transponder_generic_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Fixed_Tone_Transponder_Generic){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 1 + 4*6){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->tracked)

            COPY_OUT_OF_AN_PACKET(device_config->interrogation_period)
            COPY_OUT_OF_AN_PACKET(device_config->interrogation_duration)
            COPY_OUT_OF_AN_PACKET(device_config->interrogation_frequency)
            COPY_OUT_OF_AN_PACKET(device_config->transmit_frequency)
            COPY_OUT_OF_AN_PACKET(device_config->transmit_duration)
            COPY_OUT_OF_AN_PACKET(device_config->reply_delay)
            return 0;
        }
    }
    return 1;
}

void encode_fixed_tone_transponder_benthowave_device_config_packet(an_packet_t *an_packet, const fixed_tone_transponder_benthowave_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Fixed_Tone_Transponder_Benthowave);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->tracked)

    COPY_INTO_AN_PACKET(device_config->interrogation_period)
    COPY_INTO_AN_PACKET(device_config->interrogation_duration)
    COPY_INTO_AN_PACKET(device_config->interrogation_frequency)
    COPY_INTO_AN_PACKET(device_config->transmit_frequency)
    COPY_INTO_AN_PACKET(device_config->transmit_duration)
    COPY_INTO_AN_PACKET(device_config->reply_delay)
    COPY_INTO_AN_PACKET(device_config->one_way_delay)
    an_packet->length = count;
}

int decode_fixed_tone_transponder_benthowave_device_config_packet(fixed_tone_transponder_benthowave_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Fixed_Tone_Transponder_Benthowave){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 1 + 4*6 + 1){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->tracked)

            COPY_OUT_OF_AN_PACKET(device_config->interrogation_period)
            COPY_OUT_OF_AN_PACKET(device_config->interrogation_duration)
            COPY_OUT_OF_AN_PACKET(device_config->interrogation_frequency)
            COPY_OUT_OF_AN_PACKET(device_config->transmit_frequency)
            COPY_OUT_OF_AN_PACKET(device_config->transmit_duration)
            COPY_OUT_OF_AN_PACKET(device_config->reply_delay)
            COPY_OUT_OF_AN_PACKET(device_config->one_way_delay)
            return 0;
        }
    }
    return 1;
}

void encode_passive_device_config_packet(an_packet_t *an_packet, const passive_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Passive_Device);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)
    COPY_INTO_AN_PACKET(device_config->latitude)
    COPY_INTO_AN_PACKET(device_config->longitude)
    COPY_INTO_AN_PACKET(device_config->height)
    an_packet->length = count;
}

int decode_passive_device_config_packet(passive_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Passive_Device){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 8*3){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)
            COPY_OUT_OF_AN_PACKET(device_config->latitude)
            COPY_OUT_OF_AN_PACKET(device_config->longitude)
            COPY_OUT_OF_AN_PACKET(device_config->height)
            return 0;
        }
    }
    return 1;
}

void encode_fixed_tone_pinger_emergency_device_config_packet(an_packet_t *an_packet, const fixed_tone_pinger_emergency_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Fixed_Tone_Pinger_Emergency);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->tracked)
    an_packet->length = count;
}

int decode_fixed_tone_pinger_emergency_device_config_packet(fixed_tone_pinger_emergency_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Fixed_Tone_Pinger_Emergency){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 1){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->tracked)
            return 0;
        }
    }
    return 1;
}

void encode_hemisphere_gnss_compass_device_config_packet(an_packet_t *an_packet, const hemisphere_gnss_compass_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Hemisphere_GNSS_Compass);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->port)
    COPY_INTO_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

    COPY_INTO_AN_PACKET(device_config->enable)
    COPY_INTO_AN_PACKET(device_config->offset_x)
    COPY_INTO_AN_PACKET(device_config->offset_y)
    COPY_INTO_AN_PACKET(device_config->offset_z)
    an_packet->length = count;
}

int decode_hemisphere_gnss_compass_device_config_packet(hemisphere_gnss_compass_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Hemisphere_GNSS_Compass){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 2 + HOST_NAME_SIZE + 13){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->port)
            COPY_OUT_OF_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

            COPY_OUT_OF_AN_PACKET(device_config->enable)
            COPY_OUT_OF_AN_PACKET(device_config->offset_x)
            COPY_OUT_OF_AN_PACKET(device_config->offset_y)
            COPY_OUT_OF_AN_PACKET(device_config->offset_z)
            return 0;
        }
    }
    return 1;
}

void encode_wb2_transponder_device_config_packet(an_packet_t *an_packet, const wb2_transponder_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)WB2_Transponder);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->tracked)

    COPY_INTO_AN_PACKET(device_config->wake_enable)
    COPY_INTO_AN_PACKET(device_config->reply_code)
    COPY_INTO_AN_PACKET(device_config->interrogation_code)
    COPY_INTO_AN_PACKET(device_config->interrogation_period)
    COPY_INTO_AN_PACKET(device_config->reply_delay)
    an_packet->length = count;
}

int decode_wb2_transponder_device_config_packet(wb2_transponder_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == WB2_Transponder){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 1 + 1 + 2*2 + 2*4){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->tracked)

            COPY_OUT_OF_AN_PACKET(device_config->wake_enable)
            COPY_OUT_OF_AN_PACKET(device_config->reply_code)
            COPY_OUT_OF_AN_PACKET(device_config->interrogation_code)
            COPY_OUT_OF_AN_PACKET(device_config->interrogation_period)
            COPY_OUT_OF_AN_PACKET(device_config->reply_delay)
            return 0;
        }
    }
    return 1;
}

void encode_generic_nmea_gnss_compass_device_config_packet(an_packet_t *an_packet, const generic_nmea_gnss_compass_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Generic_NMEA_GNSS_Compass);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->port)
    COPY_INTO_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

    COPY_INTO_AN_PACKET(device_config->enable)
    COPY_INTO_AN_PACKET(device_config->offset_x)
    COPY_INTO_AN_PACKET(device_config->offset_y)
    COPY_INTO_AN_PACKET(device_config->offset_z)
    an_packet->length = count;
}

int decode_generic_nmea_gnss_compass_device_config_packet(generic_nmea_gnss_compass_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Generic_NMEA_GNSS_Compass){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 2 + HOST_NAME_SIZE + 13){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->port)
            COPY_OUT_OF_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

            COPY_OUT_OF_AN_PACKET(device_config->enable)
            COPY_OUT_OF_AN_PACKET(device_config->offset_x)
            COPY_OUT_OF_AN_PACKET(device_config->offset_y)
            COPY_OUT_OF_AN_PACKET(device_config->offset_z)
            return 0;
        }
    }
    return 1;
}

void encode_micron_transponder_device_config_packet(an_packet_t *an_packet, const micron_transponder_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Micron_Transponder);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->tracked)

    COPY_INTO_AN_PACKET(device_config->interrogation_code)
    COPY_INTO_AN_PACKET(device_config->interrogation_period)
    an_packet->length = count;
}

int decode_micron_transponder_device_config_packet(micron_transponder_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Micron_Transponder){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 1 + 2 + 4){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->tracked)

            COPY_OUT_OF_AN_PACKET(device_config->interrogation_code)
            COPY_OUT_OF_AN_PACKET(device_config->interrogation_period)
            return 0;
        }
    }
    return 1;
}

void encode_advanced_navigation_spatial_device_config_packet(an_packet_t *an_packet, const advanced_navigation_spatial_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Advanced_Navigation_Spatial);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->port)
    COPY_INTO_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

    COPY_INTO_AN_PACKET_A(device_config->serial_number[0],3)

    COPY_INTO_AN_PACKET(device_config->external_ins)
    COPY_INTO_AN_PACKET(device_config->offset_x)
    COPY_INTO_AN_PACKET(device_config->offset_y)
    COPY_INTO_AN_PACKET(device_config->offset_z)
    an_packet->length = count;
}

int decode_advanced_navigation_spatial_device_config_packet(advanced_navigation_spatial_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Advanced_Navigation_Spatial){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 2 + HOST_NAME_SIZE  + 3*sizeof(uint32_t) + 1 + 3*sizeof(float)){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->port)
            COPY_OUT_OF_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

            COPY_OUT_OF_AN_PACKET_A(device_config->serial_number[0],3)

            COPY_OUT_OF_AN_PACKET(device_config->external_ins)
            COPY_OUT_OF_AN_PACKET(device_config->offset_x)
            COPY_OUT_OF_AN_PACKET(device_config->offset_y)
            COPY_OUT_OF_AN_PACKET(device_config->offset_z)
            return 0;
        }
    }
    return 1;
}

void encode_advanced_navigation_fog_device_config_packet(an_packet_t *an_packet, const advanced_navigation_fog_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Advanced_Navigation_FOG);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->port)
    COPY_INTO_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

    COPY_INTO_AN_PACKET_A(device_config->serial_number[0],3)

    COPY_INTO_AN_PACKET(device_config->external_ins)
    COPY_INTO_AN_PACKET(device_config->offset_x)
    COPY_INTO_AN_PACKET(device_config->offset_y)
    COPY_INTO_AN_PACKET(device_config->offset_z)
    an_packet->length = count;
}

int decode_advanced_navigation_fog_device_config_packet(advanced_navigation_fog_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Advanced_Navigation_FOG){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 2 + HOST_NAME_SIZE  + 3*sizeof(uint32_t) + 1 + 3*sizeof(float)){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->port)
            COPY_OUT_OF_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

            COPY_OUT_OF_AN_PACKET_A(device_config->serial_number[0],3)

            COPY_OUT_OF_AN_PACKET(device_config->external_ins)
            COPY_OUT_OF_AN_PACKET(device_config->offset_x)
            COPY_OUT_OF_AN_PACKET(device_config->offset_y)
            COPY_OUT_OF_AN_PACKET(device_config->offset_z)
            return 0;
        }
    }
    return 1;
}

void encode_advanced_navigation_dual_device_config_packet(an_packet_t *an_packet, const advanced_navigation_dual_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Advanced_Navigation_Dual);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->port)
    COPY_INTO_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

    COPY_INTO_AN_PACKET_A(device_config->serial_number[0],3)

    COPY_INTO_AN_PACKET(device_config->external_ins)
    COPY_INTO_AN_PACKET(device_config->enable)
    COPY_INTO_AN_PACKET(device_config->offset_x)
    COPY_INTO_AN_PACKET(device_config->offset_y)
    COPY_INTO_AN_PACKET(device_config->offset_z)
    an_packet->length = count;
}

int decode_advanced_navigation_dual_device_config_packet(advanced_navigation_dual_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Advanced_Navigation_Dual){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 2 + HOST_NAME_SIZE  + 3*sizeof(uint32_t) + 2 + 3*sizeof(float)){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->port)
            COPY_OUT_OF_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

            COPY_OUT_OF_AN_PACKET_A(device_config->serial_number[0],3)

            COPY_OUT_OF_AN_PACKET(device_config->external_ins)
            COPY_OUT_OF_AN_PACKET(device_config->enable)
            COPY_OUT_OF_AN_PACKET(device_config->offset_x)
            COPY_OUT_OF_AN_PACKET(device_config->offset_y)
            COPY_OUT_OF_AN_PACKET(device_config->offset_z)
            return 0;
        }
    }
    return 1;
}

void encode_advanced_navigation_fog_dual_device_config_packet(an_packet_t *an_packet, const advanced_navigation_fog_dual_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Advanced_Navigation_FOG_Dual);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->port)
    COPY_INTO_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

    COPY_INTO_AN_PACKET_A(device_config->serial_number[0],3)

    COPY_INTO_AN_PACKET(device_config->external_ins)
    COPY_INTO_AN_PACKET(device_config->enable)
    COPY_INTO_AN_PACKET(device_config->offset_x)
    COPY_INTO_AN_PACKET(device_config->offset_y)
    COPY_INTO_AN_PACKET(device_config->offset_z)
    an_packet->length = count;
}

int decode_advanced_navigation_fog_dual_device_config_packet(advanced_navigation_fog_dual_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Advanced_Navigation_FOG_Dual){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 2 + HOST_NAME_SIZE  + 3*sizeof(uint32_t) + 2 + 3*sizeof(float)){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->port)
            COPY_OUT_OF_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

            COPY_OUT_OF_AN_PACKET_A(device_config->serial_number[0],3)

            COPY_OUT_OF_AN_PACKET(device_config->external_ins)
            COPY_OUT_OF_AN_PACKET(device_config->enable)
            COPY_OUT_OF_AN_PACKET(device_config->offset_x)
            COPY_OUT_OF_AN_PACKET(device_config->offset_y)
            COPY_OUT_OF_AN_PACKET(device_config->offset_z)
            return 0;
        }
    }
    return 1;
}


void encode_generic_nmea_depth_sensor_device_config_packet(an_packet_t *an_packet, const generic_nmea_depth_sensor_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Generic_NMEA_Depth_Sensor);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->port)
    COPY_INTO_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

    COPY_INTO_AN_PACKET(device_config->offset_x)
    COPY_INTO_AN_PACKET(device_config->offset_y)
    COPY_INTO_AN_PACKET(device_config->offset_z)
    an_packet->length = count;
}

int decode_generic_nmea_depth_sensor_device_config_packet(generic_nmea_depth_sensor_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Generic_NMEA_Depth_Sensor){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 2 + HOST_NAME_SIZE + 12){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->port)
            COPY_OUT_OF_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

            COPY_OUT_OF_AN_PACKET(device_config->offset_x)
            COPY_OUT_OF_AN_PACKET(device_config->offset_y)
            COPY_OUT_OF_AN_PACKET(device_config->offset_z)
            return 0;
        }
    }
    return 1;
}

void encode_generic_anpp_device_config_packet(an_packet_t *an_packet, const generic_anpp_device_config_t *device_config){
    encode_subsonus_device_configuration_packet(an_packet, (uint16_t)Generic_ANPP);
    int count = an_packet->length;
    COPY_INTO_AN_PACKET(device_config->device_id)
    COPY_INTO_AN_PACKET(device_config->device_address)
    COPY_INTO_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
    COPY_INTO_AN_PACKET(device_config->active)

    COPY_INTO_AN_PACKET(device_config->port)
    COPY_INTO_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

    COPY_INTO_AN_PACKET(device_config->enable)

    COPY_INTO_AN_PACKET(device_config->offset_x)
    COPY_INTO_AN_PACKET(device_config->offset_y)
    COPY_INTO_AN_PACKET(device_config->offset_z)

    COPY_INTO_AN_PACKET(device_config->alignment_x)
    COPY_INTO_AN_PACKET(device_config->alignment_y)
    COPY_INTO_AN_PACKET(device_config->alignment_z)
    an_packet->length = count;
}

int decode_generic_anpp_device_config_packet(generic_anpp_device_config_t *device_config, an_packet_t *an_packet){
    if(get_device_type_from_configuration_packet(an_packet) == Generic_ANPP){
        if(an_packet->length >= device_configuration_header_size + 5 + DISPLAY_NAME_SIZE + 2 + HOST_NAME_SIZE + 1 + 6*sizeof(float)){
            int count = device_configuration_header_size;
            COPY_OUT_OF_AN_PACKET(device_config->device_id)
            COPY_OUT_OF_AN_PACKET(device_config->device_address)
            COPY_OUT_OF_AN_PACKET_A(device_config->display_name[0],DISPLAY_NAME_SIZE)
            COPY_OUT_OF_AN_PACKET(device_config->active)

            COPY_OUT_OF_AN_PACKET(device_config->port)
            COPY_OUT_OF_AN_PACKET_A(device_config->hostname[0],HOST_NAME_SIZE)

            COPY_OUT_OF_AN_PACKET(device_config->enable)

            COPY_OUT_OF_AN_PACKET(device_config->offset_x)
            COPY_OUT_OF_AN_PACKET(device_config->offset_y)
            COPY_OUT_OF_AN_PACKET(device_config->offset_z)

            COPY_OUT_OF_AN_PACKET(device_config->alignment_x)
            COPY_OUT_OF_AN_PACKET(device_config->alignment_y)
            COPY_OUT_OF_AN_PACKET(device_config->alignment_z)
            return 0;
        }
    }
    return 1;
}
