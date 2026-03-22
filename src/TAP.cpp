// TAP.cpp
#include "pico/stdlib.h"
#include <stdio.h>
#include "TAP.h" 
#include <cstring>

// Constructor

    TAP::TAP() {
        messages_since_last_datalink_telem = 0;
    }

    uint8_t tapInit(uint8_t source_id){
        
    }


    uint8_t TAP::serialize(const TAP_ADDRESS_HEADER *header, const void *payload, TAP_TRAILER *trailer, uint8_t *buffer, uint8_t max_len) {
        if (header->message_len + sizeof(TAP_ADDRESS_HEADER) + sizeof(TAP_TRAILER) > max_len) {
            return -1;
        }
        // Serialize header
        memcpy(buffer, header, sizeof(TAP_ADDRESS_HEADER));
        uint8_t offset = sizeof(TAP_ADDRESS_HEADER);

        // Serialize payload
        memcpy(buffer + offset, payload, header->message_len);
        offset += header->message_len;

        // Serialize trailer, if one was provided (it should be!)
        if (trailer) {
            memcpy(buffer + offset, trailer, sizeof(TAP_TRAILER));
        }
        return offset + sizeof(TAP_TRAILER);
    }
    
    //TODO: Look for the SOF word and implement COBS!
    uint8_t TAP::tapSendTelem(const TAP_TELEMETRY &telemetry) {
        uint8_t buffer[255];
        TAP::TAP_ADDRESS_HEADER header;
        TAP::TAP_TRAILER trailer;
        header.sof_word = 0xAA55;
        header.message_type = TELEMETRY;
        header.message_len = sizeof(TAP_TELEMETRY);

        trailer.eof_word = 0xAA55;

        uint8_t len = serialize(&header, &telemetry, &trailer, buffer, sizeof(buffer));
        if (len == 0) return TAP::TAP_ERROR_INVALID_LENGTH;

        for (uint8_t i = 0; i < len; i++) {
            printf("%02X ", buffer[i]);
        }
        printf("\t\n");

        return TAP_OK;
    }


    //This should be able to fail to detect a correct struct after unraveling the header
    uint8_t* deserialize(uint8_t *raw_message, uint8_t* message);