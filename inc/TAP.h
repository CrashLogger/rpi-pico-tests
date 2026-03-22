# pragma once
#include <cstdint>

class TAP{

    public:
    TAP();
    /*
        About the "pragma" lines: I copied them from someone's NRF24L01 driver example
        They are meant to guarantee that the structs don't get padded when we don't tell them to
        I don't really know what they're up to, nor am I interested
        It works with them and they shall stay there
    */

    public:
    // ========================================================================================
    //vars and datatypes

    enum TAP_ERROR : uint8_t {
        TAP_OK = 0,
        TAP_ERROR_CRC_MISMATCH = 1,
        TAP_ERROR_INVALID_LENGTH = 2
    };

    enum TAP_MESSAGE_TYPE : uint8_t {
        ACK_NACK = 0x00,
        DIRECT_COMMAND = 0x01,
        INDIRECT_COMMAND = 0x02,

        //RESERVED until 0x10

        TELEMETRY = 0x10,

        //RESERVED until 0x1F

        //AVAILABLE: 0x20 -> 0xFD

        NEGOTIATE_DATALINK = 0xFE,
        TELEMETRY_DATALINK = 0xFF
    };


    #pragma pack(push, 1)
    struct TAP_ADDRESS_HEADER{
        uint16_t sof_word;
        uint8_t target_id;
        uint8_t source_id;
        uint8_t message_len;
        uint8_t message_type;
        uint16_t cobs;
    };
    #pragma pack(pop)

    #pragma pack(push, 1)
    struct TAP_ACK_NACK{
        uint8_t ack_type;
        uint8_t token[3];
    };
    #pragma pack(pop)

    #pragma pack(push, 1)
    struct TAP_TELEMETRY{
        float lat;
        float lon;
        uint16_t alt;
        int16_t heading;
        float roll;
        float pitch;
    };
    #pragma pack(pop)

    #pragma pack(push, 1)
    struct TAP_DATALINK_TELEMETRY{
        uint16_t rssi;
        uint16_t snr;
        uint16_t rtt;
        uint16_t sent_pkts;
        uint16_t delta_t;
        uint16_t reserved;
    };
    #pragma pack(pop)

    #pragma pack(push, 1)
    struct TAP_INDIRECT_COMMAND{
        uint16_t bools;
        uint16_t reserved;
        float lat;
        float lon;
        uint16_t alt;
        int16_t heading;
    };
    #pragma pack(pop)

    #pragma pack(push, 1)
    struct TAP_TRAILER{
        uint16_t crc_16;
        uint16_t eof_word;
    };
    #pragma pack(pop)

    // ========================================================================================
    //Methods

    uint8_t tapInit(uint8_t source_id);

    //"Async"-ly gather telemetry data and send a telemetry packet
    uint8_t tapSendTelem(const TAP_TELEMETRY &telemetry);

    //This should always serialize, no matter what it is given
    uint8_t serialize(const TAP_ADDRESS_HEADER *header, const void *payload, TAP_TRAILER *trailer, uint8_t *buffer, uint8_t max_len);

    //This should be able to fail to detect a correct struct after unraveling the header
    uint8_t* deserialize(uint8_t *raw_message, uint8_t* message);

    private:
    //vars and datatypes
    uint32_t messages_since_last_datalink_telem;
    TAP_ADDRESS_HEADER tx_header;
    

};


