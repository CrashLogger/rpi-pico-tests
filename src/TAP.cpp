class TAP{

    //Constructor
    TAP() {
    }

    /*
        About the "pragma" lines: I copied them from someone's NRF24L01 driver example
        They are meant to guarantee that the structs don't get padded when we don't tell them to

    */

    public:
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

};


