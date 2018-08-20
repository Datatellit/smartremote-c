#ifndef _PUBLICDEFINE_H
#define _PUBLICDEFINE_H

#define RF24

// Window Watchdog
// Uncomment this line if in debug mode
#define DEBUG_NO_WWDG

// RF channel for the sensor net, 0-127
#define RF24_CHANNEL	   		100

#define NODEID_MAINDEVICE       8

#define BR_MIN_VALUE            1
#define CT_MIN_VALUE            2700
#define CT_MAX_VALUE            6500
#define CT_SCOPE                38    
#define CT_STEP                 ((CT_MAX_VALUE-CT_MIN_VALUE)/10)

#define UNIQUE_ID_LEN           8
#define NUM_DEVICES             4
#define NUM_RELAY_KEYS          4

#define ADDRESS_WIDTH                   5
#define PLOAD_WIDTH                     32

// Unique ID for STM8L151x4
#define     UNIQUE_ID_ADDRESS           (0x4926)

// Switch value for set power command
#define DEVICE_SW_OFF               0       // Turn Off
#define DEVICE_SW_ON                1       // Turn On
#define DEVICE_SW_TOGGLE            2       // Toggle

// Update operator for set brightness & CCT command
#define OPERATOR_SET                0
#define OPERATOR_ADD                1
#define OPERATOR_SUB                2
#define OPERATOR_MUL                3
#define OPERATOR_DIV                4

// Filter (special effect)
#define FILTER_SP_EF_NONE           0
#define FILTER_SP_EF_BREATH         1       // Normal breathing light
#define FILTER_SP_EF_FAST_BREATH    2       // Fast breathing light
#define FILTER_SP_EF_FLORID         3       // Randomly altering color
#define FILTER_SP_EF_FAST_FLORID    4       // Fast randomly altering color

/* Exported types ------------------------------------------------------------*/
// Common Data Type
#define UC                        uint8_t
#define US                        uint16_t
#define UL                        uint32_t
#define SHORT                     int16_t
#define LONG                      int32_t

// Node type
#define NODE_TYP_GW               'g'
#define NODE_TYP_LAMP             'l'
#define NODE_TYP_REMOTE           'r'
#define NODE_TYP_SYSTEM           's'
#define NODE_TYP_AC               'a'
#define NODE_TYP_CURTAIN          'c'
#define NODE_TYP_AIRPURE          'p'
#define NODE_TYP_SWITCH           'w'
#define NODE_TYP_THIRDPARTY       't'

// NodeID Convention
#ifdef RF24
#define NODEID_GATEWAY          0
#else
#define NODEID_GATEWAY          1
#endif
#define NODEID_MIN_LAMP         8
#define NODEID_MAX_LAMP         31
#define NODEID_MIN_COLORFULBAR  32
#define NODEID_MAX_COLORFULBAR  39
#define NODEID_MIN_SWITCH       40
#define NODEID_MAX_SWITCH       63
#define NODEID_MIN_REMOTE       64
#define NODEID_MAX_REMOTE       71
#define NODEID_MIN_AC           72
#define NODEID_MAX_AC           79
#define NODEID_MIN_FAN          80
#define NODEID_MAX_FAN          95
#define NODEID_MIN_IR           96
#define NODEID_MAX_IR           127
#define NODEID_PROJECTOR        128
#define NODEID_KEYSIMULATOR     129
#define NODEID_SUPERSENSOR      130
#define NODEID_SMARTPHONE       139
#define NODEID_MIN_AIRPURE      140
#define NODEID_MAX_AIRPURE      147
#define NODEID_MIN_CURTAIN      148
#define NODEID_MAX_CURTAIN      179
#define NODEID_MIN_GROUP        192
#define NODEID_MAX_GROUP        223
#define NODEID_RF_SCANNER       250
#define NODEID_DUMMY            255
#define BASESERVICE_ADDRESS     0xFE
#define BROADCAST_ADDRESS       0xFF

#define IS_GROUP_NODEID(nID)        (nID >= NODEID_MIN_GROUP && nID <= NODEID_MAX_GROUP)
#define IS_SPECIAL_NODEID(nID)      (nID >= NODEID_PROJECTOR && nID <= NODEID_SMARTPHONE)
#define IS_LAMP_NODEID(nID)          (nID >= NODEID_MIN_LAMP && nID <= NODEID_MAX_LAMP)
#define IS_SWITCH_NODEID(nID)        (nID >= NODEID_MIN_SWITCH && nID <= NODEID_MAX_SWITCH)
#define IS_REMOTE_NODEID(nID)       (nID >= NODEID_MIN_REMOTE && nID <= NODEID_MAX_REMOTE)
#define IS_AC_NODEID(nID)           (nID >= NODEID_MIN_AC && nID <= NODEID_MAX_AC)
#define IS_FAN_NODEID(nID)           (nID >= NODEID_MIN_FAN && nID <= NODEID_MAX_FAN)
#define IS_AIRPURE_NODEID(nID)       (nID >= NODEID_MIN_AIRPURE && nID <= NODEID_MAX_AIRPURE)
#define IS_CURTAIN_NODEID(nID)       (nID >= NODEID_MIN_CURTAIN && nID <= NODEID_MAX_CURTAIN)
#define IS_COLORFULBAR_NODEID(nID)   (nID >= NODEID_MIN_COLORFULBAR && nID <= NODEID_MIN_COLORFULBAR)

#endif /* _PUBLICDEFINE_H */