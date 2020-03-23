#ifndef _RSP_TCP_API_H
#define _RSP_TCP_API_H

// RSP extended capability bit field  
#define RSP_CAPABILITY_BIAS_T (1 << 0)
#define RSP_CAPABILITY_REF_OUT (1 << 1)
#define RSP_CAPABILITY_REF_IN (1 << 2)
#define RSP_CAPABILITY_BROADCAST_NOTCH (1 << 3)
#define RSP_CAPABILITY_DAB_NOTCH (1 << 4)
#define RSP_CAPABILITY_AM_NOTCH  (1 << 5)
#define RSP_CAPABILITY_RF_NOTCH (1 << 6)
#define RSP_CAPABILITY_AGC (1 << 7)

#define RSP_CAPABILITIES_MAGIC "RSP0"
#define RSP_CAPABILITIES_VERSION (0x00000001)

/* ******************************************************************************* */

#define RSP_TCP_COMMAND_BASE 0x1f
typedef enum
{
	RSP_TCP_COMMAND_SET_ANTENNA = RSP_TCP_COMMAND_BASE + 0,
	RSP_TCP_COMMAND_SET_LNASTATE = RSP_TCP_COMMAND_BASE + 1,
	RSP_TCP_COMMAND_SET_IF_GAIN_R = RSP_TCP_COMMAND_BASE + 2,
	RSP_TCP_COMMAND_SET_AGC = RSP_TCP_COMMAND_BASE + 3,
	RSP_TCP_COMMAND_SET_AGC_SETPOINT = RSP_TCP_COMMAND_BASE + 4,
	RSP_TCP_COMMAND_SET_NOTCH = RSP_TCP_COMMAND_BASE + 5,
	RSP_TCP_COMMAND_SET_BIAST = RSP_TCP_COMMAND_BASE + 6,
	RSP_TCP_COMMAND_SET_REFOUT = RSP_TCP_COMMAND_BASE + 7,		
} rsp_tcp_commands_t;

typedef enum
{
	RSP_TCP_ANTENNA_INPUT_A = 0x0,
	RSP_TCP_ANTENNA_INPUT_B = 0x1,
	RSP_TCP_ANTENNA_INPUT_HIZ = 0x2
} rsp_tcp_antenna_t;

typedef enum
{
	RSP_TCP_AGC_DISABLE = 0x0,
	RSP_TCP_AGC_ENABLE = 0x1
} rsp_tcp_agc_state_t;

typedef enum
{
	RSP_TCP_BIAST_DISABLE = 0x0,
	RSP_TCP_BIAST_ENABLE = 0x1
} rsp_tcp_biast_state_t;

typedef enum
{
	RSP_TCP_NOTCH_AM = (1 << 0),
	RSP_TCP_NOTCH_BROADCAST = (1 << 1),
	RSP_TCP_NOTCH_DAB = (1 << 2),
	RSP_TCP_NOTCH_RF = (1 << 3)
} rsp_tcp_notches_t;

typedef enum
{
	RSP_TCP_SAMPLE_FORMAT_UINT8 = 0x1,
	RSP_TCP_SAMPLE_FORMAT_INT16 = 0x2
} rsp_tcp_sample_format_t;


/* ******************************************************************************* */

#ifdef _WIN32
#define __attribute__(x)
#pragma pack(push, 1)
#endif
typedef struct {
	// "RSP0"
	char magic[4];

	// Struct version
	unsigned int version;

	// Capabilities bitmap
	unsigned int capabilities;

	// Reserved for future use
	unsigned int __reserved__;

	// Hardware version from library
	unsigned int hardware_version;

	// see enum rsp_tcp_sample_format_t
	unsigned int sample_format;

	unsigned char antenna_input_count;
	char third_antenna_name[13];
	int third_antenna_freq_limit;

	unsigned char tuner_count;	
	unsigned char ifgr_min;
	unsigned char ifgr_max;
} __attribute__((packed)) rsp_extended_capabilities_t;
#ifdef _WIN32
#pragma pack(pop)
#endif

#endif /* RSP_TCP_API_H */
