/*
* rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
* Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
* Copyright (C) 2012-2013 by Hoernchen <la@tfc-server.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#include "rsp_tcp_api.h"

#ifndef _WIN32
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <fcntl.h>
#define CTRL_C_EVENT        0
#define CTRL_BREAK_EVENT    1
#define CTRL_CLOSE_EVENT    2
#else
#include <winsock2.h>
#include "getopt/getopt.h"
#endif

#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>

#include <sdrplay_api.h>

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")

typedef int socklen_t;

#else
#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1
#endif

static SOCKET s;

static int fsc = 0;
static int rfc = 0;
static int grc = 0;
static int timeout = 500;
static pthread_t tcp_worker_thread;
static pthread_t command_thread;

static pthread_mutex_t ll_mutex;
static pthread_cond_t cond;

struct llist {
	char *data;
	size_t len;
	struct llist *next;
};

typedef struct { /* structure size must be multiple of 2 bytes */
	char magic[4];
	uint32_t tuner_type;
	uint32_t tuner_gain_count;
} dongle_info_t;

double atofs(char *s)
/* standard suffixes */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(s);
	last = s[len - 1];
	s[len - 1] = '\0';
	switch (last) {
	case 'g':
	case 'G':
		suff *= 1e3;
		/* fall-through */
	case 'm':
	case 'M':
		suff *= 1e3;
		/* fall-through */
	case 'k':
	case 'K':
		suff *= 1e3;
		suff *= atof(s);
		s[len - 1] = last;
		return suff;
	}
	s[len - 1] = last;
	return atof(s);
}

static int global_numq = 0;
static struct llist *ll_buffers = 0;
static int llbuf_num = 500;

static int overload = 0;

static volatile int do_exit = 0;
static volatile int ctrlC_exit = 0;

#define RSP_TCP_VERSION_MAJOR (1)
#define RSP_TCP_VERSION_MINOR (1)

#define MAX_DECIMATION_FACTOR (64)
#define MAX_DEVS 4
#define WORKER_TIMEOUT_SEC 3
#define DEFAULT_BW_T sdrplay_api_BW_1_536
#define DEFAULT_FREQUENCY (100000000)
#define DEFAULT_SAMPLERATE (2000000)
#define DEFAULT_AGC_SETPOINT -30
#define DEFAULT_GAIN_REDUCTION 40
#define DEFAULT_LNA_STATE 4
#define DEFAULT_AGC_STATE 1
#define RTLSDR_TUNER_R820T 5

static int bwType = sdrplay_api_BW_Undefined;
static int last_gain_idx = 0;
static int verbose = 0;
static uint8_t max_lnastate;

sdrplay_api_DeviceT devices[MAX_DEVS];
sdrplay_api_DeviceT *chosenDev;
sdrplay_api_CallbackFnsT cbFns;

sdrplay_api_DeviceParamsT *deviceParams;
sdrplay_api_RxChannelParamsT *chParams;

// *************************************
#define GAIN_STEPS (29)

const uint8_t rsp1_am_gains_lnastates[]       = {  3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_am_gains_ifgains[]         = { 59,56,53,50,47,44,41,58,55,52,49,46,43,45,42,58,55,52,49,46,43,41,38,35,32,29,26,23,20 };
const uint8_t rsp1_vhf_gains_lnastates[]      = {  3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_vhf_gains_ifgains[]        = { 59,56,53,50,47,44,41,58,55,52,49,46,43,45,42,58,55,52,49,46,43,41,38,35,32,29,26,23,20 };
const uint8_t rsp1_band3_gains_lnastates[]    = {  3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_band3_gains_ifgains[]      = { 59,56,53,50,47,44,41,58,55,52,49,46,43,45,42,58,55,52,49,46,43,41,38,35,32,29,26,23,20 };
const uint8_t rsp1_bandx_gains_lnastates[]    = {  3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_bandx_gains_ifgains[]      = { 59,56,53,50,47,44,41,58,55,52,49,46,43,45,42,58,55,52,49,46,43,41,38,35,32,29,26,23,20 };
const uint8_t rsp1_band45_gains_lnastates[]   = {  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0 };
const uint8_t rsp1_band45_gains_ifgains[]     = { 59,57,54,52,50,47,45,43,40,38,36,33,31,29,27,24,22,27,24,22,32,29,27,25,22,27,25,22,20 };
const uint8_t rsp1_lband_gains_lnastates[]    = {  3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_lband_gains_ifgains[]      = { 59,57,55,52,50,48,46,43,41,44,42,53,51,49,47,44,42,45,43,40,38,36,34,31,29,27,25,22,20 };

const uint8_t rsp1a_am_gains_lnastates[]      = {  6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 3, 3, 3, 3, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_am_gains_ifgains[]        = { 59,55,52,48,45,41,57,53,49,46,42,44,40,56,52,48,45,41,44,40,43,45,41,38,34,31,27,24,20 };
const uint8_t rsp1a_vhf_gains_lnastates[]     = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_vhf_gains_ifgains[]       = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rsp1a_band3_gains_lnastates[]   = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_band3_gains_ifgains[]     = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rsp1a_bandx_gains_lnastates[]   = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_bandx_gains_ifgains[]     = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rsp1a_band45_gains_lnastates[]  = {  9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 6, 6, 5, 5, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_band45_gains_ifgains[]    = { 59,55,52,48,44,41,56,52,49,45,41,44,46,42,45,41,44,40,44,40,42,46,42,38,35,31,27,24,20 };
const uint8_t rsp1a_lband_gains_lnastates[]   = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 3, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_lband_gains_ifgains[]     = { 59,55,52,48,45,41,56,53,49,46,42,43,46,42,44,41,43,48,44,40,43,45,42,38,34,31,27,24,20 };

const uint8_t rsp2_am_gains_lnastates[]       = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_am_gains_ifgains[]         = { 59,55,52,48,44,41,56,52,49,45,41,44,45,41,48,44,40,45,42,43,49,46,42,38,35,31,27,24,20 };
const uint8_t rsp2_vhf_gains_lnastates[]      = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_vhf_gains_ifgains[]        = { 59,55,52,48,44,41,56,52,49,45,41,44,45,41,48,44,40,45,42,43,49,46,42,38,35,31,27,24,20 };
const uint8_t rsp2_band3_gains_lnastates[]    = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_band3_gains_ifgains[]      = { 59,55,52,48,44,41,56,52,49,45,41,44,45,41,48,44,40,45,42,43,49,46,42,38,35,31,27,24,20 };
const uint8_t rsp2_bandx_gains_lnastates[]    = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_bandx_gains_ifgains[]      = { 59,55,52,48,44,41,56,52,49,45,41,44,45,41,48,44,40,45,42,43,49,46,42,38,35,31,27,24,20 };
const uint8_t rsp2_band45_gains_lnastates[]   = {  5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_band45_gains_ifgains[]     = { 59,56,53,50,48,45,42,58,55,52,49,47,44,41,43,40,44,41,42,46,43,40,37,34,31,29,26,23,20 };
const uint8_t rsp2_lband_gains_lnastates[]    = {  4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_lband_gains_ifgains[]      = { 59,56,54,51,48,45,43,40,56,54,51,48,45,43,40,43,41,44,41,44,42,39,36,34,31,28,25,23,20 };
const uint8_t rsp2_hiz_gains_lnastates[]      = {  4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_hiz_gains_ifgains[]        = { 59,56,54,51,48,45,43,40,56,54,51,48,45,43,40,43,41,44,41,44,42,39,36,34,31,28,25,23,20 };

const uint8_t rspduo_am_gains_lnastates[]     = {  6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 3, 3, 3, 3, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_am_gains_ifgains[]       = { 59,55,52,48,45,41,57,53,49,46,42,44,40,56,52,48,45,41,44,40,43,45,41,38,34,31,27,24,20 };
const uint8_t rspduo_vhf_gains_lnastates[]    = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_vhf_gains_ifgains[]      = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rspduo_band3_gains_lnastates[]  = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_band3_gains_ifgains[]    = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rspduo_bandx_gains_lnastates[]  = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_bandx_gains_ifgains[]    = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rspduo_band45_gains_lnastates[] = {  9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 6, 6, 5, 5, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_band45_gains_ifgains[]   = { 59,55,52,48,44,41,56,52,49,45,41,44,46,42,45,41,44,40,44,40,42,46,42,38,35,31,27,24,20 };
const uint8_t rspduo_lband_gains_lnastates[]  = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 3, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_lband_gains_ifgains[]    = { 59,55,52,48,45,41,56,53,49,46,42,43,46,42,44,41,43,48,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rspduo_hiz_gains_lnastates[]    = {  4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_hiz_gains_ifgains[]      = { 59,56,54,51,48,45,43,40,56,54,51,48,45,43,40,43,41,44,41,44,42,39,36,34,31,28,25,23,20 };

const uint8_t rspdx_am_gains_lnastates[]      = { 18,18,18,18,18,18,17,16,14,13,12,11,10, 9, 7, 6, 5, 5, 5, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_am_gains_ifgains[]        = { 59,55,52,48,45,41,41,40,43,42,42,41,41,40,42,42,47,44,40,43,42,42,41,38,34,31,27,24,20 };
const uint8_t rspdx_vhf_gains_lnastates[]     = { 26,26,26,26,26,25,23,22,20,19,17,16,14,13,11,10, 8, 7, 5, 5, 5, 3, 2, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_vhf_gains_ifgains[]       = { 59,55,50,46,41,40,42,40,42,40,42,41,42,41,43,41,43,41,49,45,40,42,40,42,38,33,29,24,20 };
const uint8_t rspdx_band3_gains_lnastates[]   = { 26,26,26,26,26,25,23,22,20,19,17,16,14,13,11,10, 8, 7, 5, 5, 5, 3, 2, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_band3_gains_ifgains[]     = { 59,55,50,46,41,40,42,40,42,40,42,41,42,41,43,41,43,41,49,45,40,42,40,42,38,33,29,24,20 };
const uint8_t rspdx_bandx_gains_lnastates[]   = { 27,27,27,27,27,26,24,23,21,20,18,17,15,14,12,11, 9, 8, 6, 6, 5, 3, 2, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_bandx_gains_ifgains[]     = { 59,55,50,46,41,40,42,40,42,40,42,41,42,41,43,41,43,41,46,42,40,42,40,42,38,33,29,24,20 };
const uint8_t rspdx_band45_gains_lnastates[]  = { 20,20,20,20,20,20,18,17,16,14,13,12,11, 9, 8, 7, 7, 5, 4, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_band45_gains_ifgains[]    = { 59,55,51,48,44,40,42,42,41,43,42,41,41,43,42,44,40,43,42,41,40,46,43,39,35,31,28,24,20 };
const uint8_t rspdx_lband_gains_lnastates[]   = { 18,18,18,18,18,18,16,15,14,13,11,10, 9, 8, 7, 6, 6, 6, 5, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_lband_gains_ifgains[]     = { 59,55,52,48,44,40,43,42,41,41,43,42,41,41,40,48,45,41,40,42,42,41,42,39,35,31,27,24,20 };
const uint8_t rspdx_hiz_gains_lnastates[]     = { 18,18,18,18,18,18,17,16,14,13,12,11,10, 9, 7, 6, 5, 5, 5, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_hiz_gains_ifgains[]       = { 59,55,52,48,45,41,41,40,43,42,42,41,41,40,42,42,47,44,40,43,42,42,41,38,34,31,27,24,20 };

typedef enum {
	RSP_MODEL_UNKNOWN = 0,
	RSP_MODEL_RSP1 = 1,
	RSP_MODEL_RSP1A = 2,
	RSP_MODEL_RSP2 = 3,
	RSP_MODEL_RSPDUO = 4,
	RSP_MODEL_RSPDX = 5
} rsp_model_t;

typedef enum {
	BAND_UNKNOWN = 0,
	BAND_AM = 1,
	BAND_VHF = 2,
	BAND_3 = 3,
	BAND_X = 4,
	BAND_45 = 5,
	BAND_L = 6,
	BAND_AM_HIZ = 7
} rsp_band_t;

typedef struct {
	rsp_model_t model;
	char *name;
	uint8_t antenna_input_count;
	char third_antenna_name[13];
	int third_antenna_freq_limit;
	uint8_t tuner_count;
	uint32_t capabilities;

	uint8_t min_ifgr;
	uint8_t max_ifgr;

	const uint8_t* am_lna_states;
	const uint8_t* am_if_gains;
	const uint8_t* vhf_lna_states;
	const uint8_t* vhf_if_gains;
	const uint8_t* band3_lna_states;
	const uint8_t* band3_if_gains;
	const uint8_t* bandx_lna_states;
	const uint8_t* bandx_if_gains;
	const uint8_t* band45_lna_states;
	const uint8_t* band45_if_gains;
	const uint8_t* lband_lna_states;
	const uint8_t* lband_if_gains;
	const uint8_t* hiz_lna_states;
	const uint8_t* hiz_if_gains;

} rsp_capabilities_t;

static rsp_capabilities_t device_caps[] = {
	{
		.model = RSP_MODEL_RSP1,
		.name = "RSP1",
		.antenna_input_count = 1,
		.third_antenna_name = "",
		.third_antenna_freq_limit = 0,
		.tuner_count = 1,
		.capabilities = RSP_CAPABILITY_AGC,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rsp1_am_gains_lnastates,
		.am_if_gains = rsp1_am_gains_ifgains,
		.vhf_lna_states = rsp1_vhf_gains_lnastates,
		.vhf_if_gains = rsp1_vhf_gains_ifgains,
		.band3_lna_states = rsp1_band3_gains_lnastates,
		.band3_if_gains = rsp1_band3_gains_ifgains,
		.bandx_lna_states = rsp1_bandx_gains_lnastates,
		.bandx_if_gains = rsp1_bandx_gains_ifgains,
		.band45_lna_states = rsp1_band45_gains_lnastates,
		.band45_if_gains = rsp1_band45_gains_ifgains,
		.lband_lna_states = rsp1_lband_gains_lnastates,
		.lband_if_gains = rsp1_lband_gains_ifgains,
		.hiz_lna_states = NULL,
		.hiz_if_gains = NULL,
	},
	{
		.model = RSP_MODEL_RSP1A,
		.name = "RSP1A",
		.antenna_input_count = 1,
		.third_antenna_name = "",
		.third_antenna_freq_limit = 0,
		.tuner_count = 1,
		.capabilities = RSP_CAPABILITY_AGC |
						RSP_CAPABILITY_BIAS_T |
						RSP_CAPABILITY_DAB_NOTCH |
						RSP_CAPABILITY_BROADCAST_NOTCH,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rsp1a_am_gains_lnastates,
		.am_if_gains = rsp1a_am_gains_ifgains,
		.vhf_lna_states = rsp1a_vhf_gains_lnastates,
		.vhf_if_gains = rsp1a_vhf_gains_ifgains,
		.band3_lna_states = rsp1a_band3_gains_lnastates,
		.band3_if_gains = rsp1a_band3_gains_ifgains,
		.bandx_lna_states = rsp1a_bandx_gains_lnastates,
		.bandx_if_gains = rsp1a_bandx_gains_ifgains,
		.band45_lna_states = rsp1a_band45_gains_lnastates,
		.band45_if_gains = rsp1a_band45_gains_ifgains,
		.lband_lna_states = rsp1a_lband_gains_lnastates,
		.lband_if_gains = rsp1a_lband_gains_ifgains,
		.hiz_lna_states = NULL,
		.hiz_if_gains = NULL,
	},
	{
		.model = RSP_MODEL_RSP2,
		.name = "RSP2",
		.antenna_input_count = 3,
		.third_antenna_name = "Antenna Hi-Z",
		.third_antenna_freq_limit = 30000000,
		.tuner_count = 1,
		.capabilities = RSP_CAPABILITY_AGC |
						RSP_CAPABILITY_BIAS_T |
						RSP_CAPABILITY_RF_NOTCH |
						RSP_CAPABILITY_REF_IN |
						RSP_CAPABILITY_REF_OUT,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rsp2_am_gains_lnastates,
		.am_if_gains = rsp2_am_gains_ifgains,
		.vhf_lna_states = rsp2_vhf_gains_lnastates,
		.vhf_if_gains = rsp2_vhf_gains_ifgains,
		.band3_lna_states = rsp2_band3_gains_lnastates,
		.band3_if_gains = rsp2_band3_gains_ifgains,
		.bandx_lna_states = rsp2_bandx_gains_lnastates,
		.bandx_if_gains = rsp2_bandx_gains_ifgains,
		.band45_lna_states = rsp2_band45_gains_lnastates,
		.band45_if_gains = rsp2_band45_gains_ifgains,
		.lband_lna_states = rsp2_lband_gains_lnastates,
		.lband_if_gains = rsp2_lband_gains_ifgains,
		.hiz_lna_states = rsp2_hiz_gains_lnastates,
		.hiz_if_gains = rsp2_hiz_gains_ifgains,
	},
	{
		.model = RSP_MODEL_RSPDUO,
		.name = "RSPduo",
		.antenna_input_count = 3,
		.third_antenna_name = "Antenna Hi-Z",
		.third_antenna_freq_limit = 30000000,
		.tuner_count = 2,
		.capabilities = RSP_CAPABILITY_AGC |
						RSP_CAPABILITY_BIAS_T |
						RSP_CAPABILITY_AM_NOTCH |
						RSP_CAPABILITY_DAB_NOTCH |
						RSP_CAPABILITY_BROADCAST_NOTCH |
						RSP_CAPABILITY_REF_IN |
						RSP_CAPABILITY_REF_OUT,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rspduo_am_gains_lnastates,
		.am_if_gains = rspduo_am_gains_ifgains,
		.vhf_lna_states = rspduo_vhf_gains_lnastates,
		.vhf_if_gains = rspduo_vhf_gains_ifgains,
		.band3_lna_states = rspduo_band3_gains_lnastates,
		.band3_if_gains = rspduo_band3_gains_ifgains,
		.bandx_lna_states = rspduo_bandx_gains_lnastates,
		.bandx_if_gains = rspduo_bandx_gains_ifgains,
		.band45_lna_states = rspduo_band45_gains_lnastates,
		.band45_if_gains = rspduo_band45_gains_ifgains,
		.lband_lna_states = rspduo_lband_gains_lnastates,
		.lband_if_gains = rspduo_lband_gains_ifgains,
		.hiz_lna_states = rspduo_hiz_gains_lnastates,
		.hiz_if_gains = rspduo_hiz_gains_ifgains,
	},
	{
		.model = RSP_MODEL_RSPDX,
		.name = "RSPdx",
		.antenna_input_count = 3,
		.third_antenna_name = "Antenna C",
		.third_antenna_freq_limit = 200000000,
		.tuner_count = 1,
		.capabilities = RSP_CAPABILITY_AGC |
						RSP_CAPABILITY_BIAS_T |
						RSP_CAPABILITY_DAB_NOTCH |
						RSP_CAPABILITY_BROADCAST_NOTCH |
						RSP_CAPABILITY_REF_IN,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rspdx_am_gains_lnastates,
		.am_if_gains = rspdx_am_gains_ifgains,
		.vhf_lna_states = rspdx_vhf_gains_lnastates,
		.vhf_if_gains = rspdx_vhf_gains_ifgains,
		.band3_lna_states = rspdx_band3_gains_lnastates,
		.band3_if_gains = rspdx_band3_gains_ifgains,
		.bandx_lna_states = rspdx_bandx_gains_lnastates,
		.bandx_if_gains = rspdx_bandx_gains_ifgains,
		.band45_lna_states = rspdx_band45_gains_lnastates,
		.band45_if_gains = rspdx_band45_gains_ifgains,
		.lband_lna_states = rspdx_lband_gains_lnastates,
		.lband_if_gains = rspdx_lband_gains_ifgains,
		.hiz_lna_states = rspdx_hiz_gains_lnastates,
		.hiz_if_gains = rspdx_hiz_gains_ifgains,
	},
};

static int extended_mode = 0;
static int hardware_version = 0;
static rsp_capabilities_t *hardware_caps = NULL;
static rsp_model_t hardware_model = RSP_MODEL_UNKNOWN;
static rsp_tcp_sample_format_t sample_format = RSP_TCP_SAMPLE_FORMAT_UINT8;
static rsp_band_t current_band = BAND_UNKNOWN;
static int current_antenna_input = 0;
static unsigned int current_frequency;
static int lna_state = DEFAULT_LNA_STATE;
static int agc_state = DEFAULT_AGC_STATE;
static int agc_set_point = DEFAULT_AGC_SETPOINT;
static int gain_reduction = DEFAULT_GAIN_REDUCTION;
static int sample_shift = 2;

// *************************************

#ifdef _WIN32
int gettimeofday(struct timeval *tv, void* ignored)
{
	FILETIME ft;
	unsigned __int64 tmp = 0;
	if (NULL != tv) {
		GetSystemTimeAsFileTime(&ft);
		tmp |= ft.dwHighDateTime;
		tmp <<= 32;
		tmp |= ft.dwLowDateTime;
		tmp /= 10;
#ifdef _MSC_VER
		tmp -= 11644473600000000Ui64;
#else
		tmp -= 11644473600000000ULL;
#endif
		tv->tv_sec = (long)(tmp / 1000000UL);
		tv->tv_usec = (long)(tmp % 1000000UL);
	}
	return 0;
}

BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "CTRL-C caught, exiting!\n");
		do_exit = 1;
		ctrlC_exit = 1;
		return TRUE;
	}
	else if (CTRL_CLOSE_EVENT == signum) {
		fprintf(stderr, "SIGQUIT caught, exiting!\n");
		do_exit = 1;
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal (%d) caught, ask for exit!\n", signum);
	do_exit = 1;
}
#endif

void event_callback(sdrplay_api_EventT eventId, sdrplay_api_TunerSelectT tunerS, sdrplay_api_EventParamsT *params, void* cbContext)
{
	switch (eventId)
	{
	case sdrplay_api_GainChange:
		if (params->gainParams.gRdB < 200)
		{
			printf("gRdB: %u, lnaGRdB: %u, LNAstate: %u, Gain: %0.1f\n", params->gainParams.gRdB, params->gainParams.lnaGRdB, chParams->tunerParams.gain.LNAstate, params->gainParams.currGain);
		}
		break;
	case sdrplay_api_PowerOverloadChange:
		sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Ctrl_OverloadMsgAck, sdrplay_api_Update_Ext1_None);

		if (params->powerOverloadParams.powerOverloadChangeType == sdrplay_api_Overload_Detected)
		{
			printf("adc overload detected\n");
			overload = 1;
		}
		else if (params->powerOverloadParams.powerOverloadChangeType == sdrplay_api_Overload_Corrected)
		{
			printf("adc overload corrected\n");
			overload = 0;
		}
		break;
	case sdrplay_api_DeviceRemoved:
		printf("RSP removed\n");
		sdrplay_api_Uninit(chosenDev->dev);
		sdrplay_api_ReleaseDevice(chosenDev);
		sdrplay_api_Close();
		exit(1);
	case sdrplay_api_RspDuoModeChange:
		printf("RSPduo mode changed\n");
		break;
	}
}

void rxa_callback(short* xi, short* xq, sdrplay_api_StreamCbParamsT *params, unsigned int numSamples, unsigned int reset, void* cbContext)
{
	if(params->fsChanged != 0)
	{
		fsc = params->fsChanged;
		printf("params->fsChanged = %d\n", params->fsChanged);
	}
	if(params->rfChanged != 0)
	{
		rfc = params->rfChanged;
		printf("params->rfChanged = %d\n", params->rfChanged);
	}
	if(params->grChanged != 0)
	{
		grc = params->grChanged;
		printf("params->grChanged = %d\n", params->grChanged);
	}

	if (!do_exit) {
		unsigned int i;
		struct llist *rpt = (struct llist*)malloc(sizeof(struct llist));

		if (sample_format == RSP_TCP_SAMPLE_FORMAT_UINT8)
		{
			rpt->data = (char*)malloc(2 * numSamples);

			// assemble the data
			char *data;
			data = rpt->data;
			for (i = 0; i < numSamples; i++, xi++, xq++)
			{
				*(data++) = (unsigned char)(((*xi << sample_shift) >> 8) + 128);
				*(data++) = (unsigned char)(((*xq << sample_shift) >> 8) + 128);
			}

			rpt->len = 2 * numSamples;
		}
		else if (sample_format == RSP_TCP_SAMPLE_FORMAT_INT16)
		{
			rpt->data = (char*)malloc(4 * numSamples);

			short *data;
			data = (short*)rpt->data;
			for (i = 0; i < numSamples; i++, xi++, xq++)
			{
				*(data++) = *xi;
				*(data++) = *xq;
			}

			rpt->len = 4 * numSamples;
		}

		rpt->next = NULL;

		pthread_mutex_lock(&ll_mutex);

		if (ll_buffers == NULL) {
			ll_buffers = rpt;
		}
		else {
			struct llist *cur = ll_buffers;
			int num_queued = 0;

			while (cur->next != NULL) {
				cur = cur->next;
				num_queued++;
			}

			if (llbuf_num && llbuf_num == num_queued - 2) {
				struct llist *curelem;

				free(ll_buffers->data);
				curelem = ll_buffers->next;
				free(ll_buffers);
				ll_buffers = curelem;
			}

			cur->next = rpt;
			//if (verbose) {
			//	if (num_queued > global_numq)
			//		printf("ll+, now %d\n", num_queued);
			//	else if (num_queued < global_numq)
			//		printf("ll-, now %d\n", num_queued);
			//}
			global_numq = num_queued;
		}
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&ll_mutex);
	}
}

void rxb_callback(short* xi, short* xq, sdrplay_api_StreamCbParamsT *params, unsigned int numSamples, unsigned int reset, void* cbContext)
{
	if (!do_exit) {
		unsigned int i;
		struct llist *rpt = (struct llist*)malloc(sizeof(struct llist));

		if (sample_format == RSP_TCP_SAMPLE_FORMAT_UINT8) {
			rpt->data = (char*)malloc(2 * numSamples);

			// assemble the data
			char *data;
			data = rpt->data;
			for (i = 0; i < numSamples; i++, xi++, xq++) {
				// Calculation to 8 bit was wrong, this one is correct and better. Bas ON5HB.
				*(data++) = (unsigned char)(((*xi << 1) + 16384) / 128) + 0.5;
				*(data++) = (unsigned char)(((*xq << 1) + 16384) / 128) + 0.5;
				
			}

			rpt->len = 2 * numSamples;
		}
		else
			if (sample_format == RSP_TCP_SAMPLE_FORMAT_INT16) {
				rpt->data = (char*)malloc(4 * numSamples);

				short *data;
				data = (short*)rpt->data;
				for (i = 0; i < numSamples; i++, xi++, xq++) {
					*(data++) = *xi;
					*(data++) = *xq;
				}

				rpt->len = 4 * numSamples;
			}

		rpt->next = NULL;

		pthread_mutex_lock(&ll_mutex);

		if (ll_buffers == NULL) {
			ll_buffers = rpt;
		}
		else {
			struct llist *cur = ll_buffers;
			int num_queued = 0;

			while (cur->next != NULL) {
				cur = cur->next;
				num_queued++;
			}

			if (llbuf_num && llbuf_num == num_queued - 2) {
				struct llist *curelem;

				free(ll_buffers->data);
				curelem = ll_buffers->next;
				free(ll_buffers);
				ll_buffers = curelem;
			}

			cur->next = rpt;
			if (verbose) {
				if (num_queued > global_numq)
					printf("ll+, now %d\n", num_queued);
				else if (num_queued < global_numq)
					printf("ll-, now %d\n", num_queued);
			}
			global_numq = num_queued;
		}
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&ll_mutex);
	}
}

static void *tcp_worker(void *arg)
{
	struct llist *curelem, *prev;
	int bytesleft, bytessent, index;
	struct timeval tv = { 1,0 };
	struct timespec ts;
	struct timeval tp;
	fd_set writefds;
	int r = 0;

	while (1) {
		if (do_exit) {
			pthread_exit(0);
		}

		pthread_mutex_lock(&ll_mutex);
		gettimeofday(&tp, NULL);
		ts.tv_sec = tp.tv_sec + WORKER_TIMEOUT_SEC;
		ts.tv_nsec = tp.tv_usec * 1000;
		r = pthread_cond_timedwait(&cond, &ll_mutex, &ts);
		if (r == ETIMEDOUT) {
			pthread_mutex_unlock(&ll_mutex);
			printf("worker cond timeout\n");
			sighandler(CTRL_CLOSE_EVENT);
			pthread_exit(NULL);
		}

		curelem = ll_buffers;
		ll_buffers = 0;
		pthread_mutex_unlock(&ll_mutex);

		while (curelem != 0) {
			bytesleft = curelem->len;
			index = 0;
			bytessent = 0;
			while (bytesleft > 0) {
				FD_ZERO(&writefds);
				FD_SET(s, &writefds);
				tv.tv_sec = 1;
				tv.tv_usec = 0;
				r = select(s + 1, NULL, &writefds, NULL, &tv);
				if (r) {
					bytessent = send(s, &curelem->data[index], bytesleft, 0);
					bytesleft -= bytessent;
					index += bytessent;
				}
				if (bytessent == SOCKET_ERROR || do_exit) {
#ifdef _WIN32
					printf("worker socket bye (%d), do_exit:%d\n", WSAGetLastError(), do_exit);
#else
					printf("worker socket bye, do_exit:%d\n", do_exit);
#endif
					sighandler(CTRL_CLOSE_EVENT);
					pthread_exit(NULL);
				}
			}
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}
	}
}

static rsp_model_t hardware_ver_to_model(int hw_version)
{
	// Convert hardware version from library to internal enumerated type
	switch (hw_version)
	{
	case 1:
		return RSP_MODEL_RSP1;
	case 255:
		return RSP_MODEL_RSP1A;
	case 2:
		return RSP_MODEL_RSP2;
	case 3:
		return RSP_MODEL_RSPDUO;
	case 4:
		return RSP_MODEL_RSPDX;
	default:
		return RSP_MODEL_UNKNOWN;
	}
}

static rsp_band_t frequency_to_band(unsigned int f)
{
	if (f >= 0 && f < 60000000) {
		return current_antenna_input == 2 ? BAND_AM_HIZ : BAND_AM;
	}
	else
	if (f >= 60000000 && f < 120000000)
	{
		return BAND_VHF;
	}
	else
	if (f >= 120000000 && f < 250000000)
	{
		return BAND_3;
	}
	else
	if (f >= 250000000 && f < 420000000)
	{
		return BAND_X;
	}
	else
	if (f >= 420000000 && f < 1000000000)
	{
		return BAND_45;
	}
	else
	if (f >= 1000000000 && f <= 2000000000)
	{
		return BAND_L;
	}
	else
	{
		return BAND_UNKNOWN;
	}
}

static const char* model_to_string(rsp_model_t model)
{
	// Convert enumerated model to string for printing
	switch (model)
	{
	case RSP_MODEL_RSP1:
		return "RSP1";
	case RSP_MODEL_RSP1A:
		return "RSP1A";
	case RSP_MODEL_RSP2:
		return "RSP2";
	case RSP_MODEL_RSPDUO:
		return "RSPduo";
	case RSP_MODEL_RSPDX:
		return "RSPdx";
	default:
		return "Unknown";
	}
}

static rsp_capabilities_t *model_to_capabilities(rsp_model_t model)
{
	unsigned int i;
	for (i = 0; i < sizeof(device_caps) / sizeof(device_caps[0]); i++)
	{
		if (device_caps[i].model == model) {
			return &device_caps[i];
		}
	}

	return NULL;
}

static int gain_index_to_gain(unsigned int index, uint8_t *if_gr_out, uint8_t *lna_state_out)
{
	const uint8_t *if_gains;
	const uint8_t *lnastates;

	if_gains = NULL;
	lnastates = NULL;

	switch (current_band)
	{
	case BAND_AM:
		if_gains = hardware_caps->am_if_gains;
		lnastates = hardware_caps->am_lna_states;
		break;

	case BAND_VHF:
		if_gains = hardware_caps->vhf_if_gains;
		lnastates = hardware_caps->vhf_lna_states;
		break;

	case BAND_3:
		if_gains = hardware_caps->band3_if_gains;
		lnastates = hardware_caps->band3_lna_states;
		break;

	case BAND_X:
		if_gains = hardware_caps->bandx_if_gains;
		lnastates = hardware_caps->bandx_lna_states;
		break;

	case BAND_45:
		if_gains = hardware_caps->band45_if_gains;
		lnastates = hardware_caps->band45_lna_states;
		break;

	case BAND_L:
		if_gains = hardware_caps->lband_if_gains;
		lnastates = hardware_caps->lband_lna_states;
		break;

	case BAND_AM_HIZ:
		if_gains = hardware_caps->hiz_if_gains;
		lnastates = hardware_caps->hiz_lna_states;
		break;

	default:
		break;
	}

	if (if_gains && lnastates) {

		max_lnastate = lnastates[0];
		uint8_t if_gr = if_gains[index];

		*if_gr_out = if_gr;
		*lna_state_out = lnastates[index];
		return 0;
	}

	return 1;
}

static int apply_agc_settings()
{
	int r;
	sdrplay_api_AgcControlT agc = agc_state ? sdrplay_api_AGC_CTRL_EN : sdrplay_api_AGC_DISABLE;

	chParams->ctrlParams.agc.enable = agc;
	chParams->ctrlParams.agc.setPoint_dBfs = agc_set_point;
	chParams->ctrlParams.agc.attack_ms = 500;
	chParams->ctrlParams.agc.decay_ms = 500;
	chParams->ctrlParams.agc.decay_delay_ms = 200;
	chParams->ctrlParams.agc.decay_threshold_dB = 5;

	printf("apply agc settings - enable:%d\n", agc);

	r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Ctrl_Agc, sdrplay_api_Update_Ext1_None);
	if (r != sdrplay_api_Success) {
		printf("agc control error (%d)\n", r);
	}

	return r;
}

static int apply_gain_settings()
{
	int r;

	chParams->tunerParams.gain.gRdB = gain_reduction;
	chParams->tunerParams.gain.LNAstate = lna_state;

	grc = 0;
	int count = 0;
	sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Tuner_Gr, sdrplay_api_Update_Ext1_None);

	while(count < timeout)
	{
		if(grc != 0)
		{
			printf("GR updated in %d ms\n", count);
			r = 0;
			break;
		}
#ifdef _WIN32
		Sleep(1);
#else
		usleep(1000);
#endif
		count++;
	}

	if(count >= timeout)
	{
		printf("GR failed to update in %.1f seconds\n", (timeout / 1000.0));
		r = 1;
	}

	return r;
}

static int set_if_gain_reduction(int gr)
{
	if (gr != gain_reduction && !agc_state) {
		gain_reduction = gr;
		apply_gain_settings();
	}

	return 0;
}

static int set_lna(unsigned int lnastate)
{
	if (lnastate != lna_state) {
		lna_state = lnastate;
		apply_gain_settings();
	}

	return 0;
}

static int set_agc(unsigned int enable)
{
	if (enable != agc_state) {
		agc_state = enable ? 1 : 0;
		apply_agc_settings();
	}

	return 0;
}

static int set_agc_setpoint(int set_point)
{
	if (set_point != agc_set_point) {
		agc_set_point = set_point;
		apply_agc_settings();
	}

	return 0;
}

static int set_bias_t(unsigned int enable)
{
	int r;
	switch (hardware_model)
	{
	case RSP_MODEL_RSP2:
		chParams->rsp2TunerParams.biasTEnable = enable;

		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Rsp2_BiasTControl, sdrplay_api_Update_Ext1_None);
		if (r != sdrplay_api_Success) {
			printf("bias-t control error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSP1A:
		chParams->rsp1aTunerParams.biasTEnable = enable;

		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Rsp1a_BiasTControl, sdrplay_api_Update_Ext1_None);
		if (r != sdrplay_api_Success) {
			printf("bias-t control error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSPDUO:
		chParams->rspDuoTunerParams.biasTEnable = enable;

		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_RspDuo_BiasTControl, sdrplay_api_Update_Ext1_None);
		if (r != sdrplay_api_Success) {
			printf("bias-t control error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSPDX:
		deviceParams->devParams->rspDxParams.biasTEnable = enable;

		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_None, sdrplay_api_Update_RspDx_BiasTControl);
		if (r != sdrplay_api_Success) {
			printf("bias-t control error (%d)\n", r);
		}
		break;

	default:
		if (verbose) {
			printf("bias-t not supported\n");
		}
		break;
	}

	return 0;
}

static int set_refclock_output(unsigned int enable)
{
	int r;
	switch (hardware_model)
	{
	case RSP_MODEL_RSP2:
		deviceParams->devParams->rsp2Params.extRefOutputEn = enable;

		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Rsp2_ExtRefControl, sdrplay_api_Update_Ext1_None);
		if (r != sdrplay_api_Success) {
			printf("external reference control error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSPDUO:
		deviceParams->devParams->rspDuoParams.extRefOutputEn = enable;

		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_RspDuo_ExtRefControl, sdrplay_api_Update_Ext1_None);
		if (r != sdrplay_api_Success) {
			printf("external reference control error (%d)\n", r);
		}
		break;

	default:
		if (verbose) {
			printf("reference clock output not supported\n");
		}
		break;
	}

	return 0;
}

static int set_antenna_input(unsigned int antenna)
{
	if (hardware_model == RSP_MODEL_RSP2 || hardware_model == RSP_MODEL_RSPDUO || hardware_model == RSP_MODEL_RSPDX)
	{
		int r;
		sdrplay_api_ReasonForUpdateT reason1 = sdrplay_api_Update_None;
		sdrplay_api_ReasonForUpdateExtension1T reason2 = sdrplay_api_Update_Ext1_None;
		switch (antenna)
		{
		case RSP_TCP_ANTENNA_INPUT_A:

			if (hardware_model == RSP_MODEL_RSP2)
			{
				chParams->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_2;
				chParams->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_A;
				reason1 = sdrplay_api_Update_Rsp2_AmPortSelect;
				reason1 |= sdrplay_api_Update_Rsp2_AntennaControl;
			}
			else if (hardware_model == RSP_MODEL_RSPDUO)
			{
				if (chosenDev->tuner != sdrplay_api_Tuner_A)
				{
					r = sdrplay_api_SwapRspDuoActiveTuner(chosenDev->dev, &chosenDev->tuner, sdrplay_api_RspDuo_AMPORT_2);
					if (r != sdrplay_api_Success) {
						printf("set tuner error (%d)\n", r);
					}
					chParams = deviceParams->rxChannelA;
				}
				else
				{
					chParams->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_2;
					reason1 = sdrplay_api_Update_RspDuo_AmPortSelect;
				}
			}
			else // RSP_MODEL_RSPDX
			{
				deviceParams->devParams->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_A;
				reason2 = sdrplay_api_Update_RspDx_AntennaControl;
			}

			break;

		case RSP_TCP_ANTENNA_INPUT_B:

			if (hardware_model == RSP_MODEL_RSP2)
			{
				deviceParams->rxChannelA->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_2;
				deviceParams->rxChannelA->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_B;
				reason1 = sdrplay_api_Update_Rsp2_AmPortSelect;
				reason1 |= sdrplay_api_Update_Rsp2_AntennaControl;
			}
			else if (hardware_model == RSP_MODEL_RSPDUO)
			{
				if (chosenDev->tuner != sdrplay_api_Tuner_B)
				{
					r = sdrplay_api_SwapRspDuoActiveTuner(chosenDev->dev, &chosenDev->tuner, sdrplay_api_RspDuo_AMPORT_2);
					if (r != sdrplay_api_Success) {
						printf("set tuner error (%d)\n", r);
					}
					chParams = deviceParams->rxChannelB;
				}
			}
			else // RSP_MODEL_RSPDX
			{
				deviceParams->devParams->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_B;
				reason2 = sdrplay_api_Update_RspDx_AntennaControl;
			}

			break;

		case RSP_TCP_ANTENNA_INPUT_HIZ:

			if (hardware_model == RSP_MODEL_RSPDUO)
			{
				if (current_frequency < 30000000)
				{
					if (chosenDev->tuner != sdrplay_api_Tuner_A)
					{
						r = sdrplay_api_SwapRspDuoActiveTuner(chosenDev->dev, &chosenDev->tuner, sdrplay_api_RspDuo_AMPORT_1);
						if (r != sdrplay_api_Success) {
							printf("set tuner error (%d)\n", r);
						}
						chParams = deviceParams->rxChannelA;
					}
					else
					{
						chParams->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_1;
						reason1 = sdrplay_api_Update_RspDuo_AmPortSelect;
					}
				}
			}
			else if (hardware_model == RSP_MODEL_RSP2)
			{
				if (current_frequency < 30000000)
				{
					chParams->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_1;
					reason1 = sdrplay_api_Update_Rsp2_AmPortSelect;
				}
			}
			else // RSP_MODEL_RSPDX
			{
				if (current_frequency < 200000000)
				{
					deviceParams->devParams->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_C;
					reason2 = sdrplay_api_Update_RspDx_AntennaControl;
				}
			}

			break;
		}

		current_antenna_input = antenna;

		rsp_band_t new_band = frequency_to_band(current_frequency);
		if (new_band != current_band) {
			uint8_t if_gr, lnastate;

			current_band = new_band;

			gain_index_to_gain(last_gain_idx, &if_gr, &lnastate);			
			gain_reduction = if_gr;
			lna_state = lnastate;
		}

		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, reason1, reason2);
		if (r != sdrplay_api_Success) {
			printf("set tuner error (%d)\n", r);
		}
	}
	else
	{
		if (verbose) {
			printf("antenna input not supported\n");
		}
	}

	return 0;
}


static int set_notch_filters(unsigned int notch)
{
	int r;
	unsigned int rf_notch = (notch & RSP_TCP_NOTCH_RF) ? 1 : 0;
	unsigned int am_notch = (notch & RSP_TCP_NOTCH_AM) ? 1 : 0;
	unsigned int dab_notch = (notch & RSP_TCP_NOTCH_DAB) ? 1 : 0;
	unsigned int bc_notch = (notch & RSP_TCP_NOTCH_BROADCAST) ? 1 : 0;

	switch (hardware_model)
	{
	case RSP_MODEL_RSP2:
		chParams->rsp2TunerParams.rfNotchEnable = rf_notch;
		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Rsp2_RfNotchControl, sdrplay_api_Update_Ext1_None);
		if (r != sdrplay_api_Success) {
			printf("set rf notch error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSP1A:
		deviceParams->devParams->rsp1aParams.rfDabNotchEnable = dab_notch;
		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Rsp1a_RfDabNotchControl, sdrplay_api_Update_Ext1_None);
		if (r != sdrplay_api_Success) {
			printf("set dab notch error (%d)\n", r);
		}
		deviceParams->devParams->rsp1aParams.rfNotchEnable = bc_notch;
		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Rsp1a_RfNotchControl, sdrplay_api_Update_Ext1_None);
		if (r != sdrplay_api_Success) {
			printf("set broadcast notch error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSPDUO:
		chParams->rspDuoTunerParams.rfDabNotchEnable = dab_notch;
		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_RspDuo_RfDabNotchControl, sdrplay_api_Update_Ext1_None);
		if (r != sdrplay_api_Success) {
			printf("set dab notch error (%d)\n", r);
		}
		chParams->rspDuoTunerParams.rfNotchEnable = bc_notch;
		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_RspDuo_RfNotchControl, sdrplay_api_Update_Ext1_None);
		if (r != sdrplay_api_Success) {
			printf("set broadcast notch error (%d)\n", r);
		}
		chParams->rspDuoTunerParams.tuner1AmNotchEnable = am_notch;
		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_RspDuo_Tuner1AmNotchControl, sdrplay_api_Update_Ext1_None);
		if (r != sdrplay_api_Success) {
			printf("set am notch error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSPDX:
		deviceParams->devParams->rspDxParams.rfDabNotchEnable = dab_notch;
		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_None, sdrplay_api_Update_RspDx_RfDabNotchControl);
		if (r != sdrplay_api_Success) {
			printf("set dab notch error (%d)\n", r);
		}
		deviceParams->devParams->rspDxParams.rfNotchEnable = bc_notch;
		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_None, sdrplay_api_Update_RspDx_RfNotchControl);
		if (r != sdrplay_api_Success) {
			printf("set broadcast notch error (%d)\n", r);
		}
		break;

	default:
		if (verbose) {
			printf("notch filter not supported\n");
		}
		break;
	}

	return 0;
}

static int set_gain_by_index(unsigned int index)
{
	int r;
	uint8_t if_gr, lnastate;

	if (index > GAIN_STEPS - 1) {
		printf("gain step %d out of range", index);
		return 0;
	}

	if (gain_index_to_gain(index, &if_gr, &lnastate) != 0) {
		printf("unable to get gain for current band\n");
		return 0;
	}

	gain_reduction = if_gr;
	lna_state = lnastate;

	chParams->tunerParams.gain.gRdB = if_gr;
	chParams->tunerParams.gain.LNAstate = lnastate;

	grc = 0;
	int count = 0;

	sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Tuner_Gr, sdrplay_api_Update_Ext1_None);

	while(count < timeout)
	{
		if(grc != 0)
		{
			printf("GR updated in %d ms\n", count);
			r = 0;
			break;
		}
#ifdef _WIN32
		Sleep(1);
#else
		usleep(1000);
#endif
		count++;
	}

	if(count >= timeout)
	{
		printf("GR failed to update in %.1f seconds\n", (timeout / 1000.0));
		r = 1;
	}

	apply_agc_settings();
	last_gain_idx = index;

	return r;
}

static int set_gain(unsigned int db)
{
	int p;
	unsigned int index;

	// quantise R820T gains in tenths of dB into indexes 
	p = ((9 + db) / 5);
	
	// clamp
	if (p > 100)
	{
		p = 100;
	}
	if (p < 0)
	{
		p = 0;
	}
	
	index = (unsigned int) (((GAIN_STEPS-1) / 100.0f) * p);
	return set_gain_by_index(index);
}

static int set_tuner_gain_mode(unsigned int mode)
{
	int r;

	if (mode) {
		chParams->ctrlParams.agc.enable = sdrplay_api_AGC_DISABLE;
		chParams->tunerParams.gain.LNAstate = lna_state;
		grc = 0;
		int count = 0;
		sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Ctrl_Agc | sdrplay_api_Update_Tuner_Gr, sdrplay_api_Update_Ext1_None);
		while(count < timeout)
		{
			if(grc != 0)
			{
				printf("GR updated in %d ms\n", count);
				r = 0;
				break;
			}
#ifdef _WIN32
			Sleep(1);
#else
			usleep(1000);
#endif
			count++;
		}

		if(count >= timeout)
		{
			printf("GR failed to update in %.1f seconds\n", (timeout / 1000.0));
			r = 1;
		}

		set_gain_by_index(last_gain_idx);
		printf("agc disabled\n");
	}
	else {
		chParams->ctrlParams.agc.enable = sdrplay_api_AGC_100HZ;
		chParams->ctrlParams.agc.setPoint_dBfs = agc_set_point;
		chParams->tunerParams.gain.LNAstate = lna_state;

		grc = 0;
		int count = 0;
		sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Ctrl_Agc | sdrplay_api_Update_Tuner_Gr, sdrplay_api_Update_Ext1_None);
		while(count < timeout)
		{
			if(grc != 0)
			{
				printf("GR updated in %d ms\n", count);
				r = 0;
				break;
			}
#ifdef _WIN32
			Sleep(1);
#else
			usleep(1000);
#endif
			count++;
		}

		if(count >= timeout)
		{
			printf("GR failed to update in %.1f seconds\n", (timeout / 1000.0));
			r = 1;
		}

		printf("agc enabled\n");
	}

	return r;
}

static int set_freq_correction(int32_t corr)
{
	int r;

	deviceParams->devParams->ppm = (double)corr;
	r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Dev_Ppm, sdrplay_api_Update_Ext1_None);
	if (r != sdrplay_api_Success) {
		printf("set freq correction error (%d)\n", r);
	}

	return r;
}

static int set_freq(uint32_t f)
{
	int r;
	rsp_band_t new_band, old_band;

	old_band = current_band;
	new_band = frequency_to_band(f);

	current_frequency = f;
	current_band = new_band;

	chParams->tunerParams.rfFreq.rfHz = f;

	rfc = 0;
	int count = 0;
	sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Tuner_Frf, sdrplay_api_Update_Ext1_None);

	while (count < timeout)
	{
		if(rfc != 0)
		{
			r = 0;
			printf("Frequency updated in %d ms\n", count);
			break;
		}
#ifdef _WIN32
		Sleep(1);
#else
		usleep(1000);
#endif
		count++;
	}

	if(count >= timeout)
	{
		r = 1;
		printf("Frequency failed to update in %.1f seconds\n", (timeout / 1000.0));
	}

	apply_agc_settings();

	// Reapply valid gain for new band
	if (old_band != new_band) {
		set_gain_by_index(last_gain_idx);
	}

	return r;
}

static int set_sample_rate(uint32_t sr)
{
	int r;
	double f;
	int decimation;

	if (sr < (2000000 / MAX_DECIMATION_FACTOR) || sr > 10000000) {
		printf("sample rate %u is not supported\n", sr);
		return -1;
	}

	decimation = 1;
	if (sr < 2000000)
	{
		int c = 0;

		// Find best decimation factor
		while (sr * (1 << c) < 2000000 && (1 << c) < MAX_DECIMATION_FACTOR) {
			c++;
		}

		decimation = 1 << c;

		if (sr >= 1536000 && sr <= 2000000)
		{
			bwType = sdrplay_api_BW_1_536;
		}
		else
		if (sr >= 600000 && sr < 1536000)
		{
			bwType = sdrplay_api_BW_0_600;
		}
		else
		if (sr >= 300000 && sr < 600000)
		{
			bwType = sdrplay_api_BW_0_300;
		}
		else
		{
			bwType = sdrplay_api_BW_0_200;
		}
	}
	else
	{
		if (sr >= 8000000)
		{
			bwType = sdrplay_api_BW_8_000;
		}
		else
		if (sr >= 7000000)
		{
			bwType = sdrplay_api_BW_7_000;
		}
		else
		if (sr >= 6000000)
			{
				bwType = sdrplay_api_BW_6_000;
			}
		else
		if (sr >= 5000000)
		{
			bwType = sdrplay_api_BW_5_000;
		}
		else
		{
			bwType = sdrplay_api_BW_1_536;
		}
	}

	f = (double)(sr * decimation);

	if (decimation == 1) {
		chParams->ctrlParams.decimation.enable = 0;
	}
	else {
		chParams->ctrlParams.decimation.enable = 1;
		chParams->ctrlParams.decimation.decimationFactor = decimation;
		chParams->ctrlParams.decimation.wideBandSignal = 1;
	}

	chParams->tunerParams.bwType = bwType;
	deviceParams->devParams->fsFreq.fsHz = f;

	printf("device SR %.2f, decim %d, output SR %u, IF Filter BW %d kHz\n", f, decimation, sr, bwType);

	fsc = 0;
	int count = 0;

	sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Dev_Fs | sdrplay_api_Update_Ctrl_Decimation, sdrplay_api_Update_Ext1_None);

	while(count < timeout)
	{
		if(fsc != 0)
		{
			printf("Sample rate changed after %d ms\n", count);
			break;
		}
#ifdef _WIN32
		Sleep(1);
#else
		usleep(1000);
#endif
		count++;
	}

	if(count >= timeout)
	{
		printf("Sample rate not changed after %.1f seconds\n", (timeout / 1000.0));
	}

	r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Tuner_BwType, sdrplay_api_Update_Ext1_None);

	if (r != sdrplay_api_Success) {
		printf("set bw error (%d)\n", r);
	}

	apply_agc_settings();

	return r;
}

#ifdef _WIN32
#define __attribute__(x)
#pragma pack(push, 1)
#endif
struct command {
	unsigned char cmd;
	unsigned int param;
}__attribute__((packed));
#ifdef _WIN32
#pragma pack(pop)
#endif

static void *command_worker(void *arg)
{
	int left, received = 0;
	fd_set readfds;
	struct command cmd = { 0, 0 };
	struct timeval tv = { 1, 0 };
	int r = 0;
	uint32_t tmp;

	while (1) {
		left = sizeof(cmd);
		while (left > 0) {
			FD_ZERO(&readfds);
			FD_SET(s, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(s + 1, &readfds, NULL, NULL, &tv);
			if (r) {
				received = recv(s, (char*)&cmd + (sizeof(cmd) - left), left, 0);
				left -= received;
			}
			if (received == SOCKET_ERROR || do_exit) {
#ifdef _WIN32
				printf("comm recv bye (%d), do_exit:%d\n", WSAGetLastError(), do_exit);
#else
				printf("comm recv bye, do_exit:%d\n", do_exit);
#endif
				sighandler(CTRL_CLOSE_EVENT);
				pthread_exit(NULL);
			}
		}
		switch (cmd.cmd) {
		case 0x01:
			printf("set freq %d\n", ntohl(cmd.param));
			set_freq(ntohl(cmd.param));
			break;
		case 0x02:
			printf("set sample rate %d\n", ntohl(cmd.param));
			set_sample_rate(ntohl(cmd.param));
			break;
		case 0x03:
			printf("set gain mode %d\n", ntohl(cmd.param));
			set_tuner_gain_mode(ntohl(cmd.param));
			break;
		case 0x04:
			printf("set gain %d\n", ntohl(cmd.param));
			set_gain(ntohl(cmd.param));
			break;
		case 0x05:
			printf("set freq correction %d\n", ntohl(cmd.param));
			set_freq_correction(ntohl(cmd.param));
			break;
		case 0x06:
			tmp = ntohl(cmd.param);
			printf("set if stage %d gain %d\n", tmp >> 16, (short)(tmp & 0xffff));
			break;
		case 0x07:
			printf("set test mode %d\n", ntohl(cmd.param));
			break;
		case 0x08:
			printf("set agc mode %d\n", ntohl(cmd.param));
			break;
		case 0x09:
			printf("set direct sampling %d\n", ntohl(cmd.param));
			break;
		case 0x0a:
			printf("set offset tuning %d\n", ntohl(cmd.param));
			break;
		case 0x0b:
			printf("set rtl xtal %d\n", ntohl(cmd.param));
			break;
		case 0x0c:
			printf("set tuner xtal %d\n", ntohl(cmd.param));
			break;
		case 0x0d:
			printf("set tuner gain by index %d\n", ntohl(cmd.param));
			set_gain_by_index(ntohl(cmd.param));
			break;
		case 0x0e:
			printf("set bias tee %d\n", ntohl(cmd.param));
			set_bias_t((int)ntohl(cmd.param));
			break;

			// Extended mode commands
		case RSP_TCP_COMMAND_SET_ANTENNA:
			if (extended_mode) {
				printf("set antenna input %d\n", ntohl(cmd.param));
				set_antenna_input((unsigned int)ntohl(cmd.param));
			}
			break;

		case RSP_TCP_COMMAND_SET_NOTCH:
			if (extended_mode) {
				printf("set notch filter 0x%x\n", ntohl(cmd.param));
				set_notch_filters((unsigned int)ntohl(cmd.param));
			}
			break;

		case RSP_TCP_COMMAND_SET_LNASTATE:
			if (extended_mode) {
				printf("set LNAState %d\n", ntohl(cmd.param));
				set_lna((unsigned int)ntohl(cmd.param));
			}
			break;

		case RSP_TCP_COMMAND_SET_IF_GAIN_R:
			if (extended_mode) {
				printf("set if gain reduction %d\n", ntohl(cmd.param));
				set_if_gain_reduction((int)ntohl(cmd.param));
			}
			break;

		case RSP_TCP_COMMAND_SET_AGC:
			if (extended_mode) {
				printf("set agc %d\n", ntohl(cmd.param));
				set_agc((unsigned int)ntohl(cmd.param));
			}
			break;

		case RSP_TCP_COMMAND_SET_AGC_SETPOINT:
			if (extended_mode) {
				printf("set agc set point %d\n", ntohl(cmd.param));
				set_agc_setpoint((int)ntohl(cmd.param));
			}
			break;

		case RSP_TCP_COMMAND_SET_BIAST:
			if (extended_mode) {
				printf("set bias-t %d\n", ntohl(cmd.param));
				set_bias_t((unsigned int)ntohl(cmd.param));
			}
			break;

		case RSP_TCP_COMMAND_SET_REFOUT:
			if (extended_mode) {
				printf("set reference out %d\n", ntohl(cmd.param));
				set_refclock_output((unsigned int)ntohl(cmd.param));
			}
			break;

		default:
			break;
		}
		cmd.cmd = 0xff;
	}
}

int init_rsp_device(unsigned int sr, unsigned int freq, int enable_bias_t, unsigned int notch, int enable_refout, int antenna)
{
	int r, dec;
	uint8_t ifgain, lnastate;

	// initialise frequency state
	current_band = frequency_to_band(freq);
	current_frequency = freq;

	// initialise at minimum gain	
	if (!gain_index_to_gain(0, &ifgain, &lnastate)) {
		gain_reduction = ifgain;
		lna_state = lnastate;
	}
	
	if (sr < 300e3) { bwType = sdrplay_api_BW_0_200; }
	else if (sr < 600e3) { bwType = sdrplay_api_BW_0_300; }
	else if (sr < 1536e3) { bwType = sdrplay_api_BW_0_600; }
	else if (sr < 5e6) { bwType = sdrplay_api_BW_1_536; }
	else if (sr < 6e6) { bwType = sdrplay_api_BW_5_000; }
	else if (sr < 7e6) { bwType = sdrplay_api_BW_6_000; }
	else if (sr < 8e6) { bwType = sdrplay_api_BW_7_000; }
	else { bwType = sdrplay_api_BW_8_000; }
	
	dec = 1;
	if (sr < 2e6)
	{
		while (sr < 2e6)
		{
			sr = sr * 2;
			dec = dec * 2;
		}
	}

	if (chosenDev->hwVer == SDRPLAY_RSPduo_ID)
	{
		chosenDev->rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;
		chosenDev->rspDuoSampleFreq = sr;
		if (antenna == 0 || antenna == 2)
		{
			chosenDev->tuner = sdrplay_api_Tuner_A;
		}
		else
		{
			chosenDev->tuner = sdrplay_api_Tuner_B;
		}

		chParams = chosenDev->tuner == sdrplay_api_Tuner_B ? deviceParams->rxChannelB : deviceParams->rxChannelA;
	}
	chParams->tunerParams.gain.gRdB = gain_reduction;
	deviceParams->devParams->fsFreq.fsHz = sr;
	chParams->tunerParams.rfFreq.rfHz = freq;
	chParams->tunerParams.bwType = bwType;
	chParams->tunerParams.ifType = sdrplay_api_IF_Zero;
	chParams->tunerParams.gain.LNAstate = lna_state;

	cbFns.StreamACbFn = rxa_callback;
	cbFns.StreamBCbFn = rxb_callback;
	cbFns.EventCbFn = event_callback;

	fsc = 0;
	rfc = 0;
	grc = 0;
	int count = 0;
	//int donefs = 0; int donerf = 0; int donegr = 0;

	r = sdrplay_api_Init(chosenDev->dev, &cbFns, NULL);

	while (count < timeout)
	{
	/*	if (fsc != 0 && donefs == 0)
		{
			printf("Sample rate changed after init in %d ms\n", count);
			donefs = 1;
		}
		if (rfc != 0 && donerf == 0)
		{
			printf("Frequency changed after init in %d ms\n", count);
			donerf = 1;
		}*/
		if (grc != 0) // && donegr == 0)
		{
			printf("GR changed after init in %d ms\n", count);
			//donegr = 1;
			break;
		}
	/*	if (fsc != 0 && rfc != 0 && grc != 0)
		{
			printf("Everything changed after init in %d ms\n", count);
			break;
		} */
#ifdef _WIN32
		Sleep(1);
#else
		usleep(1000);
#endif
		count++;
	}

	if (count >= timeout)
	{
		printf("Something failed to change after init - Fs:%d, Rf:%d, Gr:%d\n", fsc, rfc, grc);
	}

	if (r != sdrplay_api_Success) {
		fprintf(stderr, "failed to start the RSP device, return (%d)\n", r);
		return -1;
	}
	
	if (dec > 1)
	{
		chParams->ctrlParams.decimation.enable = 1;
		chParams->ctrlParams.decimation.decimationFactor = dec;
		chParams->ctrlParams.decimation.wideBandSignal = 1;
		r = sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Ctrl_Decimation, sdrplay_api_Update_Ext1_None);

		if (r != sdrplay_api_Success) {
			printf("set decimation error, return (%d)\n", r);
		}
	}

	printf("started rx\n");
	
	// set bias-T
	set_bias_t(enable_bias_t);

	// set notch filters
	set_notch_filters(notch);

	// set external reference output
	set_refclock_output(enable_refout);

	// set antenna input
	set_antenna_input(antenna);

	apply_agc_settings();

	return 0;
}

void usage(void)
{
	printf(SERVER_NAME", an I/Q spectrum server for SDRPlay receivers, "
		"version "SERVER_VERSION
		"\n\n"
		"Usage:\n"
		"\t"SERVER_NAME" [OPTIONS]\n\n"
		"Options:\n"
		"\t-a listen address\n"
		"\t-p listen port (default: 1234)\n"
		"\t-d RSP device to use (default: 1, first found)\n"
		"\t-P Antenna Port select* (0/1/2, default: 0, Port A)\n"
		"\t-T Bias-T enable* (default: disabled)\n"
		"\t-R Refclk output enable* (default: disabled)\n"
		"\t-f frequency to tune to [Hz]\n"
		"\t-s samplerate in Hz (default: 2048000 Hz)\n"
		"\t-n max number of linked list buffers to keep (default: 500)\n"
		"\t-v Verbose output (debug) enable (default: disabled)\n"
		"\t-E RSP extended mode enable (default: rtl_tcp compatible mode)\n"
		"\t-A AM notch enable (default: disabled)\n"
		"\t-B Broadcast notch enable (default: disabled)\n"
		"\t-D DAB notch enable (default: disabled)\n"
		"\t-F RF notch enable (default: disabled)\n"
		"\t-b Sample bit-depth (8/16 default: 8)\n"
		"\t-h This help\n");
	exit(1);
}

int main(int argc, char **argv)
{
	int r, opt;
	char* addr = "127.0.0.1";
	int port = 1234;
	uint32_t frequency = DEFAULT_FREQUENCY, samp_rate = DEFAULT_SAMPLERATE;
	struct sockaddr_in local, remote;
	struct llist *curelem, *prev;
	pthread_attr_t attr;
	void *status;
	struct timeval tv = { 1,0 };
	struct linger ling = { 1,0 };
	SOCKET listensocket;
	socklen_t rlen;
	fd_set readfds;
	dongle_info_t dongle_info;

	float ver;

	unsigned int numDevs;
	unsigned int notch = 0;
	int device = 0;
	int antenna = 0;
	int enable_biastee = 0;
	int enable_refout = 0;
	int bit_depth = 8;

#ifdef _WIN32
	WSADATA wsd;
	int i = WSAStartup(MAKEWORD(2, 2), &wsd);
#else
	struct sigaction sigact, sigign;
#endif

	while ((opt = getopt(argc, argv, "a:p:f:b:s:n:d:P:TvADBFREh")) != -1) {
		switch (opt) {
		case 'd':
			device = atoi(optarg) - 1;
			break;
		case 'b':
			bit_depth = atoi(optarg);
			break;
		case 'P':
			antenna = atoi(optarg);
			break;
		case 'f':
			frequency = (uint32_t)atofs(optarg);
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			break;
		case 'a':
			addr = optarg;
			break;
		case 'p':
			port = atoi(optarg);
			break;
		case 'n':
			llbuf_num = atoi(optarg);
			break;

		case 'T':
			enable_biastee = 1;
			break;

		case 'R':
			enable_refout = 1;
			break;
		case 'v':
			verbose = 1;
			break;

		case 'E':
			extended_mode = 1;
			break;
		case 'A':
			notch |= RSP_TCP_NOTCH_AM;
			break;
		case 'D':
			notch |= RSP_TCP_NOTCH_DAB;
			break;
		case 'B':
			notch |= RSP_TCP_NOTCH_BROADCAST;
			break;
		case 'F':
			notch |= RSP_TCP_NOTCH_RF;
			break;
		case 'h':
		default:
			usage();
			break;
		}
	}

	printf(SERVER_NAME" version %s\n\n", SERVER_VERSION);

	if (bit_depth != 8 && bit_depth != 16) {
		usage();
	}

	sample_format = bit_depth == 16 ? RSP_TCP_SAMPLE_FORMAT_INT16 : RSP_TCP_SAMPLE_FORMAT_UINT8;

	if (argc < optind) {
		usage();
	}

	r = sdrplay_api_Open();
	if (r != sdrplay_api_Success) {
		fprintf(stderr, "Cannot connect to API service\n");
		exit(1);
	}

	r = sdrplay_api_LockDeviceApi();
	if (r != sdrplay_api_Success) {
		fprintf(stderr, "Cannot lock the API\n");
		sdrplay_api_Close();
		exit(1);
	}

	// check API version
	r = sdrplay_api_ApiVersion(&ver);
	if (ver != SDRPLAY_API_VERSION) {
		//  Error detected, include file does not match dll. Deal with error condition.
		fprintf(stderr, "API library must be version %.2f\n", ver);
		exit(1);
	}
	printf("API library version %.2f found\n", ver);

	// enable debug output
	if (verbose) {
		sdrplay_api_DebugEnable(NULL, 1);
	}

	// select RSP device
	r = sdrplay_api_GetDevices(devices, &numDevs, MAX_DEVS);
	if (r != sdrplay_api_Success) {
		fprintf(stderr, "Failed to get device list (%d)\n", r);
		exit(1);
	}

	if (numDevs == 0) {
		fprintf(stderr, "no RSP devices available.\n");
		exit(1);
	}

	chosenDev = &devices[device];

	if (chosenDev->hwVer == SDRPLAY_RSPduo_ID)
	{
		chosenDev->rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;
	}

	r = sdrplay_api_SelectDevice(chosenDev);
	if (r != sdrplay_api_Success) {
		fprintf(stderr, "Failed to set device index (%d)\n", r);
		sdrplay_api_UnlockDeviceApi();
		sdrplay_api_Close();
		exit(1);
	}

	sdrplay_api_UnlockDeviceApi();

	// get RSP model
	hardware_version = devices[device].hwVer;
	hardware_model = hardware_ver_to_model(hardware_version);
	hardware_caps = model_to_capabilities(hardware_model);

	if (hardware_model == RSP_MODEL_UNKNOWN || hardware_caps == NULL) {
		printf("unknown RSP model (hw ver %d)\n", hardware_version);

		// force compatibility mode when model is unknown
		extended_mode = 0;
	}
	else {
		printf("detected RSP model '%s' (hw ver %d)\n", model_to_string(hardware_model), hardware_version);
	}

	r = sdrplay_api_GetDeviceParams(chosenDev->dev, &deviceParams);
	if (r != sdrplay_api_Success) {
		fprintf(stderr, "Cannot get device params (%d)\n", r);
		sdrplay_api_ReleaseDevice(chosenDev);
		sdrplay_api_Close();
		exit(1);
	}

	chParams = (chosenDev->tuner == sdrplay_api_Tuner_B) ? deviceParams->rxChannelB : deviceParams->rxChannelA;

	// enable DC offset and IQ imbalance correction
	chParams->ctrlParams.dcOffset.DCenable = 1;
	chParams->ctrlParams.dcOffset.IQenable = 1;
	// disable decimation and  set decimation factor to 1
	chParams->ctrlParams.decimation.decimationFactor = 1;

	if (chosenDev->hwVer == SDRPLAY_RSPduo_ID)
	{
		chosenDev->rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;
	}

#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigign.sa_handler = SIG_IGN;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigign, NULL);
#else
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)sighandler, TRUE);
#endif

	pthread_mutex_init(&ll_mutex, NULL);
	pthread_cond_init(&cond, NULL);

	memset(&local, 0, sizeof(local));
	local.sin_family = AF_INET;
	local.sin_port = htons(port);
	local.sin_addr.s_addr = inet_addr(addr);

	listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	r = 1;
	setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
	setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
	bind(listensocket, (struct sockaddr *)&local, sizeof(local));

#ifdef _WIN32
	opt = 1;
	ioctlsocket(listensocket, FIONBIO, &opt);
#else
	r = fcntl(listensocket, F_GETFL, 0);
	r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);
#endif

	while (1) {
		printf("listening...\n");

		if (!extended_mode) {
			printf("Use the device argument 'rtl_tcp=%s:%d' in OsmoSDR "
				"(gr-osmosdr) source\n"
				"to receive samples in GRC and control "
				"rtl_tcp parameters (frequency, gain, ...).\n",
				addr, port);
		}
		listen(listensocket, 1);

		while (1) {
			FD_ZERO(&readfds);
			FD_SET(listensocket, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(listensocket + 1, &readfds, NULL, NULL, &tv);
			if (do_exit) {
				goto out;
			}
			else if (r) {
				rlen = sizeof(remote);
				s = accept(listensocket, (struct sockaddr *)&remote, &rlen);
				break;
			}
		}

		setsockopt(s, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

		printf("client accepted!\n");

		memset(&dongle_info, 0, sizeof(dongle_info));
		memcpy(&dongle_info.magic, "RTL0", 4);

		dongle_info.tuner_type = htonl(RTLSDR_TUNER_R820T);
		dongle_info.tuner_gain_count = htonl(GAIN_STEPS-1);

		r = send(s, (const char *)&dongle_info, sizeof(dongle_info), 0);
		if (sizeof(dongle_info) != r) {
			printf("failed to send dongle information\n");
		}

		if (extended_mode)
		{
			rsp_extended_capabilities_t rsp_cap;

			printf("sending RSP extended capabilities structure\n");

			memset(&rsp_cap, 0, sizeof(rsp_extended_capabilities_t));
			memcpy(&rsp_cap.magic, RSP_CAPABILITIES_MAGIC, 4);

			rsp_cap.version = htonl(RSP_CAPABILITIES_VERSION);
			rsp_cap.hardware_version = htonl(hardware_version);
			rsp_cap.capabilities = htonl(hardware_caps->capabilities);
			rsp_cap.sample_format = htonl(sample_format);

			rsp_cap.antenna_input_count = hardware_caps->antenna_input_count;
			strcpy(rsp_cap.third_antenna_name, hardware_caps->third_antenna_name);
			rsp_cap.third_antenna_freq_limit = hardware_caps->third_antenna_freq_limit;
			rsp_cap.tuner_count = hardware_caps->tuner_count;
			rsp_cap.ifgr_min = hardware_caps->min_ifgr;
			rsp_cap.ifgr_max = hardware_caps->max_ifgr;

			r = send(s, (const char *)&rsp_cap, sizeof(rsp_cap), 0);
			if (sizeof(rsp_cap) != r) {
				printf("failed to send RSP capabilities information\n");
			}
		}

		// must start the tcp_worker before the first samples are available from the rx
		// because the rx_callback tries to send a condition to the worker thread
		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		r = pthread_create(&tcp_worker_thread, &attr, tcp_worker, NULL);
		if (r != 0) {
			printf("failed to create tcp worker thread\n");
			break;
		}

		// initialise API and start the rx		
		r = init_rsp_device(samp_rate, frequency, enable_biastee, notch, enable_refout, antenna);
		if (r != 0) {
			printf("failed to initialise RSP device\n");
			break;
		}

		// the rx must be started before accepting commands from the command worker
		r = pthread_create(&command_thread, &attr, command_worker, NULL);
		if (r != 0) {
			printf("failed to create command thread\n");
			break;
		}
		pthread_attr_destroy(&attr);

		// wait for the workers to exit
		pthread_join(tcp_worker_thread, &status);
		pthread_join(command_thread, &status);

		closesocket(s);

		// stop the receiver
		sdrplay_api_Uninit(chosenDev->dev);
		printf("all threads dead..\n");

		curelem = ll_buffers;
		ll_buffers = 0;

		while (curelem != 0) {
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}

		if (!ctrlC_exit) do_exit = 0;
		global_numq = 0;
	}

out:
	sdrplay_api_ReleaseDevice(chosenDev);
	sdrplay_api_Close();

	closesocket(listensocket);
	closesocket(s);
#ifdef _WIN32
	WSACleanup();
#endif
	printf("bye!\n");
	return r >= 0 ? r : -r;
}
