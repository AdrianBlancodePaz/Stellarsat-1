#ifndef STELLARSAT_FRAMES_H
#define STELLARSAT_FRAMES_H
#include <inttypes.h>

#define BROADCAST_ID 0xDD //Broadcast identifier
#define BEACON 0x00 //Beacon Identifyer of TM (defined option list)
#define LEOP_MODE 2
#define OPERATION_MODE 3
#define SAFE_MODE 1
int create_housekeeping(uint8_t *beacon_frame);
int create_header(uint8_t *hdr, uint8_t *size_hdr);
int create_telemetry(uint8_t *telemetry_frame, int byte_start, int byte_end, uint16_t meas_id);


#endif