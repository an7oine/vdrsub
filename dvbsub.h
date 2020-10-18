/*

 A DVB subtitling decoder model suitable for Finnish YLE broadcasts

 NOTE: this model does not implement the standard in full, cf. the specification below.

 http://www.etsi.org/deliver/etsi_en/300700_300799/300743/01.03.01_60/en_300743v010301p.pdf

 -A simple VobSub encoder and a DVBsub -> VobSub translation
 convenience wrapper is also included

*/

#ifndef __DVBSUB_H
#define __DVBSUB_H

typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned long dword;
typedef unsigned long long qword;

enum live_state_flag { WIPE, NONE, DRAW, STAY };
struct colour { byte y,cr,cb,t; }; // values as per ITU-R BT.601

typedef struct {
    enum live_state_flag live_state;	// see above
    int x,y, w,h;               // left, top, width, height
    byte *pix;                  // pixel data as indexes to the clut
	struct colour clut[256];
	void *ctx;	// internal context data for continuous dvbsub processing
} subpicture;


// Set verbosity (to stderr) level of messages output when decoding subpictures
// 0 =none; 1 =errors; 2 =decoded subtitling segments; 3 =both
// NOTE: level gets overridden by env. variable SUB_VERBOSE, if found
// NOTE: -DVERBOSE is needed when compiling dvbsub.c to actually enable output
void init_verbose(int level);

// Initialize and return a new context for dvbsub stream decoding
// NOTE: the same context should be used throughout the translation of a stream
subpicture init_subp(void);

// Free allocated buffers and context data when a subtitle stream finishes
void release_subp(subpicture subp);

// Decode given data, update live_state and fill in possible decoded picture
void decode_dvbsub(byte *data, size_t len, subpicture *dst);

/////////////
// The following is implemented in dvbsub.c iff -DVOBSUB is given on compile :

// Decode a dvbsub packet in 'data' with length 'len' and encode the
// possible generated subpicture into a bare vobsub packet
// (w/o the PS headers and stuffing needed in an actual .sub file)
// Returns the generated packet or NULL if no draw or wipe is to be done
// NOTE: freeing of the returned data pointer ('p') is up to the caller;
// length of the vobsub packet can be determined by p[0]<<8 | p[1]
byte *dvb2vobsub_translation(byte *data,size_t len,subpicture *ctx);

#endif
