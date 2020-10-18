#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>

#pragma pack(1)

#include "dvbsub.h"

#define NETWORD(x) (x = (word) ntohs(x))
#define isnum(a) ((a)>='0' && (a)<='9')
#define isalpha(a) ((a)>='a' && (a)<='z')

// in dvbsub.c :
#ifdef VERBOSE
extern int verbose_level;
void verb(int level, const char *format, ...);
#else
#define verb(...)
#endif
qword loadbits(int bits, byte **p, byte *b, byte *mask);

// in write-ps.c :
void write_vobsub_ps(byte*,size_t,qword,FILE*);

subpicture subp;	// stores the subpicture decoding context
FILE *dotsub;		// output files
FILE *dotidx;
qword first_video_pts = 0;	// subtracted from each subpicture PTS
word pmt_pid = -1, video_pid = -1, sub_pid = -1;	// PIDs found in the TS
word composition_id = -1, ancillary_id = -1;

enum { TS, VDR } input_type = TS;
enum { CONVERT=1, PSI=2 } operation = CONVERT | PSI;
char *language = NULL;

// Only used when processing a .VDR file
static byte *pes_data = NULL;	// aggregate subtitle PES payload here
static size_t pes_len = 0;
static qword pes_pts = 0;		// obtain PTS from each leading packet

void process_dvbsub_data(byte *p, size_t length, qword pts)
{
	// When carrying a DVB subtitle stream, PES packet data content is:
	// 1 byte : data_identifier 				=0x20
	// 1 byte : subtitle_stream_id				=0x00
	//  bytes : subtitling_segments  (begin with 0x0F)
	// 1 byte : end_of_PES_data_field_marker	=0xFF

	if (p[0] != 0x20 || p[1] != 0x00)
		return;
	p += 2; length -= 2;

	verb(16,"Processing dvbsub data length=%d\n",length);
	byte *vobsub = dvb2vobsub_translation(p,length-1,&subp);

	if (!vobsub)
		return;
	float s=(pts - first_video_pts)/90000.0;
	
#ifdef VERBOSE
	if (subp.live_state == DRAW)
	{ verb(4,"%02d:%02d:%02d:%03d :\n",
	 (int) s/3600,(int) (s/60) % 60,(int) s % 60,
	 (int) ((s-((int) s))*1000));
	for (int i=0; i<subp.h; i++, verb(8,"\n"))
		for (int j=0; j<subp.w && j<80; j++)
			verb(8,"%c",' '+subp.pix[i*subp.w + j]); }
	else
		verb(4,"---> %02d:%02d:%02d:%03d\n-----------------\n",
		 (int) s/3600,(int) (s/60) % 60,(int) s % 60,
		 (int) ((s-((int) s))*1000));
	if (verbose_level & 32768)
	{
		FILE *raw = fopen("vobsub.dat","ab");
		fwrite(vobsub,1,((word) vobsub[0])<<8 | vobsub[1],raw);
		fclose(raw);
	}	 
#endif

	if (operation & CONVERT)
	{
		fprintf(dotidx,"timestamp: %02d:%02d:%02d:%03d, filepos: %08X\n",
		 (int) s/3600,(int) (s/60) % 60,(int) s % 60,
		 (int) ((s-((int) s))*1000), ftell(dotsub));
		write_vobsub_ps(vobsub, ((word) vobsub[0])<<8 | vobsub[1], pts - first_video_pts, dotsub);
	}
	free(vobsub);
}
	
void process_pes_packet(byte data[])
{
	byte *p = data;

	struct pes_packet {
		byte sync_bytes[3];
		byte stream_id;
		word packet_length;

		word flags;
		byte pes_header_length;

		qword pts, dts;
		qword escr_base;
		word escr_ext;
		qword es_rate;

		byte trick;
		byte copy_info;
		word prev_crc;

		byte ext_flags;
		byte pes_private[16];
		byte pack_length;
		byte *pack_header;

		byte seq_counter;
		byte orig_stuff_length;
		word p_std;
	} pes = { .pts = 0 };
	memcpy(&pes,p,6); p+=6;

	NETWORD(pes.packet_length);

	verb(32,"PES: %d bytes, type id 0x%02X: ",pes.packet_length+6,pes.stream_id);
	switch (pes.stream_id)
	{
	case 0xBC: verb(32,"Program stream map\n"); return;
	case 0xBD: verb(32,"Subtitles, i.e. Private stream 1\n"); break;
	case 0xBE: verb(32,"Padding stream\n"); return;
	case 0xBF: verb(32,"Private stream 2\n"); return;
	case 0xF0: verb(32,"ECM stream\n"); return;
	case 0xF1: verb(32,"EMM stream\n"); return;
	case 0xF2: verb(32,"ITU-T Rec. H.222.0 | ISO/IEC 13818-1-A etc.\n"); return;
	case 0xF3: verb(32,"ISO/IEC_13522 stream\n"); break;
	case 0xF4: verb(32,"ITU-T Rec. H.222.1 type A\n"); break;
	case 0xF5: verb(32,"ITU-T Rec. H.222.1 type B\n"); break;
	case 0xF6: verb(32,"ITU-T Rec. H.222.1 type C\n"); break;
	case 0xF7: verb(32,"ITU-T Rec. H.222.1 type D\n"); break;
	case 0xF8: verb(32,"ITU-T Rec. H.222.1 type E\n"); return;
	case 0xF9: verb(32,"ancillary stream\n"); break;
	case 0xFA: verb(32,"ISO/IEC14496-1 SL-packetized stream\n"); break;
	case 0xFB: verb(32,"ISO/IEC14496-1 FlexMux stream\n"); break;
	case 0xFC: case 0xFD: case 0xFE: verb(32,"reserved data stream\n"); break;
	case 0xFF: verb(32,"Program stream directory\n"); return;
	default: if ((pes.stream_id & 0xE0) == 0xC0)
			verb(32,"Audio, i.e. ISO/IEC 13818-3, etc.\n");
		else if ((pes.stream_id & 0xF0) == 0xE0)
            verb(32,"Video, i.e. ITU-T Rec. H.262 | ISO/IEC 13818-2, etc.\n");
		else verb(32,"stream_id=0x%X\n",pes.stream_id); break;
	}

	memcpy(&pes.flags,p,3); p+=3;
	NETWORD(pes.flags);
	if (pes.flags & 0x0080)
	{
		pes.pts = (*(p++) & 0x0E) << 29; pes.pts |= *(p++) << 22;
		pes.pts |= (*(p++) & 0xFE) << 14;
		pes.pts |= *(p++) << 7; pes.pts |= *(p++) >> 1;
		if ((pes.stream_id & 0xF0) == 0xE0 && first_video_pts == 0)
		{ first_video_pts = pes.pts; verb(16,"First video pts established\n"); }
	}
	if (pes.flags & 0x0040)
	{
		pes.dts = (*(p++) & 0x0E) << 29; pes.dts |= *(p++) << 22;
		pes.dts |= (*(p++) & 0xFE) << 14;
		pes.dts |= *(p++) << 7; pes.dts |= *(p++) >> 1;
	}
	if (pes.flags & 0x0020)
	{
		byte b,m=0;
		loadbits(2,&p,&b,&m); pes.escr_base = loadbits(3,&p,&b,&m) << 30;
		loadbits(1,&p,&b,&m); pes.escr_base |= loadbits(8,&p,&b,&m) << 22;
		pes.escr_base |= loadbits(7,&p,&b,&m) << 15;
		loadbits(1,&p,&b,&m); pes.escr_base |= loadbits(8,&p,&b,&m) << 7;
        pes.escr_base |= loadbits(7,&p,&b,&m);
		loadbits(1,&p,&b,&m); pes.escr_ext = loadbits(8,&p,&b,&m) << 1;
		pes.escr_ext |= loadbits(1,&p,&b,&m);
		loadbits(0,NULL,&b,&m);
	}
	if (pes.flags & 0x0010)
	{
		pes.es_rate = (*(p++) & 0x7F) << 15;
		pes.es_rate |= *(p++) << 7; pes.es_rate |= *(p++) >> 1;
	}
	if (pes.flags & 0x0008)
		pes.trick = *(p++);
	if (pes.flags & 0x0004)
		pes.copy_info = *(p++);
	if (pes.flags & 0x0002)
	{
		pes.prev_crc = *(p++) << 8; pes.prev_crc |= *(p++);
	}
	if (pes.flags & 0x0001)
	{
		pes.ext_flags = *(p++);
		if (pes.ext_flags & 0x80)
		{
			memcpy(pes.pes_private,p,16); p += 16;
		}
		if (pes.ext_flags & 0x40)
		{
			pes.pack_length = *(p++);
			memcpy(pes.pack_header = malloc(pes.pack_length),
			 p,pes.pack_length);
			p += pes.pack_length;
		}
		if (pes.ext_flags & 0x20)
		{
			pes.seq_counter = *(p++) & 0x7F;
			pes.orig_stuff_length = *(p++) & 0x3F;
		}
		if (pes.ext_flags & 0x10)
		{
			pes.p_std = *(p++) << 8; pes.p_std |= *(p++);
		}
		if (pes.ext_flags & 0x01)
			p += *(p++) & 0x7F;
	}
	
	// skip possible stuffing bytes
	p = data + 9 + pes.pes_header_length;
	
	// Only process private stream 1 any further
	if (pes.stream_id != 0xBD) return;
	
	verb(16,"PES: subtitle packet, packet =%d, header =%d, payload: %02X %02X %02X %02X %02X %02X %02X\n",
			pes.packet_length, pes.pes_header_length, p[0],p[1],p[2],p[3],p[4],p[5],p[6]);
	if (input_type == VDR)
	{
		if (p[3] == 0)
		{
			if (pes_len > 0)
				// before beginning a new packet sequence, process any previously accumulated payload
				process_dvbsub_data(pes_data, pes_len, pes_pts);
			pes_len = 0;
			pes_pts = pes.pts;
		}
		p += 4;	// VDR quirks: skip a 0x20010000 (new PES sequence) or 0x20010001 (continued sequence) header
	}

	// PES.PACKET_LENGTH=     	extra fields + 2 +  dvbsub payload
	pes.packet_length -= p - (data+6);
	// now				=						 	dvbsub payload
	
	// cache subtitle payload from .VDR content, further processing is handled above
	if (input_type == VDR)
	{
		verb(16,"Appending %d bytes to %d\n",pes.packet_length,pes_len);
		pes_data = realloc(pes_data, pes_len + pes.packet_length);
		memcpy(pes_data+pes_len, p, pes.packet_length);
		pes_len += pes.packet_length;
	}
	// process subtitle payload immediately for TS content
	else
		process_dvbsub_data(p, pes.packet_length, pes.pts);
}

void process_subtitle_pes_chunk(byte *p, size_t len, byte continuity_counter)
{
	static byte *sub_pes = NULL;
	static size_t sub_len = 0, sub_complete_len = -1;

	static byte sub_counter = 16;
	if (sub_counter == continuity_counter)
	{	verb(32,"PES is a duplicate subtitle packet\n"); return; }
	sub_counter=continuity_counter;

	if (sub_complete_len == -1)
	{
		if (memcmp(p,(byte []) {0,0,1},3))
		{
			verb(32,"PES: sync bytes not found; skipping\n");
			return;
		}
		sub_complete_len = 6 + (((word) p[4])<<8 | p[5]); // header + payload
	}
	
	verb(32,"PES packet has complete length 0x%X (%d), we have %d\n",sub_complete_len,sub_complete_len,sub_len);
	sub_pes = realloc(sub_pes, sub_len + len); memcpy(sub_pes+sub_len, p, len); sub_len += len;
		
	if (sub_len >= sub_complete_len)
	{
		process_pes_packet(sub_pes);
		free(sub_pes); sub_pes = NULL; 
		sub_len = 0; sub_complete_len = -1;
	}
}

void process_psi_section(byte *data)
{
	byte *p = data;

	switch (*(p++))
	{
	case 0: {
		verb(128,"Program Association Section :\n");
		struct {
			word syntax_length;
			word ts_stream_id;
			byte ver_current_next;
			byte section_number;
			byte last_section_number;
		} pat;
		memcpy(&pat,p,sizeof pat); p += sizeof pat;
		NETWORD(pat.syntax_length); NETWORD(pat.ts_stream_id);

		verb(128,"syntax_length=%04X, ts_stream_id=%04X, ver_current_next=%02X, section_number=%02X\n",
		 pat.syntax_length,pat.ts_stream_id,pat.ver_current_next,pat.section_number);
		verb(128,"last_section_number=%02X, program/PMT assignments :\n",pat.last_section_number);
		while (p <= data+3+(pat.syntax_length&0x0FFF)-8)
		{
			word program = ((word) *(p++)) << 8 | *(++p);
			word pid = (((word) *(p++)) << 8 | *(++p)) & 0x1FFF;
			if (!program) { verb(128," -Network PID =%04X\n",pid); continue; }
			if (pid < 0x10 || pid == 0x1FFF) continue;
			verb(128," -Program %04X has PMT PID %04X\n",program,pid);
			if (operation & PSI) 
				printf("Carrying program 0x%04X with PMT PID 0x%04X\n",program,pid);
			pmt_pid = pid;
		}
		p += 4; // skip CRC_32
		} break;
	case 2: {
		verb(128,"Program Map Section :\n");
		struct {
			word section_length;
			word program;
			byte ver_current_next;
			byte section_number;
			byte last_section_number;
			word pcr_pid;
			word program_info_length;
		} pmt;
		memcpy(&pmt, p, sizeof pmt); p += sizeof pmt;
		NETWORD(pmt.section_length);
		pmt.section_length &= 0x0FFF;
		NETWORD(pmt.program);
		NETWORD(pmt.pcr_pid); NETWORD(pmt.program_info_length);

		verb(128,"section_length=%04X, program=%04X, ver_current_next=%02X, section_number=%02X\n",
		 pmt.section_length,pmt.program,pmt.ver_current_next,pmt.section_number);
		verb(128,"last_section_number=%02X, pcr_pid=%04X, program_info_length=%04X\n",
			pmt.last_section_number,pmt.pcr_pid,pmt.program_info_length&0x0FFF);
		p += pmt.program_info_length & 0x0FFF;
		while (p <= data+3+pmt.section_length-9)
		{
			byte stream_type = *(p++);
			word elementary_pid = (((word) *(p++)) << 8 | *(++p)) & 0x1FFF;
			word len = (((word) *(p++)) << 8 | *(++p)) & 0x0FFF;
			verb(128," -type=%02X, pid=%04X, descriptor len=%d",
				stream_type, elementary_pid, len);
			if (stream_type == 2)
			{ 
				verb(128,", video track");
				if (operation & PSI)
					printf("Video track with PID 0x%04X\n",elementary_pid);
				video_pid = elementary_pid;
			}
			if (stream_type == 4)
			{
				verb(128,", audio track");
				if (operation & PSI)
					printf("Audio track with PID 0x%04X\n",elementary_pid);
			}
			// Only process non-empty descriptors of private data streams
			if (stream_type != 6 || len == 0) { verb(128,"\n"); p+=len; continue; }
			
			byte tag = *(p++); len = *(p++);	// descriptor tag and length
			switch (tag)
			{
			case 0x56:
				while (len >= 5)
				{
					struct {
						byte language[3];
						byte magazine_number : 3; byte teletext_type : 5;
						byte page_number;
					} ttxt_desc;
					memcpy(&ttxt_desc,p,sizeof ttxt_desc); p+=sizeof ttxt_desc; len-=sizeof ttxt_desc;
					if (operation & PSI) 
						printf("Teletext data in language \"%.3s\" - PID 0x%04X, type %02X, page numbers (%d,%d)\n",
						 ttxt_desc.language, elementary_pid, ttxt_desc.teletext_type,
						 ttxt_desc.magazine_number, ttxt_desc.page_number);
				}
				break;
			case 0x59:
				while (len >= 8)
				{
					struct {
						byte language[3];
						byte subtitling_type;
						word composition_id;
						word ancillary_id;
					} sub_desc;
					memcpy(&sub_desc,p,sizeof sub_desc); p+=sizeof sub_desc; len-=sizeof sub_desc;
					NETWORD(sub_desc.composition_id); NETWORD(sub_desc.ancillary_id);
					if (operation & PSI) 
						printf("Subtitles in language \"%.3s\" - PID 0x%04X, type %02X, page numbers (%d,%d)\n",
						 sub_desc.language, elementary_pid, sub_desc.subtitling_type,
						 sub_desc.composition_id,sub_desc.ancillary_id);
					if (!language || !memcmp(language,sub_desc.language,3))
					{
						sub_pid = elementary_pid;
						composition_id = sub_desc.composition_id;
						ancillary_id = sub_desc.ancillary_id;
					}
				}
				break;
			default:
				verb(128, "Unhandled descriptor type: %02X\n", tag);
				p += len;
			}
		}		
		p += 4; // skip crc-32
		} break;
	}
}

void process_ts_packet(byte data[])
{
	byte *p = data;

	struct {
		byte sync_byte;
		word flags_pid;
		byte controls;
	} tp = { p[0], p[1] << 8 | p[2], p[3] }; p+=4;

	static int packet_index = 0;
	if (tp.sync_byte != 0x47)
	{
		verb(64,"TS packet # %d has sync byte 0x%02X != 0x47\n",packet_index++,tp.sync_byte);
		return;
	}
	verb(64,"TS packet # %d : error=%d, payload_start=%d, priority=%d\n",
	 packet_index++, (tp.flags_pid & 0x8000) != 0, (tp.flags_pid & 0x4000) != 0, (tp.flags_pid & 0x2000) != 0);
	verb(64,"  PID=0x%X, scrambling=%d, adapt=%d, counter=%d\n",
	 tp.flags_pid & 0x1FFF, tp.controls >> 6, (tp.controls >> 4)&3, tp.controls &0xF);
	if ((tp.flags_pid & 0x1FFF) == 0x1FFF) 
	{
		verb(32,"TS: null packet, skipping\n"); return;
    }
	if (tp.controls & 0x30 == 0) { verb(64,"TS packet has no payload or adaptation field\n"); return; }

	int af_len = 0;
	if ((tp.controls & 0x20) && (af_len = *(p++) + 1) > 1)
	{
		struct {
			byte flags;

			qword pcr_base;
			word pcr_ext;

			qword opcr_base;
			word opcr_ext;

			byte splice_countdown;

			byte private_length;
			byte *private_data;

			byte ext_length;
			byte ext_flags;

			word ltw;

			dword piecewise;

			byte splice_type;
			qword dts_next_au;
		} af = { *(p++) };

		verb(64, "TS adaptation field (len=%d): \n", af_len);
		if (af.flags&0x10)
		{
			af.pcr_base = *(p++) << 25 | *(p++) << 17 | *(p++) << 9 | *(p++) << 1;
			af.pcr_ext = *(p++); af.pcr_base |= (af.pcr_ext & 0x80 != 0);
			af.pcr_ext = (af.pcr_ext<<8 | *(p++)) & 0x1FF;
			float s=af.pcr_base/90000.0;
           	verb(64,"TS: PCR for PID 0x%04X: %02d:%02d:%02d.%02d\n",
			 tp.flags_pid & 0x1FFF,(int) s/3600,(int) (s/60) % 60,
			 (int) s % 60,(int) ((s-((int) s))*100));
		}
		if (af.flags & 0x08)
		{
			af.opcr_base = *(p++) << 25 | *(p++) << 17 | *(p++) << 9 | *(p++) << 1;
			af.opcr_ext = *(p++); af.opcr_base |= (af.opcr_ext & 0x80 != 0);
			af.opcr_ext = (af.opcr_ext<<8 | *(p++)) & 0x1FF;
             float s=af.opcr_base/90000.0;
             verb(64,"TS: Original PCR for PID 0x%04X: %02d:%02d:%02d.%02d\n",
			 tp.flags_pid & 0x1FFF, (int) s/3600,(int) (s/60) % 60,
			 (int) s % 60,(int) ((s-((int) s))*100));
		}
		if (af.flags & 0x04)
		{
			af.splice_countdown = *(p++);
			verb(64,"splice_countdown=%d, ", af.splice_countdown);
		}
		if (af.flags & 0x02)
		{
			af.private_length = *(p++);
			af.private_data = malloc(af.private_length);
			memcpy(af.private_data,p,af.private_length); p+=af.private_length;
			verb(64,"private data (%d bytes), ",af.private_length);
		}
		if (af.flags & 0x01)
		{
			af.ext_length = *(p++);
			af.ext_flags = *(p++);
			if (af.ext_flags & 0x80)
			{
				af.ltw = *(p++) << 8 | *(p++);
				verb(64,"ltw=%d, ", af.ltw);
			}
			if (af.ext_flags & 0x40)
			{
				af.piecewise = (*(p++) << 16 | *(p++) << 8 | *(p++)) & 0x3FFFFF;
				verb(64,"piecewise=%d, ", af.piecewise);
			}
			if (af.ext_flags & 0x20)
			{
				af.dts_next_au = *(p++); af.splice_type = af.dts_next_au >> 4;
				af.dts_next_au = (af.dts_next_au & 0x0E) << 29 | *(p++) << 22;
				af.dts_next_au |= (*(p++) & 0xFE) << 14 | *(p++) << 7 | *(p++) >> 1;
				verb(64,"splice_type=%d, dts=0x%X", af.splice_type, af.dts_next_au);
			}
		}
		verb(64,"\n");
	}

	if (! (tp.controls & 0x10)) { verb(64,"TS packet has no payload\n"); return; }

	p = data + 4 + af_len;
	verb(64,"TS: %d bytes of data for PID 0x%X: ",
	 188 - (p-data),tp.flags_pid & 0x1FFF);
	for (int j=0; j<8; j++) verb(64,"%02X ",p[j]); verb(64,"\n");
	verb(64,"pmt_pid=%04X, sub_pid=%04X, video_pid=%04X\n",pmt_pid,sub_pid,video_pid);

	// assemble and parse complete PES packets for subtitle data
	if ((tp.flags_pid & 0x1FFF) == sub_pid)
		process_subtitle_pes_chunk(p,188-(p-data),tp.controls & 0xF);

	// parse first chunk of each video ES packet, until we have first_video_pts
	else if ((tp.flags_pid & 0x1FFF) == video_pid)
	{
		if (first_video_pts != -1) return;	// we already established 1st video pts
		if (tp.flags_pid & 0x4000 == 0) return; // no payload_unit_start indication
		process_pes_packet(p);
	}

	// assemble and parse complete PAT packets until we have established PMT-PID
	// then a.&p. complete PMT packets until we have established a subtitle PID
	else if (sub_pid == 0xFFFF)
	{
		// ensure that packets of only one pid (i.e. pat or pmt) are processed here
		if ((tp.flags_pid & 0x1FFF) != pmt_pid && (pmt_pid != 0xFFFF
		 || (tp.flags_pid & 0x1FFF) != 0 /* ==pat_pid */ ))
			return;

		static byte *psi = NULL;
		static size_t psi_len = 0, psi_complete_len = -1;

/*		static byte psi_counter = 16;
		if (psi_counter == (tp.controls & 0xF))
		{	verb(32,"TS: duplicate PSI packet\n"); return; }
		psi_counter=tp.controls & 0xF;
*/
		// if we have no current section nor is there one starting now, quit
		if (psi_complete_len == -1 && !(tp.flags_pid & 0x4000)) return;

		// if we have no current section but another one is starting, seek to that
		if (psi_complete_len == -1)
		{
			verb(64,"TS: no current section (pointer %02X)\n",*p);
			p += *p + 1;
		}

		// if there's both a current section and a new one, process the old one now
		else if (tp.flags_pid & 0x4000) 
		{
			verb(64,"TS: parsing current, then new sections (pointer %02X)\n",*p);
			psi = realloc(psi, psi_len + *p);
			memcpy(psi+psi_len, p+1, *p); psi_len += *p; p += *p +1;
			if (psi_complete_len > psi_len) verb(1,"PSI underflow by %d\n",
				psi_complete_len - psi_len);
			process_psi_section(psi);
			free(psi); psi = NULL; psi_len = 0; psi_complete_len = -1;
		}

		// we have a current section to parse, no new one is set to begin
		else
		{
			verb(64,"TS: parsing current section (no pointer)\n");
			psi = realloc(psi, psi_len + 188-(p-data));
			memcpy(psi+psi_len, p, 188-(p-data)); psi_len += 188-(p-data);
			if (psi_len >= psi_complete_len)
			{
				process_psi_section(psi);
				free(psi); psi = NULL; psi_len = 0; psi_complete_len = -1;
			}
			return;
		}
		
		// process each section (or portion thereof) beginning in this packet
		while (p < data+188)
		{
				psi_complete_len = 3 + ((((word) p[1])<<8 | p[2]) & 0x0FFF);
				verb(64,"TS: parsing new section w/ len %04X\n",psi_complete_len);
				if (p + psi_complete_len > data+188)
				{
					psi_len = 188-(p-data); psi = malloc(psi_len);
					memcpy(psi,p,psi_len); break;
				}
				process_psi_section(p); 
				p += psi_complete_len; psi_complete_len = -1;
				if ((tp.flags_pid & 0x1FFF) != pmt_pid && *p != 0 ||
				 (tp.flags_pid & 0x1FFF) == pmt_pid && *p != 2)	break;
		}
	}
}

int main(int argc, char *argv[])
{
	FILE *input = NULL;

	dotsub = dotidx = NULL;
	for (int i=1; i < argc; i++)
		if (!strcmp(argv[i],"-h"))
		{
			fprintf(stderr,"vdrsub [-h] [-d ss.ss] [-vdr][-ts] [lähtötiedosto]\n");
			fprintf(stderr,"(Antti Hautaniemi 2011-12)\n\n");
			fprintf(stderr,"Muuntaa vdr-nauhoitustiedoston (.vdr tai .ts) sisältämän tai oletussyötteestä\n");
			fprintf(stderr,"luetun tekstitysraidan VobSub-muotoon .sub- ja .idx-tiedostoksi\n");
			fprintf(stderr," -h   apua\n");
			fprintf(stderr," -d   aseta videoraidan aloitus-PTS sekunteina (luetaan automaattisesti)\n");
			fprintf(stderr," -vdr aseta lähtötiedoston tyypiksi .vdr (oletus .vdr-päätteiselle tiedostolle)\n");
			fprintf(stderr," -ts  aseta lähtötiedoston tyypiksi .ts (oletus muutoin)\n");
			return 0;
		}
		else if (!strcmp(argv[i],"-d"))
			first_video_pts = (i <= argc-2) ? atof(argv[++i])*90000 : 0;
		else if (!strcmp(argv[i],"-vdr"))
			input_type = VDR;
		else if (!strcmp(argv[i],"-ts"))
			input_type = TS;
		else if (!strcmp(argv[i],"-psi"))
			operation = PSI;
		else if (argv[i][0] != '-')
		{
			char *name = (char *) malloc(strlen(argv[i])+4), *ext;
			strcpy(name,argv[i]); 
			if ((input = fopen(name,"rb")) == NULL) 
			{ fprintf(stderr,"Unable to open: %s\n",name); continue; }
			if (operation & CONVERT)
			{
				ext = (ext=strrchr(name,'.')) != NULL ? ext : name+strlen(name);
				if (!strcmp(ext,".vdr")) 
					input_type = VDR;
				strcpy(ext,".sub"); dotsub = fopen(name,"wb");
				strcpy(ext,".idx"); dotidx = fopen(name,"w");
			}
		}

	if (input == NULL)
	{	input = stdin;
		if (operation & CONVERT)
		{ dotsub = fopen("out.sub","wb"); dotidx = fopen("out.idx","w"); }
	}
	if (operation & CONVERT)
	{
		if (dotsub == NULL || dotidx == NULL)
		{ fprintf(stderr,"Unable to write .sub and/or .idx file\n"); return 1; }

		fputs("# VobSub index file, v7 (do not modify this line!)\n",dotidx);
		fputs("palette: 000000, 131313, 868686, D6D6D6, ",dotidx);
		fputs("000000, 000000, 000000, 000000, 000000, 000000, ",dotidx);
		fputs("000000, 000000, 000000, 000000, 000000, 000000\n",dotidx);
		fputs("\nid: fi, index: 0\n",dotidx);
	}
	subp = init_subp();
	init_verbose(1);

	switch (input_type)
	{
	case TS: {
		byte ts_packet[188];
		int sync_byte;
		while (1)
		{
			do if ((sync_byte = fgetc(input)) == EOF) break;
			while (sync_byte != 0x47);
		 	if (fread(ts_packet+1,1,187,input) != 187)
				break;
			ts_packet[0] = sync_byte;
			process_ts_packet(ts_packet);
		}
		} break;
	case VDR: {
		byte *pes_packet = malloc(6);
		while (1)
		{
			if (fread(pes_packet,1,6,input) < 6)
				break;
			word pes_length = (((word) pes_packet[4]) << 8) + pes_packet[5];
			if ((pes_packet = realloc(pes_packet, 6+pes_length)) == NULL)
			{ fprintf(stderr, "Out of memory, requested %d bytes\n",6+pes_length); return 2; }
			if (fread(pes_packet+6,1,pes_length,input) < pes_length)
				break;
			process_pes_packet(pes_packet);
		}
		free(pes_packet);
		if (pes_len > 0)
			// forward contents of any subtitle PES sequence remaining in the cache
			process_dvbsub_data(pes_data, pes_len, pes_pts);
		} break;
	}
				
	release_subp(subp);
	if (operation & CONVERT) { fclose(dotsub); fclose(dotidx); }
	if (input != stdin) fclose(input);
	return 0;
}
