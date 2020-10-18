/*
   MPEG header generation code written by
   Mike Matsnev <mike@po.cs.msu.su>.

   (in mkvtoolnix code : xtr_vobsub.cpp)
 */

#include <stdio.h>
#include <string.h>

typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned long dword;
typedef unsigned long long qword;

// Write vobsub packet in 'data' to 'fp' formatted into dvd-compatible ps packets
void write_vobsub_ps(byte *data, size_t size, qword pts, FILE *fp)
{
	static byte padding_data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	struct mpeg_es_header_t {
		byte pfx[3];               // 00 00 01
		byte stream_id;            // BD
		byte len[2];
		byte flags[2];
		byte hlen;
		byte pts[5];
		byte lidx;
	} es;
	memset(&es, 0, sizeof es);

	struct mpeg_ps_header_t {
		byte pfx[4];               // 00 00 01 BA
		byte scr[6];
		byte muxr[3];
		byte stlen;
	} ps;
	memset(&ps, 0, sizeof ps);

	dword padding = (2048 - (size + sizeof ps + sizeof es)) & 2047;
	dword first   = size + sizeof ps + sizeof es > 2048
					? 2048 - sizeof ps - sizeof es : size;

	ps.pfx[2]  = 0x01;
	ps.pfx[3]  = 0xba;
	ps.scr[0]  = 0x40 | ((byte)(pts >> 27) & 0x38) | 0x04 | ((byte)(pts >> 28) & 0x03);
	ps.scr[1]  = (byte)(pts >> 20);
	ps.scr[2]  = ((byte)(pts >> 12) & 0xf8) | 0x04 | ((byte)(pts >> 13) & 0x03);
	ps.scr[3]  = (byte)(pts >> 5);
	ps.scr[4]  = ((byte)(pts << 3) & 0xf8) | 0x04;
	ps.scr[5]  = 1;
	ps.muxr[0] = 1;
	ps.muxr[1] = 0x89;
	ps.muxr[2] = 0xc3; // just some value
	ps.stlen   = 0xf8;

	es.pfx[2]    = 1;
	es.stream_id = 0xbd;
	es.len[0]    = (byte)((first + 9) >> 8);
	es.len[1]    = (byte)(first + 9);
	es.flags[0]  = 0x81;
	es.flags[1]  = 0x80;
	es.hlen      = 5;
	es.pts[0]    = 0x20 | ((byte)(pts >> 29) & 0x0e) | 0x01;
	es.pts[1]    = (byte)(pts >> 22);
	es.pts[2]    = ((byte)(pts >> 14) & 0xfe) | 0x01;
	es.pts[3]    = (byte)(pts >> 7);
	es.pts[4]    = (byte)(pts << 1) | 0x01;
	es.lidx      = 0x20;
	if ((6 > padding) && (first == size))
	{
		es.hlen += (byte)padding;
		es.len[0]  = (byte)((first + 9 + padding) >> 8);
		es.len[1]  = (byte)(first + 9 + padding);
	}

	fwrite(&ps, 1, sizeof ps, fp);
	fwrite(&es, 1, sizeof es -1, fp);
	if ((0 < padding) && (6 > padding) && (first == size))
		fwrite(padding_data, 1, padding, fp);
  	fputc(es.lidx, fp);

	fwrite(data, 1, first, fp);
	while (first < size)
	{
		size    -= first;
		data    += first;

		padding  = (2048 - (size + 10 + sizeof ps)) & 2047;
		first    = size + 10 + sizeof ps > 2048 ? 2048 - 10 - sizeof ps : size;

		es.len[0]   = (byte)((first + 4) >> 8);
		es.len[1]   = (byte)(first + 4);
		es.flags[1] = 0;
		es.hlen     = 0;
		if ((6 > padding) && (first == size))
		{
			es.hlen += (byte)padding;
			es.len[0]  = (byte)((first + 4 + padding) >> 8);
			es.len[1]  = (byte)(first + 4 + padding);
		}

		fwrite(&ps, 1, sizeof ps, fp);
		fwrite(&es, 1, 9, fp);
		if ((0 < padding) && (6 > padding) && (first == size))
			fwrite(padding_data, 1, padding, fp);
		fputc(es.lidx, fp);
		fwrite(data, 1, first, fp);
	}
	if (6 <= padding)
	{
		padding      -= 6;
    	es.stream_id  = 0xbe;
    	es.len[0]     = (byte)(padding >> 8);
    	es.len[1]     = (byte)padding;

    	fwrite(&es, 1, 6, fp);

    	while (0 < padding)
		{
			dword todo = 8 < padding ? 8 : padding;
			fwrite(padding_data, 1, todo, fp);
			padding -= todo;
		}
	}
}
