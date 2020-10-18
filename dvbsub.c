/*
	A simple DVBSub decoder & VobSub encoder; see dvbsub.h
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <arpa/inet.h> // ntohs()

#include "dvbsub.h"

#define min(x,y) ((x)<(y) ? (x) : (y))
#define max(x,y) ((x)>(y) ? (x) : (y))

int verbose_level = 0;
void init_verbose(int level)
{
    char *verb = getenv("SUB_VERBOSE");
    if (verb != NULL) verbose_level=atoi(verb);
    else verbose_level = level;
}

#ifdef VERBOSE
void verb(int level, const char *format, ...)
{
	va_list args;
	va_start(args,format);

	if ((level & verbose_level) != level) return;
	vfprintf(stderr,format,args);
	va_end(args);
}

#else
#define verb(x,f,...)

#endif

// load a number of bits from data pointed to by *p and update *p as necessary
// during a bit-field, caller must maintain 'byte b, mask=0' and pass &b,&mask
//
qword loadbits(int bits, byte **p, byte *b, byte *mask)
{
	qword r = 0;

	if (bits == 0) *mask = 0;
	while (bits--)
	{
		if (! *mask)
		{
			*mask=0xFF; *b = *(p[0]++);
		}
		r <<= 1;
		r |= (((*b) & 0x80) != 0);
		*b <<= 1; *mask <<= 1;
	}
	return r;
}

#define smpl2(x) (x)
#define smpl4(x) (x)

void twobit_coding(byte **pix, byte **p)
{
    byte b,mask=0;

	while (1)
	{
		byte twobits = loadbits(2,p,&b,&mask);
		if (twobits) *((*pix)++) = smpl2(twobits);
		else if (loadbits(1,p,&b,&mask))
		{
			int i = loadbits(3,p,&b,&mask)+3;
			twobits = loadbits(2,p,&b,&mask);
			while (i--) *((*pix)++) = smpl2(twobits);
		}
		else if (loadbits(1,p,&b,&mask)) *((*pix)++) = smpl2(0);
		else switch (loadbits(2,p,&b,&mask))
		{
        case 0: loadbits(0,NULL,NULL,&mask); return;
        case 1: *((*pix)++) = smpl2(0); *((*pix)++) = smpl2(0); break;
        case 2: {
                int i = loadbits(4,p,&b,&mask)+12;
            	twobits = loadbits(2,p,&b,&mask);
            	while (i--) *((*pix)++) = smpl2(twobits);
				break;
                }
        case 3: {
                int i = loadbits(8,p,&b,&mask)+29;
            	twobits = loadbits(2,p,&b,&mask);
            	while (i--) *((*pix)++) = smpl2(twobits);
				break;
                }
		}
	}
}

void fourbit_coding(byte **pix, byte **p)
{
    byte b,mask=0;

	while (1)
	{
		byte fourbits = loadbits(4,p,&b,&mask);
		if (fourbits) *((*pix)++) = smpl4(fourbits);
		else if (!loadbits(1,p,&b,&mask))
		{
			int i = loadbits(3,p,&b,&mask)+2;
			if (i==2) { loadbits(0,NULL,NULL,&mask); return; }
			while (i--) *((*pix)++) = smpl4(0);
		}
		else if (!loadbits(1,p,&b,&mask))
		{
			int i = loadbits(2,p,&b,&mask)+4;
            fourbits = loadbits(4,p,&b,&mask);
            while (i--) *((*pix)++) = smpl4(fourbits);
		}
		else switch (loadbits(2,p,&b,&mask))
		{
		case 0: *((*pix)++) = smpl4(0); break;
        case 1: *((*pix)++) = smpl4(0); *((*pix)++) = smpl4(0); break;
        case 2: {
                int i = loadbits(4,p,&b,&mask)+9;
            	fourbits = loadbits(4,p,&b,&mask);
            	while (i--) *((*pix)++) = smpl4(fourbits);
				break;
                }
        case 3: {
                int i = loadbits(8,p,&b,&mask)+25;
            	fourbits = loadbits(4,p,&b,&mask);
            	while (i--) *((*pix)++) = smpl4(fourbits);
				break;
                }
        }
	}
}

void eightbit_coding(byte **pix, byte **p)
{
    byte b,mask=0;

	while (1)
	{
		byte eightbits = loadbits(8,p,&b,&mask);
		if (eightbits) *((*pix)++) = eightbits;
		else if (!loadbits(1,p,&b,&mask))
		{
			int i = loadbits(7,p,&b,&mask)+1;
			if (i==1) { loadbits(0,NULL,NULL,&mask); return; }
			while (i--) *((*pix)++) = 0;
		}
		else
		{
			int i = loadbits(7,p,&b,&mask)+3;
            eightbits = loadbits(8,p,&b,&mask);
            while (i--) *((*pix)++) = eightbits;
		}
	}
}

struct region { int x,y, w,h; };
struct object { int x,y; int r; };

// define the actual context data (pointed to by subpicture.ctx) used
// by decode_dvbsub to keep constant state between received data frames
typedef struct {
    // regions, i.e. on-screen rectangles
    struct region *r;
    // objects, i.e. positions within regions
    struct object *o;
    size_t r_n, o_n;                // currently received count of the above
} dvb_ctx;

subpicture init_subp(void)
{
	subpicture subp = (subpicture) {
        NONE,                       // live_state
        9999,9999, 0,0,             // x,y,w,h (enclosing rectangle)
        NULL,                       // pix (decoded pixel data)
        {{0,0,0}},                  // clut
        malloc(sizeof (dvb_ctx))    // ctx
    };
    *((dvb_ctx *) subp.ctx) = (dvb_ctx) { NULL, NULL, 0,0 };
    return subp;
}

void release_subp(subpicture subp)
{
    if (subp.pix != NULL)
        free(subp.pix);
    if (((dvb_ctx *) subp.ctx)->r != NULL)
        free(((dvb_ctx *) subp.ctx)->r);
    if (((dvb_ctx *) subp.ctx)->o != NULL)
        free(((dvb_ctx *) subp.ctx)->o);
    free(subp.ctx);
}

// take word x in network (bigendian) order, as read from the dvb stream
// if necessary, adjust x to (a little-endian) host byte order
#define NETWORD(x) (x = (word) ntohs(x))

void decode_dvbsub(byte *data, size_t len, subpicture *dst)
{
    verb(2,"DVBSUB decoding frame with %d bytes :\n",len);
    // Assume any previous draw or wipe operation be committed by the caller
    if (dst->live_state == WIPE)
        dst->live_state = NONE;
    else if (dst->live_state == DRAW)
        dst->live_state = STAY;

    dvb_ctx *ctx = (dvb_ctx *) dst->ctx;
    byte *p = data;

	while (p < data + len)
	{
        struct {
            byte sync_byte;
            byte segment_type;
            word page_id;
            word segment_length;
        } subseg;
		memcpy(&subseg,p,sizeof subseg); //p += sizeof subseg;
		NETWORD(subseg.page_id); NETWORD(subseg.segment_length);

        if (subseg.sync_byte != 0x0F)
		{
			verb(1,"Subtitling segment sync byte 0x%02X != 0x0F\n",
			 subseg.sync_byte);
			break;
		}
        verb(2,"DVBSUB: %d bytes of payload left, next segment takes (%d+)%d\n",
             len - (p-data),sizeof subseg,subseg.segment_length);
		p += sizeof subseg;

		switch (subseg.segment_type)
		{
		case 0x10: verb(2," page composition segment\n");
			{
                struct {
                    byte page_time_out;
                    byte page_ver_state; // 4 + 2 (+2) bits
                } pageseg;
				memcpy(&pageseg,p,2); p+=2;

				verb(2,"  time_out=%ds, version=%d, state = ",
				 pageseg.page_time_out,pageseg.page_ver_state >> 4);
				switch ((pageseg.page_ver_state & 0x0C) >> 2)
				{
					case 0: verb(2,"normal case, "); break;
					case 1: verb(2,"acquisition point, "); break;
					case 2: verb(2,"mode change, "); break;
					case 3: verb(2,"reserved, "); break;
				}
                // acquisition point & mode change signal clearing of old page
                if (pageseg.page_ver_state & 0x0C)
                {
                    if (dst->live_state == STAY)
                        dst->live_state = WIPE;
                    // delete old region + object definitions
                    if (ctx->r != NULL) free(ctx->r); ctx->r=NULL;ctx->r_n = 0;
                    if (ctx->o != NULL) free(ctx->o); ctx->o=NULL;ctx->o_n = 0;
                    // enclosing rectangle needs to be calculated anew
                    dst->x = dst->y = 9999; dst->w = dst->h = 0;
					if (dst->pix != NULL) { free(dst->pix); dst->pix = NULL; }
                }

				verb(2," regions:\n");

                byte *endp = p+subseg.segment_length-2;
                while (p < endp)
				{
                    struct {
                        byte region_id;
                        byte reserved;
                        word region_horizontal_address;
                        word region_vertical_address;
                    } reg;
                    memcpy(&reg,p,6); p+=6;
					NETWORD(reg.region_horizontal_address);
					NETWORD(reg.region_vertical_address);

                    if (ctx->r_n <= reg.region_id)
                    {
                        ctx->r_n = reg.region_id+1;
                        ctx->r = (struct region *) realloc(ctx->r,
                         sizeof (struct region) * ctx->r_n);
                    }
                    // store region position into context data
                    ctx->r[reg.region_id] = (struct region) {
					 reg.region_horizontal_address,
					 reg.region_vertical_address, 0,0 };

                    // also update the enclosing rectangle position
                    dst->x = min( dst->x, reg.region_horizontal_address);
                    dst->y = min( dst->y, reg.region_vertical_address);

                    verb(2, "  - region_id=%d, hor_addr=%d, ver_addr=%d\n",
					 reg.region_id,
					 reg.region_horizontal_address,
					 reg.region_vertical_address);
				}
			}
			break;
		case 0x11: verb(2," region composition segment ");
			{
                struct {
                    byte region_id;
                    byte reg_ver_fillflag;
                    word region_width;
                    word region_height;
                    byte reg_compat_depth;
                    byte clut_id;
                    byte region_8bit;
                    byte region_4_2bit;
                } regseg;
				memcpy(&regseg,p,10); p+=10;
				NETWORD(regseg.region_width);
				NETWORD(regseg.region_height);

                verb(2,"for region %d :\n",regseg.region_id);
                if (ctx->r_n <= regseg.region_id)
                {
                    verb(1,"ERROR: Composing undeclared region\n");
                    p += subseg.segment_length-10; break;
                }
                // store region size into context data
				ctx->r[regseg.region_id].w = regseg.region_width;
				ctx->r[regseg.region_id].h = regseg.region_height;

                // make this region also fit inside the enclosing rectangle
                int new_w = ctx->r[regseg.region_id].x
                        +ctx->r[regseg.region_id].w - dst->x;
                int new_h = ctx->r[regseg.region_id].y
                        +ctx->r[regseg.region_id].h - dst->y;
                if (dst->w < new_w || dst->h < new_h)
                {
                    dst->w = max( dst->w, new_w );
                    dst->h = max( dst->h, new_h );
                    dst->pix = (byte *) realloc(dst->pix, dst->w*dst->h);
					memset(dst->pix, 0, dst->w * dst->h);
                }

				verb(2,"  ver=%d, fill=%d, width=%d, height=%d, compatibility=%d\n",
				 regseg.reg_ver_fillflag >> 4,
				 (regseg.reg_ver_fillflag & 0x08) != 0,regseg.region_width,
				 regseg.region_height,regseg.reg_compat_depth >> 5);
				verb(2,"  depth=%d, clut_id=%d, codes=0x%X 0x%X 0x%X, objects :\n",
				 (regseg.reg_compat_depth >> 2) & 0x07,
				 regseg.clut_id,regseg.region_8bit,regseg.region_4_2bit >> 4,
				 (regseg.region_4_2bit >> 2) & 0x03);

				byte *endp = p+subseg.segment_length-10;
                while (p < endp)
				{
                    struct {
                        word object_id;
                        word obj_type_prov_horiz;   // 2 + 2 + 12 bits
                        word obj_vertical;          // (4+) 12 bits
                        byte foreground_pixel_code;
                        byte background_pixel_code;
                    } obj;
					memcpy(&obj,p,6); p+=6;
					NETWORD(obj.object_id);
					NETWORD(obj.obj_type_prov_horiz);
					NETWORD(obj.obj_vertical);

                    // store object pos. + referenced region into context data
                    if (ctx->o_n <= obj.object_id)
                    {
                        ctx->o_n = obj.object_id+1;
                        ctx->o = (struct object *) realloc(ctx->o,
                            sizeof (struct object) * ctx->o_n);
                    }
					ctx->o[obj.object_id] = (struct object) {
                        obj.obj_type_prov_horiz & 0xFFF,
                        obj.obj_vertical & 0xFFF,
                        regseg.region_id };

					verb(2,"  - id=%d, type&prov=0x%X, hor_pos=%d, ver_pos=%d\n",
					 obj.object_id,
					 obj.obj_type_prov_horiz >> 12,
					 obj.obj_type_prov_horiz & 0x0FFF,
					 obj.obj_vertical & 0x0FFF);
					switch (obj.obj_type_prov_horiz & 3)
					{
					case 1: case 2:
						memcpy(&obj.foreground_pixel_code,p,2);
						p+=2;
						verb(2,"   fg_pixel_code=%d, bg_pixel_code=%d\n",
						 obj.foreground_pixel_code,
						 obj.background_pixel_code);
					}
				}
			}
			break;
		case 0x12: verb(2," CLUT composition segment ");
			{
                struct {
                    byte clut_id;
                    byte clut_version;
                } clutseg;
				memcpy(&clutseg,p,2); p+=2;

                verb(2,"id=%d, version=%d, entries:\n",clutseg.clut_id,
				 clutseg.clut_version);

                // clear clut, so undefined entries get signalled by y=0
                memset(dst->clut, 0, sizeof dst->clut);

				byte *endp = p+subseg.segment_length-2;
                while (p < endp)
				{
                    struct {
                        byte entry_id;
                        byte flags;
                        byte y, cr, cb, t;
                    } entry;
					memcpy(&entry,p,2); p+=2;

					if (entry.flags & 1)
					{
						memcpy(&entry.y,p,4); p+=4;
					}
					else
					{
                        byte b,mask=0; // for loadbits()
                        entry.y = loadbits(6,&p,&b,&mask) << 2;
                        entry.cr = loadbits(4,&p,&b,&mask) << 4;
                        entry.cb = loadbits(4,&p,&b,&mask) << 4;
                        entry.t = loadbits(2,&p,&b,&mask) << 6;
					}
                    dst->clut[entry.entry_id] = (struct colour)
					 { entry.y? entry.y :1, entry.cr, entry.cb, entry.t };

					verb(2,"  - %X, flags=0x%X, y=0x%02X, cr=0x%02X, cb=0x%02X, t=0x%02X\n", 
					 entry.entry_id,entry.flags,entry.y, entry.cr, entry.cb, entry.t);
				}
			}
			break;
		case 0x13: verb(2," object data segment ");
			{
                struct {
                    word object_id;
                    byte obj_ver_code_colour;
                    word top_length;
                    word bottom_length;
                    struct {
                        byte *pix;
                        int height;
                        byte map_table[16];
                    } top, bottom;
                    byte code_count;
                    word *characters;
                } objseg;
				memcpy(&objseg,p,3); p+=3;
				NETWORD(objseg.object_id);

                if (objseg.object_id >= ctx->o_n)
                {
                    verb(1,"ERROR: Receiving data for undeclared object %d\n",
					 objseg.object_id);
                    p += subseg.segment_length-3; break;
                }
                struct region reg = ctx->r[ctx->o[objseg.object_id].r];
                reg.x += ctx->o[objseg.object_id].x - dst->x;
                reg.y += ctx->o[objseg.object_id].y - dst->y;
				// reg now has x,y relative to our enclosing rectangle

				verb(2,"for obj %d, version=%d, coding=%d, colour=%d\n",
				 objseg.object_id,objseg.obj_ver_code_colour >> 4,
				 (objseg.obj_ver_code_colour >> 2) & 3,
				 (objseg.obj_ver_code_colour >> 1) & 1);
				if ((objseg.obj_ver_code_colour >> 2) & 3)
				{
                    verb(1,
                     "Object coding is not run-length image data, skipping!\n");
                    p += subseg.segment_length-3;
                    break;
                }

				memcpy(&objseg.top_length,p,4); p+=4;
				NETWORD(objseg.top_length);
                NETWORD(objseg.bottom_length);

                if (objseg.top_length+objseg.bottom_length >
                 subseg.segment_length-7)
                {
                    verb(1,"ERROR: top+bottom=%d > %d=segment-7\n",
                     objseg.top_length+objseg.bottom_length,subseg.
                     segment_length-7);
                    return;
                }
                verb(2,"  top=%d,bottom=%d, top+bottom=%d\n",objseg.top_length,
                 objseg.bottom_length,objseg.top_length+objseg.bottom_length);
                verb(2,"  decoding pixel data to (%d,%d), size %d x %d\n",
					reg.x,reg.y,reg.w,reg.h);

                int row = 0;
                byte *pix = dst->pix + reg.y * dst->w + reg.x;
                byte *endp = p+objseg.top_length;
                while (p < endp)
                {
                    byte type = *(p++);
                    switch (type)
                    {
                    case 0x10: twobit_coding(&pix,&p); break;
                    case 0x11: fourbit_coding(&pix,&p); break;
                    case 0x12: eightbit_coding(&pix,&p); break;
                    case 0x20: p+=2; break;
                    case 0x21: p+=4; break;
                    case 0x22: p+=16; break;
                    case 0xF0:
                        pix = dst->pix + (reg.y + (row+=2)) * dst->w + reg.x;
                        break;
                    default: verb(1,"^%02X",type); break;
                    }
                }
                if (p > endp)
                {
                    verb(1,"ERROR: Top field overflow by %d bytes\n",p-endp);
                    return;
                }
                row = 1;
                pix = dst->pix + (reg.y +1) * dst->w + reg.x;
                endp = p+objseg.bottom_length;
                while (p < endp)
                {
                    byte type = *(p++);
                    switch (type)
                    {
                    case 0x10: twobit_coding(&pix,&p); break;
                    case 0x11: fourbit_coding(&pix,&p); break;
                    case 0x12: eightbit_coding(&pix,&p); break;
                    case 0x20: p+=2; break;
                    case 0x21: p+=4; break;
                    case 0x22: p+=16; break;
                    case 0xF0:
                        pix = dst->pix + (reg.y + (row+=2)) * dst->w + reg.x;
                        break;
                    default: verb(1,"_%02X",type); break;
                    }
                }
                if (p > endp)
                {
                    verb(1,"ERROR: Bottom field overflow by %d bytes\n",p-endp);
                    return;
                }
                // step over possible word alignment byte
                if (subseg.segment_length > 7+objseg.top_length
                 +objseg.bottom_length) p++;
            break; }
		case 0x14: verb(2,"display definition segment found, skipping\n");
			p += subseg.segment_length; break;
        case 0x80: {
			verb(2," end of display set segment (subpicture: %d x %d)\n",
			 dst->w,dst->h);
			dst->live_state = DRAW;   // e.o.d.s. signals the end of a subpicture definition
            break; }
		case 0xFF: verb(2," stuffing segment\n"); break;
		default: if (subseg.segment_type >= 0x81 && subseg.segment_type <= 0xEF)
			verb(2," private data segment\n");
			else verb(2,"segment type 0x%X, reserved for future use\n",subseg.segment_type);
		}
	}
}

#ifdef VOBSUB

/////////////////////////////////
//
// VobSub encoder model :

byte * encode_rle_row(int w,byte *input,byte clut[],byte *p)
{
#define storenb(p,n) ((subix^=1) ? (*(p) = ((n)<<4)) : \
                                    (*((p)++) |= ((byte) (n))&0xF))
#define alignnb(p) ((subix) ? ++(p) + ((subix)=0) : p)

    // VobSub (and DVD) subtitles use the following encode sequences
    // (let one digit signify two bits and 'c' a palette index to be encoded) :
    // - special sequence 00 00 00 0c signals 'c till the end of the line'
    // - for 'c into klmn pixels' (with 'klmn' within 1-255) the sequences are
    // for j=klmn (in 64-255):  00 0k [lm] (nc)
    // for j= lmn (in 16-63):      00 [lm] (nc)
    // for j=  mn (in 4-15):           0m  (nc)
    // for j=   n (in 1-3):                 nc

    int i, j, subix=0;

    for (i=0; i < w; i += j)
    {
		byte c = clut[input[i]];
        for (j=1; i+j < w; j++)
			if (clut[input[i+j]] != c) break;	// j=length
        if (i+j == w && j > 63)
        {
            if (subix)  *(++p) = ( *(++p) = 0 ) | c << 4;
            else        *(p++) = ( *(p++) = 0 ) | c;
			break;                              // end of the line, exit
        }
		if (j > 255) j=255;
        if (j > 15) storenb(p,0);    // store the leading '00' nibble
        if (j > 63) storenb(p,j>>6); // store the '0k' nibble
        if (j > 3)  storenb(p,j>>2); // store the '[lm]' or '0m' nibble
                    storenb(p,j<<2 | c); // store the '(nc)' or 'nc' nibble
    }
    return alignnb(p);
}

void encode_vobsub(byte *data, subpicture *src)
{
	byte *p = data + 4, clut[256] = {0};
	int i;

    // map src->clut by 'y' value into greyscale table { 0, 0x13, 0x86, 0xD6 }
    for (i=1; i < 256 && src->clut[i].y; i++)
        clut[i] = src->clut[i].y < 0x50 ? 1
         : src->clut[i].y < 0xBB ? 2 : 3;

    for(i=0; i < src->h; i+=2)
		p = encode_rle_row(src->w, src->pix + i * src->w, clut, p);

    int bottom_ptr = p - data;
    for(i=1; i < src->h; i+=2)
		p = encode_rle_row(src->w, src->pix + i * src->w, clut, p);
    data[2] = (p-data) >> 8; data[3] = p-data;

	byte dcsq[] = {
		0,0,										// delay=0
		data[2],data[3],                            // pointer to self
		1,											// start display
		3, 0x32, 0x10,								// set palette indices
		4, 0xFF, 0xF0,								// set alpha values
		5,											// set display area [bits]:
		src->x>>4,                                  // left [11-4]
        src->x<<4 | (((src->x+src->w-1)>>8) & 0xF), // left [3-0], right [11-8]
        src->x+src->w-1,                            // right [7-0]
		src->y>>4,                                  // top [11-4]
        src->y<<4 | (((src->y+src->h-1)>>8) & 0xF), // top[3-0], bottom [11-8]
        src->y+src->h-1,                            // bottom [7-0]
		6, 0,4, bottom_ptr >> 8,bottom_ptr,			// set rle addresses
		0xFF, 0xFF									// end command seq.
    };
	memcpy(p, dcsq, sizeof dcsq);
	size_t len = (p + sizeof dcsq - data) & 0xFFFE;
	data[0] = len >> 8; data[1] = len;
}

byte *dvb2vobsub_translation(byte *data,size_t len,subpicture *ctx)
{
	decode_dvbsub(data,len,ctx);

    switch (ctx->live_state)
	{
        case NONE: case STAY: return NULL;      // No operation right now
        case DRAW:                      // New subpicture to display
            data = (byte *) malloc(ctx->w*ctx->h + 25); // Maximum sub size?
            encode_vobsub(data,ctx);
            return data;
        case WIPE:                      // Wipe off prev. picture at this PTS
        {
            const byte spu_stop_packet[] = {
                0,24,               // #  0  size of this packet
                0,5,                // #  2  DCSQ address
                0x40,               // #  4  run-length for 1 pixel of colour 0
                0,0,                // #  5  delay: 0
                0,5,                // #  7  DCSQ address again (last block)
                0x01,               // #  9  start display (VLC wants this)
                0x02,               // # 10  stop display
                0x05, 0,0,0,0,0,0,  // # 11  position & size
                0x06, 0,4,0,4,      // # 18  offset to top & bottom field
                0xFF                // # 23  quit
            };
            data = (byte *) malloc(sizeof spu_stop_packet);
            memcpy(data,spu_stop_packet,sizeof spu_stop_packet);
            return data;
        }
    }
	verb(1,"ERROR: DVBsub decoder has unspecified state %d\n",
     ctx->live_state);
    return NULL;
}

#endif
