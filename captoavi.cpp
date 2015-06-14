#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "alfe/main.h"
#include "alfe/complex.h"
#include "alfe/evaluate.h"
#include "alfe/bitmap.h"
#include "alfe/ntsc_decode.h"

#ifdef WIN32
#define ZLIB_WINAPI
#endif
#include "zlib.h"

#define CODEC_4CC "ZMBV"

#define MAX_VECTOR	16

#define Mask_KeyFrame			0x01

#define WAVE_BUF 16*1024
#define AVI_HEADER_SIZE	500

static void host_writed(UInt8* off,UInt32 val)
{
    off[0]=(UInt8)(val);
    off[1]=(UInt8)(val >> 8);
    off[2]=(UInt8)(val >> 16);
    off[3]=(UInt8)(val >> 24);
}

static void host_writew(UInt8* off,UInt16 val)
{
    off[0]=(UInt8)(val);
    off[1]=(UInt8)(val >> 8);
}

class Program : public ProgramBase
{
public:
    void run()
    {
        bool doDecode = false;

        static const int samples = 450*1024;
        static const int inputBufferSize = samples;
        static const int sampleSpaceBefore = 256;
        static const int sampleSpaceAfter = 256;

        FileHandle in = File("captured.zdr", true).openRead();
        UInt64 inputFileSizeRemaining = in.size();
        Array<Byte> inputBuffer(inputBufferSize);
        int inputBufferRemaining = 0;
        Byte* inputPointer = 0;
        z_stream zs;
        memset(&zs, 0, sizeof(z_stream));
        if (inflateInit(&zs) != Z_OK)
            throw Exception("inflateInit failed");

        Array<Byte> buffer(sampleSpaceBefore + samples + sampleSpaceAfter);
        Byte* b = &buffer[0] + sampleSpaceBefore;
        for (int i = 0; i < sampleSpaceBefore; ++i)
            b[i - sampleSpaceBefore] = 0;
        for (int i = 0; i < sampleSpaceAfter; ++i)
            b[i + samples] = 0;
        int outputBytesRemaining = samples;

        Vector outputSize;
        NTSCCaptureDecoder<UInt32> decoder;
         
        if (doDecode) 
            outputSize = Vector(1280, 720);
        else
            outputSize = Vector(1824, 253);
        Bitmap<UInt32> decoded(outputSize);
        if (doDecode)
            decoder.setOutputBuffer(
                decoded.subBitmap(Vector(160, 0), Vector(960, 720)));
        else
            decoder.setOutputBuffer(decoded);
        decoded.fill(0);
        decoder.setInputBuffer(b);
        decoder.setOutputPixelsPerLine(1140);
        decoder.setYScale(3);
        decoder.setDoDecode(doDecode);

        _handle = fopen("u:\\captured2.avi","wb");
        if (!_handle)
            throw Exception("Can't open file");

        _VectorCount = 1;
        _VectorTable[0].x = _VectorTable[0].y = 0;
        for (int s = 1; s <= 10; ++s) {
            for (int y = -s; y <= s; ++y)
                for (int x = -s; x <= s; ++x) {
                    if (abs(x) == s || abs(y) == s) {
                        _VectorTable[_VectorCount].x = x;
                        _VectorTable[_VectorCount].y = y;
                        ++_VectorCount;
                    }
                }
        }
        memset(&_zstream, 0, sizeof(_zstream));

        _pitch = outputSize.x + 2*MAX_VECTOR;
        if (deflateInit(&_zstream, 4) != Z_OK)
            throw Exception("deflateInit failed");

        _bufSize = 4*outputSize.x*outputSize.y + 2*(1+(outputSize.x/8)) * (1+(outputSize.y/8))+1024;
        _bufSize += _bufSize / 1000;

        _buf = malloc(_bufSize);
        if (!_buf)
            throw Exception("Out of memory");
        _index = (UInt8*)malloc(16*4096);
        if (!_buf)
            throw Exception("Out of memory");
        _indexsize = 16*4096;
        _indexused = 8;

        for (int i = 0; i < AVI_HEADER_SIZE; ++i)
            fputc(0, _handle);
        _frames = 0;
        _written = 0;
        _audioused = 0;
        _audiowritten = 0;

        int blockwidth = 16;
        int blockheight = 16;
        _pixelsize = 4;
        _bufsize = (outputSize.y + 2*MAX_VECTOR)*_pitch*_pixelsize+2048;

        _buf1.allocate(_bufsize);
        _buf2.allocate(_bufsize);
        _work.allocate(_bufsize);

        int xblocks = (outputSize.x/blockwidth);
        int xleft = outputSize.x % blockwidth;
        if (xleft)
            ++xblocks;
        int yblocks = (outputSize.y/blockheight);
        int yleft = outputSize.y % blockheight;
        if (yleft)
            ++yblocks;
        _blockcount = yblocks*xblocks;
        _blocks = new FrameBlock[_blockcount];

        int i = 0;
        for (int y = 0; y < yblocks; ++y) {
            for (int x = 0; x < xblocks; ++x) {
                _blocks[i].start = ((y*blockheight) + MAX_VECTOR)*_pitch+
                    x*blockwidth + MAX_VECTOR;
                if (xleft && x == xblocks - 1) {
                    _blocks[i].dx = xleft;
                } else {
                    _blocks[i].dx = blockwidth;
                }
                if (yleft && y == yblocks - 1) {
                    _blocks[i].dy = yleft;
                } else {
                    _blocks[i].dy = blockheight;
                }
                ++i;
            }
        }

        memset(&_buf1[0], 0, _bufsize);
        memset(&_buf2[0], 0, _bufsize);
        memset(&_work[0], 0, _bufsize);
        _oldframe = &_buf1[0];
        _newframe = &_buf2[0];

        do {
            if (inputBufferRemaining == 0) {
                int bytesToRead = inputBufferSize;
                if (bytesToRead > inputFileSizeRemaining)
                    bytesToRead = inputFileSizeRemaining;
                inputPointer = &inputBuffer[0];
                in.read(inputPointer, bytesToRead);
                inputBufferRemaining = bytesToRead;
                inputFileSizeRemaining -= bytesToRead;
            }
            zs.avail_in = inputBufferRemaining;
            zs.next_in = inputPointer;
            zs.avail_out = outputBytesRemaining;
            zs.next_out = b + samples - outputBytesRemaining;
            int r = inflate(&zs, Z_SYNC_FLUSH);
            if (r != Z_STREAM_END && r != Z_OK)
                throw Exception("inflate failed");
            outputBytesRemaining = zs.avail_out;
            inputPointer = zs.next_in;
            inputBufferRemaining = zs.avail_in;

            if (outputBytesRemaining == 0) {
                if (inflateReset(&zs) != Z_OK)
                    throw Exception("inflateReset failed");
                outputBytesRemaining = samples;
                console.write(".");
                decoder.decode();

                bool keyFrame = false;
                if (_frames % 300 == 0)
                    keyFrame = true;
                 keyFrame = true;

                /* replace oldframe with new frame */
                unsigned char* copyFrame = _newframe;
                _newframe = _oldframe;
                _oldframe = copyFrame;

                compress.linesDone = 0;
                compress.writeSize = _bufSize;
                compress.writeDone = 1;
                compress.writeBuf = (unsigned char *)_buf;
                /* Set a pointer to the first byte which will contain info about this frame */
                unsigned char* firstByte = compress.writeBuf;
                *firstByte = 0;
                //Reset the work buffer
                _workUsed = 0;
                _workPos = 0;
                if (keyFrame) {
                    /* Make a keyframe */
                    *firstByte |= Mask_KeyFrame;
                    KeyframeHeader* header = (KeyframeHeader *)(compress.writeBuf + compress.writeDone);
                    header->high_version = 0; // DBZV_VERSION_HIGH;
                    header->low_version = 1; // DBZV_VERSION_LOW;
                    header->compression = 1; // COMPRESSION_ZLIB
    		        header->format = 8; // ZMBV_FORMAT_32BPP
                    header->blockwidth = 16;
                    header->blockheight = 16;
                    compress.writeDone += sizeof(KeyframeHeader);
                    /* Copy the new frame directly over */
                    /* Restart deflate */
                    deflateReset(&_zstream);
                }

                for (int i = 0; i < outputSize.y; ++i) {
                    void* rowPointer = decoded.data() + decoded.stride()*i;
                    unsigned char* destStart = _newframe + _pixelsize*(MAX_VECTOR+(compress.linesDone+MAX_VECTOR)*_pitch);
                    memcpy(destStart, rowPointer, outputSize.x * _pixelsize);
                    destStart += _pitch * _pixelsize;
                    compress.linesDone++;
                }

                if ((*compress.writeBuf) & Mask_KeyFrame) {
                    /* Add the full frame data */
                    unsigned char* readFrame = _newframe + _pixelsize*(MAX_VECTOR+MAX_VECTOR*_pitch);	
                    for (int i = 0; i < outputSize.y; ++i) {
                        memcpy(&_work[_workUsed], readFrame, outputSize.x*_pixelsize);
                        readFrame += _pitch*_pixelsize;
                        _workUsed += outputSize.x*_pixelsize;
                    }
                }
                else {
                    /* Add the delta frame data */
                    int written = 0;
                    int lastvector = 0;
                    signed char* vectors = (signed char*)&_work[_workUsed];
                    /* Align the following xor data on 4 byte boundary*/
                    _workUsed = (_workUsed + _blockcount*2 + 3) & ~3;
                    int totalx = 0;
                    int totaly = 0;
                    for (int b = 0; b < _blockcount; ++b) {
                        FrameBlock* block = &_blocks[b];
                        int bestvx = 0;
                        int bestvy = 0;
                        int bestchange = CompareBlock(0, 0, block);
                        int possibles = 64;
                        for (int v = 0; v < _VectorCount && possibles; ++v) {
                            if (bestchange < 4)
                                break;
                            int vx = _VectorTable[v].x;
                            int vy = _VectorTable[v].y;
                            if (PossibleBlock(vx, vy, block) < 4) {
                                --possibles;
                                int testchange = CompareBlock(vx, vy, block);
                                if (testchange < bestchange) {
                                    bestchange = testchange;
                                    bestvx = vx;
                                    bestvy = vy;
                                }
                            }
                        }
                        vectors[b*2+0] = (bestvx << 1);
                        vectors[b*2+1] = (bestvy << 1);
                        if (bestchange) {
                            vectors[b*2+0] |= 1;
                            long* pold=((long*)_oldframe) + block->start + bestvy*_pitch + bestvx;
                            long* pnew=((long*)_newframe) + block->start;
                            for (int y = 0; y < block->dy; ++y) {
                                for (int x = 0; x < block->dx; ++x) {
                                    *((long*)&_work[_workUsed]) = pnew[x] ^ pold[x];
                                    _workUsed += sizeof(long);
                                }
                                pold += _pitch;
                                pnew += _pitch;
                            }
                        }
                    }
                }
                /* Create the actual frame with compression */
                _zstream.next_in = (Bytef *)&_work[0];
                _zstream.avail_in = _workUsed;
                _zstream.total_in = 0;

                _zstream.next_out = (Bytef *)(compress.writeBuf + compress.writeDone);
                _zstream.avail_out = compress.writeSize - compress.writeDone;
                _zstream.total_out = 0;
                int res = deflate(&_zstream, Z_SYNC_FLUSH);
                int written = compress.writeDone + _zstream.total_out;

                CAPTURE_AddAviChunk( "00dc", written, _buf, keyFrame ? 0x10 : 0x0);
                ++_frames;

                //void CAPTURE_AddWave(UInt32 freq, UInt32 len, SInt16 * data)
                //{
                //    UInt left = WAVE_BUF - _audioused;
                //    if (left > len)
                //        left = len;
                //    memcpy( &_audiobuf[_audioused], data, left*4);
                //    _audioused += left;
                //    _audiorate = freq;
                //}

                //if ( capture.video.audioused ) {
                //    CAPTURE_AddAviChunk( "01wb", _audioused * 4, _audiobuf, 0);
                //    _audiowritten = _audioused*4;
                //    _audioused = 0;
                //}
            }

        } while (inputFileSizeRemaining != 0);

        if (inflateEnd(&zs) != Z_OK)
            throw Exception("inflateEnd failed");

        int main_list;
        _header_pos = 0;
        /* Try and write an avi header */
        AVIOUT4("RIFF");                    // Riff header
        AVIOUTd(AVI_HEADER_SIZE + _written - 8 + _indexused);
        AVIOUT4("AVI ");
        AVIOUT4("LIST");                    // List header
        main_list = _header_pos;
        AVIOUTd(0);				            // TODO size of list
        AVIOUT4("hdrl");

        AVIOUT4("avih");
        AVIOUTd(56);                         /* # of bytes to follow */
        AVIOUTd((11*912*262*2)/315);         /* Microseconds per frame */  // 1752256/105 ~= 16688
        AVIOUTd(0);
        AVIOUTd(0);                         /* PaddingGranularity (whatever that might be) */
        AVIOUTd(0x110);                     /* Flags,0x10 has index, 0x100 interleaved */
        AVIOUTd(_frames);      /* TotalFrames */
        AVIOUTd(0);                         /* InitialFrames */
        AVIOUTd(2);                         /* Stream count */
        AVIOUTd(0);                         /* SuggestedBufferSize */
        AVIOUTd(outputSize.x);       /* Width */
        AVIOUTd(outputSize.y);      /* Height */
        AVIOUTd(0);                         /* TimeScale:  Unit used to measure time */
        AVIOUTd(0);                         /* DataRate:   Data rate of playback     */
        AVIOUTd(0);                         /* StartTime:  Starting time of AVI data */
        AVIOUTd(0);                         /* DataLength: Size of AVI data chunk    */

        /* Video stream list */
        AVIOUT4("LIST");
        AVIOUTd(4 + 8 + 56 + 8 + 40);       /* Size of the list */
        AVIOUT4("strl");
        /* video stream header */
        AVIOUT4("strh");
        AVIOUTd(56);                        /* # of bytes to follow */
        AVIOUT4("vids");                    /* Type */
        AVIOUT4(CODEC_4CC);		            /* Handler */
        AVIOUTd(0);                         /* Flags */
        AVIOUTd(0);                         /* Reserved, MS says: wPriority, wLanguage */
        AVIOUTd(0);                         /* InitialFrames */
        AVIOUTd(82137);                     /* Scale */                                // 11*912*262
        AVIOUTd(4921875);                   /* Rate: Rate/Scale == samples/second */   // 157500000
        AVIOUTd(0);                         /* Start */
        AVIOUTd(_frames);      /* Length */
        AVIOUTd(0);                  /* SuggestedBufferSize */
        AVIOUTd(~0);                 /* Quality */
        AVIOUTd(0);                  /* SampleSize */
        AVIOUTd(0);                  /* Frame */
        AVIOUTd(0);                  /* Frame */
        /* The video stream format */
        AVIOUT4("strf");
        AVIOUTd(40);                 /* # of bytes to follow */
        AVIOUTd(40);                 /* Size */
        AVIOUTd(outputSize.x);         /* Width */
        AVIOUTd(outputSize.y);        /* Height */
//		OUTSHRT(1); OUTSHRT(24);     /* Planes, Count */
        AVIOUTd(0);
        AVIOUT4(CODEC_4CC);          /* Compression */
        AVIOUTd(outputSize.x*outputSize.y*4);  /* SizeImage (in bytes?) */
        AVIOUTd(0);                  /* XPelsPerMeter */
        AVIOUTd(0);                  /* YPelsPerMeter */
        AVIOUTd(0);                  /* ClrUsed: Number of colors used */
        AVIOUTd(0);                  /* ClrImportant: Number of colors important */

        /* Audio stream list */
        AVIOUT4("LIST");
        AVIOUTd(4 + 8 + 56 + 8 + 16);  /* Length of list in bytes */
        AVIOUT4("strl");
        /* The audio stream header */
        AVIOUT4("strh");
        AVIOUTd(56);            /* # of bytes to follow */
        AVIOUT4("auds");
        AVIOUTd(0);             /* Format (Optionally) */
        AVIOUTd(0);             /* Flags */
        AVIOUTd(0);             /* Reserved, MS says: wPriority, wLanguage */
        AVIOUTd(0);             /* InitialFrames */
        AVIOUTd(4);    /* Scale */
        AVIOUTd(_audiorate*4);             /* Rate, actual rate is scale/rate */
        AVIOUTd(0);             /* Start */
        if (!_audiorate)
            _audiorate = 1;
        AVIOUTd(_audiowritten/4);   /* Length */
        AVIOUTd(0);             /* SuggestedBufferSize */
        AVIOUTd(~0);            /* Quality */
        AVIOUTd(4);				/* SampleSize */
        AVIOUTd(0);             /* Frame */
        AVIOUTd(0);             /* Frame */
        /* The audio stream format */
        AVIOUT4("strf");
        AVIOUTd(16);            /* # of bytes to follow */
        AVIOUTw(1);             /* Format, WAVE_ZMBV_FORMAT_PCM */
        AVIOUTw(2);             /* Number of channels */
        AVIOUTd(_audiorate);          /* SamplesPerSec */
        AVIOUTd(_audiorate*4);        /* AvgBytesPerSec*/
        AVIOUTw(4);             /* BlockAlign */
        AVIOUTw(16);            /* BitsPerSample */
        int nmain = _header_pos - main_list - 4;
        /* Finish stream list, i.e. put number of bytes in the list to proper pos */

        int njunk = AVI_HEADER_SIZE - 8 - 12 - _header_pos;
        AVIOUT4("JUNK");
        AVIOUTd(njunk);
        /* Fix the size of the main list */
        _header_pos = main_list;
        AVIOUTd(nmain);
        _header_pos = AVI_HEADER_SIZE - 12;
        AVIOUT4("LIST");
        AVIOUTd(_written + 4); /* Length of list in bytes */
        AVIOUT4("movi");
        /* First add the index table to the end */
        memcpy(_index, "idx1", 4);
        host_writed(_index+4, _indexused - 8 );
        fwrite(_index, 1, _indexused, _handle);
        fseek(_handle, 0, SEEK_SET);
        fwrite(&_avi_header, 1, AVI_HEADER_SIZE, _handle);
        fclose(_handle);
        free(_index);
        free(_buf);
        _handle = 0;
    }
private:
    void AVIOUT4(const char* p)
    {
        memcpy(&_avi_header[_header_pos], p, 4);
        _header_pos+=4;
    }
    void AVIOUTw(int x)
    {
        host_writew(&_avi_header[_header_pos], x);
        _header_pos+=2;
    }
    void AVIOUTd(int x)
    {
        host_writed(&_avi_header[_header_pos], x);
        _header_pos+=4;
    }
    UInt8 _avi_header[AVI_HEADER_SIZE];
    int _header_pos;

    void CAPTURE_AddAviChunk(const char* tag, UInt32 size, void * data, UInt32 flags)
    {
        UInt8 chunk[8];
        UInt8 *index;
        UInt32 pos, writesize;

        chunk[0] = tag[0];
        chunk[1] = tag[1];
        chunk[2] = tag[2];
        chunk[3] = tag[3];
        host_writed(&chunk[4], size);
        /* Write the actual data */
        fwrite(chunk, 1, 8, _handle);
        writesize = (size+1)&~1;
        fwrite(data, 1, writesize, _handle);
        pos = _written + 4;
        _written += writesize + 8;
        if (_indexused + 16 >= _indexsize ) {
            _index = (UInt8*)realloc(_index, _indexsize + 16 * 4096);
            if (!_index)
                throw Exception("Ran out of memory during AVI capturing");
            _indexsize += 16*4096;
        }
        index = _index + _indexused;
        _indexused += 16;
        index[0] = tag[0];
        index[1] = tag[1];
        index[2] = tag[2];
        index[3] = tag[3];
        host_writed(index+4, flags);
        host_writed(index+8, pos);
        host_writed(index+12, size);
    }

    struct FrameBlock {
        int start;
        int dx,dy;
    };

    // methods
    int PossibleBlock(int vx, int vy, FrameBlock* block)
    {
        int ret = 0;
        long* pold = ((long*)_oldframe) + block->start + vy*_pitch + vx;
        long* pnew = ((long*)_newframe) + block->start;
        for (int y = 0; y < block->dy; y += 4) {
            for (int x = 0; x < block->dx; x += 4) {
                int test = 0-((pold[x] - pnew[x])&0x00ffffff);
                ret -= (test>>31);
            }
            pold += _pitch*4;
            pnew += _pitch*4;
        }
        return ret;
    }
    int CompareBlock(int vx, int vy, FrameBlock* block)
    {
        int ret = 0;
        long* pold = ((long*)_oldframe) + block->start + vy*_pitch + vx;
        long* pnew = ((long*)_newframe) + block->start;
        for (int y = 0; y < block->dy; ++y) {
            for (int x = 0; x < block->dx; ++x) {
                int test = 0-((pold[x] - pnew[x])&0x00ffffff);
                ret -= (test>>31);
            }
            pold += _pitch;
            pnew += _pitch;
        }
        return ret;
    }

    struct CodecVector {
        int x,y;
        int slot;
    };
    struct KeyframeHeader {
        unsigned char high_version;
        unsigned char low_version;
        unsigned char compression;
        unsigned char format;
        unsigned char blockwidth,blockheight;
    };

    struct {
        int		linesDone;
        int		writeSize;
        int		writeDone;
        unsigned char	*writeBuf;
    } compress;

    CodecVector _VectorTable[512];
    int _VectorCount;

    unsigned char *_oldframe, *_newframe;
    Array<UInt8> _buf1;
    Array<UInt8> _buf2;
    Array<UInt8> _work;
    int _bufsize;

    int _blockcount;
    FrameBlock* _blocks;

    int _workUsed, _workPos;

    int _pitch;
    int _pixelsize;

    z_stream _zstream;

    FILE* _handle;
    int _frames;
    SInt16 _audiobuf[WAVE_BUF][2];
    int _audioused;
    int _audiorate;
    int _audiowritten;
    int _written;
    int _bufSize;
    void* _buf;
    UInt8* _index;
    int _indexsize;
    int _indexused;
};
