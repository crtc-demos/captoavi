#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "alfe/main.h"
#include "alfe/complex.h"
#include "alfe/evaluate.h"
#include "alfe/bitmap.h"

#ifdef WIN32
#define ZLIB_WINAPI
#endif
#include "zlib.h"

#define CODEC_4CC "ZMBV"

typedef enum {
    ZMBV_FORMAT_NONE		= 0x00,
    ZMBV_FORMAT_32BPP	= 0x08
} zmbv_format_t;

#define MAX_VECTOR	16

#define Mask_KeyFrame			0x01

class VideoCodec
{
private:
    struct FrameBlock {
        int start;
        int dx,dy;
    };
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

    CodecVector VectorTable[512];
    int VectorCount;

    unsigned char *oldframe, *newframe;
    unsigned char *buf1, *buf2, *work;
    int bufsize;

    int blockcount;
    FrameBlock * blocks;

    int workUsed, workPos;

    int height, width, pitch;
    zmbv_format_t format;
    int pixelsize;

    z_stream zstream;

    // methods
    void FreeBuffers(void)
    {
        if (blocks) {
            delete[] blocks;blocks=0;
        }
        if (buf1) {
            delete[] buf1;buf1=0;
        }
        if (buf2) {
            delete[] buf2;buf2=0;
        }
        if (work) {
            delete[] work;work=0;
        }
    }                               
    bool SetupBuffers(zmbv_format_t _format, int blockwidth, int blockheight)
    {
        FreeBuffers();
        pixelsize = 4;
        bufsize = (height+2*MAX_VECTOR)*pitch*pixelsize+2048;

        buf1 = new unsigned char[bufsize];
        buf2 = new unsigned char[bufsize];
        work = new unsigned char[bufsize];

        int xblocks = (width/blockwidth);
        int xleft = width % blockwidth;
        if (xleft) xblocks++;
        int yblocks = (height/blockheight);
        int yleft = height % blockheight;
        if (yleft) yblocks++;
        blockcount=yblocks*xblocks;
        blocks=new FrameBlock[blockcount];

        if (!buf1 || !buf2 || !work || !blocks) {
            FreeBuffers();
            return false;
        }
        int y,x,i;
        i=0;
        for (y=0;y<yblocks;y++) {
            for (x=0;x<xblocks;x++) {
                blocks[i].start=((y*blockheight)+MAX_VECTOR)*pitch+
                    (x*blockwidth)+MAX_VECTOR;
                if (xleft && x==(xblocks-1)) {
                    blocks[i].dx=xleft;
                } else {
                    blocks[i].dx=blockwidth;
                }
                if (yleft && y==(yblocks-1)) {
                    blocks[i].dy=yleft;
                } else {
                    blocks[i].dy=blockheight;
                }
                i++;
            }
        }

        memset(buf1,0,bufsize);
        memset(buf2,0,bufsize);
        memset(work,0,bufsize);
        oldframe=buf1;
        newframe=buf2;
        format = _format;
        return true;
    }

    template<class P> void AddXorFrame(void)
    {
        int written=0;
        int lastvector=0;
        signed char * vectors=(signed char*)&work[workUsed];
        /* Align the following xor data on 4 byte boundary*/
        workUsed=(workUsed + blockcount*2 +3) & ~3;
        int totalx=0;
        int totaly=0;
        for (int b=0;b<blockcount;b++) {
            FrameBlock * block=&blocks[b];
            int bestvx = 0;
            int bestvy = 0;
            int bestchange=CompareBlock<P>(0,0, block);
            int possibles=64;
            for (int v=0;v<VectorCount && possibles;v++) {
                if (bestchange<4) break;
                int vx = VectorTable[v].x;
                int vy = VectorTable[v].y;
                if (PossibleBlock<P>(vx, vy, block) < 4) {
                    possibles--;
                    int testchange=CompareBlock<P>(vx,vy, block);
                    if (testchange<bestchange) {
                        bestchange=testchange;
                        bestvx = vx;
                        bestvy = vy;
                    }
                }
            }
            vectors[b*2+0]=(bestvx << 1);
            vectors[b*2+1]=(bestvy << 1);
            if (bestchange) {
                vectors[b*2+0]|=1;
                AddXorBlock<P>(bestvx, bestvy, block);
            }
        }
    }
    template<class P> int PossibleBlock(int vx,int vy,FrameBlock * block)
    {
        int ret=0;
        P * pold=((P*)oldframe)+block->start+(vy*pitch)+vx;
        P * pnew=((P*)newframe)+block->start;;	
        for (int y=0;y<block->dy;y+=4) {
            for (int x=0;x<block->dx;x+=4) {
                int test=0-((pold[x]-pnew[x])&0x00ffffff);
                ret-=(test>>31);
            }
            pold+=pitch*4;
            pnew+=pitch*4;
        }
        return ret;
    }
    template<class P> int CompareBlock(int vx,int vy,FrameBlock * block)
    {
        int ret=0;
        P * pold=((P*)oldframe)+block->start+(vy*pitch)+vx;
        P * pnew=((P*)newframe)+block->start;;
        for (int y=0;y<block->dy;y++) {
            for (int x=0;x<block->dx;x++) {
                int test=0-((pold[x]-pnew[x])&0x00ffffff);
                ret-=(test>>31);
            }
            pold+=pitch;
            pnew+=pitch;
        }
        return ret;
    }
    template<class P> void AddXorBlock(int vx,int vy,FrameBlock * block)
    {
        P * pold=((P*)oldframe)+block->start+(vy*pitch)+vx;
        P * pnew=((P*)newframe)+block->start;
        for (int y=0;y<block->dy;y++) {
            for (int x=0;x<block->dx;x++) {
                *((P*)&work[workUsed])=pnew[x] ^ pold[x];
                workUsed+=sizeof(P);
            }
            pold+=pitch;
            pnew+=pitch;
        }
    }
public:
    VideoCodec()
    {
        int x,y,s;
        VectorCount=1;
        VectorTable[0].x=VectorTable[0].y=0;
        for (s=1;s<=10;s++) {
            for (y=0-s;y<=0+s;y++) for (x=0-s;x<=0+s;x++) {
                if (abs(x)==s || abs(y)==s) {
                    VectorTable[VectorCount].x=x;
                    VectorTable[VectorCount].y=y;
                    VectorCount++;
                }
            }
        }
        blocks = 0;
        buf1 = 0;
        buf2 = 0;
        work = 0;
        memset( &zstream, 0, sizeof(zstream));
    }
    bool SetupCompress( int _width, int _height)
    {
        width = _width;
        height = _height;
        pitch = _width + 2*MAX_VECTOR;
        format = ZMBV_FORMAT_NONE;
        if (deflateInit (&zstream, 4) != Z_OK)
            return false;
        return true;
    }
    int NeededSize( int _width, int _height, zmbv_format_t _format)
    {
        int f = 4*_width*_height + 2*(1+(_width/8)) * (1+(_height/8))+1024;
        return f + f/1000;
    }

    void CompressLines(int lineCount, void *lineData[])
    {
        int linePitch = pitch * pixelsize;
        int lineWidth = width * pixelsize;
        int i = 0;
        unsigned char *destStart = newframe + pixelsize*(MAX_VECTOR+(compress.linesDone+MAX_VECTOR)*pitch);
        while ( i < lineCount && (compress.linesDone < height)) {
            memcpy(destStart, lineData[i],  lineWidth );
            destStart += linePitch;
            i++;compress.linesDone++;
        }
    }
    bool PrepareCompressFrame(int flags,  zmbv_format_t _format, void *writeBuf, int writeSize)
    {
        int i;
        unsigned char *firstByte;

        if (_format != format) {
            if (!SetupBuffers( _format, 16, 16))
                return false;
            flags|=1;	//Force a keyframe
        }
        /* replace oldframe with new frame */
        unsigned char *copyFrame = newframe;
        newframe = oldframe;
        oldframe = copyFrame;

        compress.linesDone = 0;
        compress.writeSize = writeSize;
        compress.writeDone = 1;
        compress.writeBuf = (unsigned char *)writeBuf;
        /* Set a pointer to the first byte which will contain info about this frame */
        firstByte = compress.writeBuf;
        *firstByte = 0;
        //Reset the work buffer
        workUsed = 0;workPos = 0;
        if (flags & 1) {
            /* Make a keyframe */
            *firstByte |= Mask_KeyFrame;
            KeyframeHeader * header = (KeyframeHeader *)(compress.writeBuf + compress.writeDone);
            header->high_version = 0; // DBZV_VERSION_HIGH;
            header->low_version = 1; // DBZV_VERSION_LOW;
            header->compression = 1; // COMPRESSION_ZLIB
            header->format = format;
            header->blockwidth = 16;
            header->blockheight = 16;
            compress.writeDone += sizeof(KeyframeHeader);
            /* Copy the new frame directly over */
            /* Restart deflate */
            deflateReset(&zstream);
        }
        return true;
    }
    int FinishCompressFrame( void )
    {
        unsigned char firstByte = *compress.writeBuf;
        if (firstByte & Mask_KeyFrame) {
            int i;
            /* Add the full frame data */
            unsigned char * readFrame = newframe + pixelsize*(MAX_VECTOR+MAX_VECTOR*pitch);	
            for (i=0;i<height;i++) {
                memcpy(&work[workUsed], readFrame, width*pixelsize);
                readFrame += pitch*pixelsize;
                workUsed += width*pixelsize;
            }
        } else {
            /* Add the delta frame data */
            switch (format) {
            case ZMBV_FORMAT_32BPP:
                AddXorFrame<long>();
                break;
            }
        }
        /* Create the actual frame with compression */
        zstream.next_in = (Bytef *)work;
        zstream.avail_in = workUsed;
        zstream.total_in = 0;

        zstream.next_out = (Bytef *)(compress.writeBuf + compress.writeDone);
        zstream.avail_out = compress.writeSize - compress.writeDone;
        zstream.total_out = 0;
        int res = deflate(&zstream, Z_SYNC_FLUSH);
        return compress.writeDone + zstream.total_out;
    }
};

#define WAVE_BUF 16*1024
#define AVI_HEADER_SIZE	500

typedef   signed short		Bit16s;
typedef unsigned int		Bitu;
typedef  unsigned long		Bit32u;
typedef  unsigned char		Bit8u;
typedef unsigned short		Bit16u;

static struct
{
    struct {
        FILE		*handle;
        Bitu		frames;
        Bit16s		audiobuf[WAVE_BUF][2];
        Bitu		audioused;
        Bitu		audiorate;
        Bitu		audiowritten;
        VideoCodec	*codec;
        Bitu		width, height, bpp;
        Bitu		written;
        float		fps;
        int			bufSize;
        void		*buf;
        Bit8u		*index;
        Bitu		indexsize, indexused;
    } video;
} capture;

typedef Bit8u * HostPt;

static void host_writed(HostPt off,Bit32u val)
{
    off[0]=(Bit8u)(val);
    off[1]=(Bit8u)(val >> 8);
    off[2]=(Bit8u)(val >> 16);
    off[3]=(Bit8u)(val >> 24);
}
static void host_writew(HostPt off,Bit16u val)
{
    off[0]=(Bit8u)(val);
    off[1]=(Bit8u)(val >> 8);
}

static void CAPTURE_AddAviChunk(const char * tag, Bit32u size, void * data, Bit32u flags)
{
    Bit8u chunk[8];Bit8u *index;Bit32u pos, writesize;

    chunk[0] = tag[0];chunk[1] = tag[1];chunk[2] = tag[2];chunk[3] = tag[3];
    host_writed(&chunk[4], size);
    /* Write the actual data */
    fwrite(chunk,1,8,capture.video.handle);
    writesize = (size+1)&~1;
    fwrite(data,1,writesize,capture.video.handle);
    pos = capture.video.written + 4;
    capture.video.written += writesize + 8;
    if (capture.video.indexused + 16 >= capture.video.indexsize ) {
        capture.video.index = (Bit8u*)realloc( capture.video.index, capture.video.indexsize + 16 * 4096);
        if (!capture.video.index)
            throw Exception("Ran out of memory during AVI capturing");
        capture.video.indexsize += 16*4096;
    }
    index = capture.video.index+capture.video.indexused;
    capture.video.indexused += 16;
    index[0] = tag[0];
    index[1] = tag[1];
    index[2] = tag[2];
    index[3] = tag[3];
    host_writed(index+4, flags);
    host_writed(index+8, pos);
    host_writed(index+12, size);
}

void CAPTURE_AddWave(Bit32u freq, Bit32u len, Bit16s * data)
{
    Bitu left = WAVE_BUF - capture.video.audioused;
    if (left > len)
        left = len;
    memcpy( &capture.video.audiobuf[capture.video.audioused], data, left*4);
    capture.video.audioused += left;
    capture.video.audiorate = freq;
}

float sinc(float z)
{
    if (z == 0.0f)
        return 1.0f;
    z *= M_PI;
    return sin(z)/z;
}

float lanczos(float z)
{
    if (z < -3 || z > 3)
        return 0;
    return sinc(z)*sinc(z/3);
}

template<class T> Byte checkClamp(T x)
{
    int y = static_cast<int>(x);
    return clamp(0, y, 255);
//    return x;
}

Complex<float> rotor(float phase)
{
    float angle = static_cast<float>(phase*tau);
    return Complex<float>(cos(angle), sin(angle));
}

class NTSCDecoder
{
public:
    NTSCDecoder()
    {
        contrast = 1.41;
        brightness = -11.0;
        saturation = 0.303;
        hue = 0;
        wobbleAmplitude = 0.0042;
        wobblePhase = 0.94;
    }
    void setBuffers(Byte* input, Byte* output) { _input = input; _output = output; }

    void decode()
    {
        // Settings

        static const int lines = 240;
        static const int nominalSamplesPerLine = 1820;
        static const int firstSyncSample = -130;  // Assumed position of previous hsync before our samples started
        static const int pixelsPerLine = 1140;
        static const float kernelSize = 3;  // Lanczos parameter
        static const int nominalSamplesPerCycle = 8;
        static const int driftSamples = 40;
        static const int burstSamples = 40;
        static const int firstBurstSample = 208;
        static const int burstCenter = firstBurstSample + burstSamples/2;

        Byte* b = _input;


        // Pass 1 - find sync and burst pulses, compute wobble amplitude and phase

        float deltaSamplesPerCycle = 0;

        int syncPositions[lines + 1];
        int oldP = firstSyncSample - driftSamples;
        int p = oldP + nominalSamplesPerLine;
        float samplesPerLine = nominalSamplesPerLine;
        Complex<float> bursts[lines + 1];
        float burstDCs[lines + 1];
        Complex<float> wobbleRotor = 0;
        Complex<float> hueRotor = rotor((33 + hue)/360);
        float totalBurstAmplitude = 0;
        float burstDCAverage = 0;
        for (int line = 0; line < lines + 1; ++line) {
            Complex<float> burst = 0;
            float burstDC = 0;
            for (int i = firstBurstSample; i < firstBurstSample + burstSamples; ++i) {
                int j = oldP + i;
                int sample = b[j];
                float phase = (j&7)/8.0f;
                burst += rotor(phase)*sample;
                burstDC += sample;
            }

            float burstAmplitude = burst.modulus()/burstSamples;
            totalBurstAmplitude += burstAmplitude;
            wobbleRotor += burstAmplitude*rotor(burst.argument() * 8 / tau);
            bursts[line] = burst*hueRotor/burstSamples;
            burstDC /= burstSamples;
            burstDCs[line] = burstDC;

            syncPositions[line] = p;
            oldP = p;
            for (int i = 0; i < driftSamples*2; ++i) {
                if (b[p] < 9)
                    break;
                ++p;
            }
            p += nominalSamplesPerLine - driftSamples;

            if (line < 200) {
                samplesPerLine = (2*samplesPerLine + p - oldP)/3;
                burstDCAverage = (2*burstDCAverage + burstDC)/3;
            }
        }
        float averageBurstAmplitude = totalBurstAmplitude / (lines + 1);

        float deltaSamplesPerLine = samplesPerLine - nominalSamplesPerLine;


        // Pass 2 - render

        Byte* output = _output;

        float q = syncPositions[1] - samplesPerLine;
        syncPositions[0] = q;
        Complex<float> burst = bursts[0];
        float rotorTable[8];
        for (int i = 0; i < 8; ++i)
            rotorTable[i] = rotor(i/8.0).x*saturation;
        Complex<float> expectedBurst = burst;
        int oldActualSamplesPerLine = nominalSamplesPerLine;
        for (int line = 0; line < lines; ++line) {
            // Determine the phase, amplitude and DC offset of the color signal
            // from the color burst, which starts shortly after the horizontal
            // sync pulse ends. The color burst is 9 cycles long, and we look
            // at the middle 5 cycles.

            float contrast1 = contrast;
            //int samplesPerLineInt = static_cast<int>(samplesPerLine);
            Complex<float> actualBurst = bursts[line];
            burst = (expectedBurst*2 + actualBurst)/3;

            float phaseDifference = (actualBurst*(expectedBurst.conjugate())).argument()/tau;
            float adjust = -phaseDifference/pixelsPerLine;

            Complex<float> chromaAdjust = burst.conjugate()*contrast1*saturation;
            burstDCAverage = (2*burstDCAverage + burstDCs[line])/3;
            float brightness1 = brightness + 65 - burstDCAverage;

            // Resample the image data

            //float samplesPerLine = nominalSamplesPerLine + deltaSamplesPerLine;
            for (int x = 151; x < 151 + 960; ++x) {
                float y = 0;
                Complex<float> c = 0;
                float t = 0;

                float kFrac0 = x*samplesPerLine/pixelsPerLine;
                float kFrac = q + kFrac0;
                int k = static_cast<int>(kFrac);
                kFrac -= k;
                float samplesPerCycle = nominalSamplesPerCycle + deltaSamplesPerCycle;
                float z0 = -kFrac/samplesPerCycle;
                int firstInput = -kernelSize*samplesPerCycle + kFrac;
                int lastInput = kernelSize*samplesPerCycle + kFrac;

                for (int j = firstInput; j <= lastInput; ++j) {
                    // The input sample corresponding to the output pixel is k+kFrac
                    // The sample we're looking at in this iteration is j+k
                    // The difference is j-kFrac
                    // So the value we pass to lanczos() is (j-kFrac)/samplesPerCycle
                    // So z0 = -kFrac/samplesPerCycle;

                    float s = lanczos(j/samplesPerCycle + z0);
                    int i = j + k;
                    float z = s*b[i];
                    y += z;
                    c.x += rotorTable[i & 7]*z;
                    c.y += rotorTable[(i + 6) & 7]*z;
                    //c += rotor((i&7)/8.0)*z*saturation;
                    t += s;
                }

                float wobble = 1 - cos(c.argument()*8 + wobblePhase*tau)*wobbleAmplitude; ///(averageBurstAmplitude*contrast);

                y = y*contrast1*wobble/t + brightness1; // - cos(c.argument()*8 + wobblePhase*tau)*wobbleAmplitude*contrast;
                c = c*chromaAdjust*rotor((x - burstCenter*pixelsPerLine/samplesPerLine)*adjust)*wobble/t;

                output[0] = checkClamp(y + 0.9563*c.x + 0.6210*c.y);
                output[1] = checkClamp(y - 0.2721*c.x - 0.6474*c.y);
                output[2] = checkClamp(y - 1.1069*c.x + 1.7046*c.y);
                output += 3;
            }

            int p = syncPositions[line + 1];
            int actualSamplesPerLine = p - syncPositions[line];
            samplesPerLine = (2*samplesPerLine + actualSamplesPerLine)/3;
            q += samplesPerLine;
            q = (10*q + p)/11;

            expectedBurst = actualBurst;
        }
    }
    float contrast;
    float brightness;
    float saturation;
    float hue;
    float wobbleAmplitude;
    float wobblePhase;
    Byte* _input;
    Byte* _output;
};

class Program : public ProgramBase
{
public:
    void run()
    {
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

        Bitmap<SRGB> decoded(Vector(640, 240));
        NTSCDecoder decoder;
        decoder.setBuffers(b, decoded.data());

        Bitmap<SRGB> output(Vector(640, 480));

        //void CAPTURE_AddImage(Bitu width, Bitu height, Bitu bpp, Bitu pitch, Bitu flags, float fps, Bit8u * data) {
        zmbv_format_t format = ZMBV_FORMAT_32BPP;
        static const int width = 1280;
        static const int height = 720;
        capture.video.handle = fopen("captured.avi","wb");
        if (!capture.video.handle)
            throw Exception("Can't open file");
        capture.video.codec = new VideoCodec();
        if (!capture.video.codec->SetupCompress( width, height))
            throw Exception("SetupCompress failed");
        capture.video.bufSize = capture.video.codec->NeededSize(width, height, format);
        capture.video.buf = malloc( capture.video.bufSize );
        if (!capture.video.buf)
            throw Exception("Out of memory");
        capture.video.index = (Bit8u*)malloc( 16*4096 );
        if (!capture.video.buf)
            throw Exception("Out of memory");
        capture.video.indexsize = 16*4096;
        capture.video.indexused = 8;

        capture.video.width = width;
        capture.video.height = height;
        capture.video.fps = 157500000/(11*912*262); // 1640625/27379 ~= 59.923
        for (i=0;i<AVI_HEADER_SIZE;i++)
            fputc(0,capture.video.handle);
        capture.video.frames = 0;
        capture.video.written = 0;
        capture.video.audioused = 0;
        capture.video.audiowritten = 0;

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

            if (outputBytesRemaining == 0) {
                decoder.decode();

                //for (int o = 0; o < 478; o += 2) {
                //    for (int x = 0; x < 640; ++x) {
                //        output[Vector(x, o)] = decoded[Vector(x, o/2)];
                //        Vector3<int> a = Vector3Cast<int>(decoded[Vector(x, o/2)]) + Vector3Cast<int>(decoded[Vector(x, o/2 + 1)]);
                //        output[Vector(x, o+1)] = Vector3Cast<Byte>(a/2);
                //        //output[Vector(x, o+1)] = decoded[Vector(x, o/2)];
                //    }
                //}
                //for (int x = 0; x < 640; ++x) {
                //    output[Vector(x, 478)] = decoded[Vector(x, 239)];
                //    output[Vector(x, 479)] = decoded[Vector(x, 239)];
                //}

                //png.save(output, File(name, true));

                //AutoHandle out = File(name + ".raw", true).openWrite();
                //out.write(b, 450*1024);


                Bitu i;
                Bitu countWidth = width;

                //void CAPTURE_AddImage(Bitu width, Bitu height, Bitu bpp, Bitu pitch, Bitu flags, float fps, Bit8u * data) {
                int codecFlags;
                if (capture.video.frames % 300 == 0)
                    codecFlags = 1;
                else codecFlags = 0;
                if (!capture.video.codec->PrepareCompressFrame( codecFlags, format, capture.video.buf, capture.video.bufSize))
                    throw Exception("PrepareCompressFrame failed");

                for (i=0;i<height;i++) {
                    void * rowPointer=(data+i*pitch);
                    capture.video.codec->CompressLines( 1, &rowPointer );
                }
                int written = capture.video.codec->FinishCompressFrame();
                if (written < 0)
                    throw Exception("FinishCompressFrame failed");
                CAPTURE_AddAviChunk( "00dc", written, capture.video.buf, codecFlags & 1 ? 0x10 : 0x0);
                capture.video.frames++;
                if ( capture.video.audioused ) {
                    CAPTURE_AddAviChunk( "01wb", capture.video.audioused * 4, capture.video.audiobuf, 0);
                    capture.video.audiowritten = capture.video.audioused*4;
                    capture.video.audioused = 0;
                }

                // TODO: Pillarbox
                // TODO: Triple lines
                // TODO: Call CAPTURE_AddWave
            }

        } while (true);

        if (inflateEnd(&zs) != Z_OK)
            throw Exception("inflateEnd failed");


        Bit8u avi_header[AVI_HEADER_SIZE];
        Bitu main_list;
        Bitu header_pos=0;
#define AVIOUT4(_S_) memcpy(&avi_header[header_pos],_S_,4);header_pos+=4;
#define AVIOUTw(_S_) host_writew(&avi_header[header_pos], _S_);header_pos+=2;
#define AVIOUTd(_S_) host_writed(&avi_header[header_pos], _S_);header_pos+=4;
        /* Try and write an avi header */
        AVIOUT4("RIFF");                    // Riff header
        AVIOUTd(AVI_HEADER_SIZE + capture.video.written - 8 + capture.video.indexused);
        AVIOUT4("AVI ");
        AVIOUT4("LIST");                    // List header
        main_list = header_pos;
        AVIOUTd(0);				            // TODO size of list
        AVIOUT4("hdrl");

        AVIOUT4("avih");
        AVIOUTd(56);                         /* # of bytes to follow */
        AVIOUTd((Bit32u)(1000000 / capture.video.fps));       /* Microseconds per frame */
        AVIOUTd(0);
        AVIOUTd(0);                         /* PaddingGranularity (whatever that might be) */
        AVIOUTd(0x110);                     /* Flags,0x10 has index, 0x100 interleaved */
        AVIOUTd(capture.video.frames);      /* TotalFrames */
        AVIOUTd(0);                         /* InitialFrames */
        AVIOUTd(2);                         /* Stream count */
        AVIOUTd(0);                         /* SuggestedBufferSize */
        AVIOUTd(capture.video.width);       /* Width */
        AVIOUTd(capture.video.height);      /* Height */
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
        AVIOUTd(1000000);                   /* Scale */
        AVIOUTd((Bit32u)(1000000 * capture.video.fps));              /* Rate: Rate/Scale == samples/second */
        AVIOUTd(0);                         /* Start */
        AVIOUTd(capture.video.frames);      /* Length */
        AVIOUTd(0);                  /* SuggestedBufferSize */
        AVIOUTd(~0);                 /* Quality */
        AVIOUTd(0);                  /* SampleSize */
        AVIOUTd(0);                  /* Frame */
        AVIOUTd(0);                  /* Frame */
        /* The video stream format */
        AVIOUT4("strf");
        AVIOUTd(40);                 /* # of bytes to follow */
        AVIOUTd(40);                 /* Size */
        AVIOUTd(capture.video.width);         /* Width */
        AVIOUTd(capture.video.height);        /* Height */
//		OUTSHRT(1); OUTSHRT(24);     /* Planes, Count */
        AVIOUTd(0);
        AVIOUT4(CODEC_4CC);          /* Compression */
        AVIOUTd(capture.video.width * capture.video.height*4);  /* SizeImage (in bytes?) */
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
        AVIOUTd(capture.video.audiorate*4);             /* Rate, actual rate is scale/rate */
        AVIOUTd(0);             /* Start */
        if (!capture.video.audiorate)
            capture.video.audiorate = 1;
        AVIOUTd(capture.video.audiowritten/4);   /* Length */
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
        AVIOUTd(capture.video.audiorate);          /* SamplesPerSec */
        AVIOUTd(capture.video.audiorate*4);        /* AvgBytesPerSec*/
        AVIOUTw(4);             /* BlockAlign */
        AVIOUTw(16);            /* BitsPerSample */
        int nmain = header_pos - main_list - 4;
        /* Finish stream list, i.e. put number of bytes in the list to proper pos */

        int njunk = AVI_HEADER_SIZE - 8 - 12 - header_pos;
        AVIOUT4("JUNK");
        AVIOUTd(njunk);
        /* Fix the size of the main list */
        header_pos = main_list;
        AVIOUTd(nmain);
        header_pos = AVI_HEADER_SIZE - 12;
        AVIOUT4("LIST");
        AVIOUTd(capture.video.written+4); /* Length of list in bytes */
        AVIOUT4("movi");
        /* First add the index table to the end */
        memcpy(capture.video.index, "idx1", 4);
        host_writed( capture.video.index+4, capture.video.indexused - 8 );
        fwrite( capture.video.index, 1, capture.video.indexused, capture.video.handle);
        fseek(capture.video.handle, 0, SEEK_SET);
        fwrite(&avi_header, 1, AVI_HEADER_SIZE, capture.video.handle);
        fclose( capture.video.handle );
        free( capture.video.index );
        free( capture.video.buf );
        delete capture.video.codec;
        capture.video.handle = 0;
    }
};