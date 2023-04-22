/*
 *
 *  Adapted by Sam Siewert for use with UVC web cameras and Bt878 frame
 *  grabber NTSC cameras to acquire digital video from a source,
 *  time-stamp each frame acquired, save to a PGM or PPM file.
 *
 *  The original code adapted was open source from V4L2 API and had the
 *  following use and incorporation policy:
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 *
 *@author - Additional code by - Khyati Satta
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h> 
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <linux/videodev2.h>
#include <time.h>
#include <syslog.h>
#include <stdbool.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define HRES 320
#define VRES 240
#define HRES_STR "320"
#define VRES_STR "240"

#define PRINT_ENABLE  (0)


// Global flag for tracking the end of camera frame capture
static volatile bool is_capture = true;


// Signal handler for SIGINT, SIGTERM and SIGALRM signals
void signalHandler(int signal)
{
    switch(signal){
        case SIGINT:
        syslog(LOG_DEBUG ,"Caught signal SIGINT\n");
        #if (PRINT_ENABLE == 1)
        printf("Caught signal SIGINT\n");
        #endif
        is_capture = false; 
        break;

        case SIGTERM:
        syslog(LOG_DEBUG ,"Caught signal SIGTERM\n");
        #if (PRINT_ENABLE == 1)
        printf("Caught signal SIGTERM\n");
        #endif
        is_capture = false;
        break;
    }

     
}

// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;

struct buffer
{
    void *start;
    size_t length;
};

const char *dev_name = "/dev/video0";
static int fd = -1;
struct buffer *buffers;
static unsigned int n_buffers;
static int force_format = 1;


/*
 * Errro handling function
 * 
 * Parameters:
 *   char *s -> The string for the function that failed
 *
 * Returns:
 * 	 None
 */
#if (PRINT_ENABLE == 1)
static void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}
#endif

/*
 * Wrapper function for the ioctl system call
 * 
 * The wrapper function makes sure that the ioctl system call 
 * does not return an error because of an interrupt signal (EINTR)
 * 
 * Parameters:
 *   int fh - File handler for the device
 *   int request - The type of operation to be performed on the device
 *   void *arg - Return value: The video standard supported by the connected device [Example: NTSC , PAL , SECAM]
 * 
 * For more information on video standards: https://www.linuxtv.org/downloads/v4l-dvb-apis-old/vidioc-enumstd.html#video-standards
 *
 * Returns:
 * 	 Error code: If ioctl fails for reasons other than an interrupt signal (EINTR)
 */
static int xioctl(int fh, int request, void *arg)
{
    int r;
    do
    {
        r = ioctl(fh, request, arg);

    } while (-1 == r && EINTR == errno);
    return r;
}


/*****************FUNCTION TO CONVERT RGB FRAME BUFFER TO A PPM FILE******************/
char ppm_header[] = "P6\n#9999999999 sec 9999999999 msec \n" HRES_STR " " VRES_STR "\n255\n";
char ppm_dumpname[] = "frames/test00000000.ppm";

static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, total, dumpfd;


    snprintf(&ppm_dumpname[11], 9, "%08d", tag);
    strncat(&ppm_dumpname[15], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec  ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec) / 1000000));
    strcat(&ppm_header[29], " msec \n" HRES_STR " " VRES_STR "\n255\n");

    // subtract 1 because sizeof for string includes null terminator
    written = write(dumpfd, ppm_header, sizeof(ppm_header) - 1);

    total = 0;

    do
    {
        written = write(dumpfd, p, size);
        total += written;
    } while (total < size);

    printf("Wrote a frame\n");

    close(dumpfd);
}


/**************************************************CAMERA CAPTURE FUNCTIONS**************************************************/
// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
    int r1, g1, b1;

    // replaces floating point coefficients
    int c = y - 16, d = u - 128, e = v - 128;

    // Conversion that avoids floating point
    r1 = (298 * c + 409 * e + 128) >> 8;
    g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
    b1 = (298 * c + 516 * d + 128) >> 8;

    // Computed values may need clipping.
    if (r1 > 255)
        r1 = 255;
    if (g1 > 255)
        g1 = 255;
    if (b1 > 255)
        b1 = 255;

    if (r1 < 0)
        r1 = 0;
    if (g1 < 0)
        g1 = 0;
    if (b1 < 0)
        b1 = 0;

    *r = r1;
    *g = g1;
    *b = b1;
}

unsigned int framecnt = 0;
unsigned char bigbuffer[(1280 * 960)];

/*
 * Function to process the frames
 *
 * In each frame, pixel by pixel, the yuv pixels to rgb
 *  
 * Parameters:
 *   const void *p -> The buffer holding each frame
 *   int size -> The size of the frame
 *
 * Returns:
 * 	 None 
 */
static void process_image(const void *p, int size)
{
    int i, newi = 0;
    struct timespec frame_time;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);

    framecnt++;
    // printf("frame %d: ", framecnt);

    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {

        for (i = 0, newi = 0; i < size; i = i + 4, newi = newi + 6)
        {
            y_temp = (int)pptr[i];
            u_temp = (int)pptr[i + 1];
            y2_temp = (int)pptr[i + 2];
            v_temp = (int)pptr[i + 3];
            yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[newi], &bigbuffer[newi + 1], &bigbuffer[newi + 2]);
            yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[newi + 3], &bigbuffer[newi + 4], &bigbuffer[newi + 5]);
        }
        dump_ppm(bigbuffer, ((size * 6) / 4), framecnt, &frame_time);
    }

    else
    {
        printf("ERROR - unknown dump format\n");
    }

    fflush(stderr);
    fflush(stdout);
}

/*
 * Function to read frames
 *  
 * Parameters:
 *   None
 *
 * Returns:
 * 	 None 
 */
static int read_frame(void)
{
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        switch (errno)
        {
        case EAGAIN:
            return 0;

        case EIO:
            /* Could ignore EIO, but drivers should only set for serious errors, although some set for
               non-fatal errors too.
             */
            return 0;

        #if (PRINT_ENABLE == 1)
        default:
            
                printf("mmap failure\n");
                errno_exit("VIDIOC_DQBUF");
        #endif
        }
    }

    assert(buf.index < n_buffers);

    process_image(buffers[buf.index].start, buf.bytesused);

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
        #if (PRINT_ENABLE == 1)
            errno_exit("VIDIOC_QBUF");
        #endif
    }
        

    return 1;
}

/*
 * Function to captures frames
 *
 * The function captures frames every 500 milliseconds
 *  
 * Parameters:
 *   None
 *
 * Returns:
 * 	 None 
 */
static void capture_frame(void)
{
    struct timespec read_delay;
    struct timespec time_error;

    read_delay.tv_sec = 0;
    read_delay.tv_nsec = 50000000;

    for (;;)
    {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r)
        {
            if (EINTR == errno)
                continue;

            #if (PRINT_ENABLE == 1)
                errno_exit("select");
            #endif
        }

        if (0 == r)
        {
            fprintf(stderr, "select timeout\n");
            exit(EXIT_FAILURE);
        }

        if (read_frame())
        {
            if (nanosleep(&read_delay, &time_error) != 0)
                perror("nanosleep");
            else
                // printf("time_error.tv_sec=%ld, time_error.tv_nsec=%ld\n", time_error.tv_sec, time_error.tv_nsec);
            break;
        }

        /* EAGAIN - continue select loop unless count done. */
    }
}

/*
 * Stop capturing the frames from the camera
 *
 * Allocate the v4l2 buffers for capturing the 
 * frames and set the buffer type to disable video streaming
 *  
 * Parameters:
 *   None
 *
 * Returns:
 * 	 None 
 */
static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type)){
        #if (PRINT_ENABLE == 1)
            errno_exit("VIDIOC_STREAMOFF");
        #endif
    }
        
}

/*
 * Start capturing the frames from the camera
 *
 * Allocate the v4l2 buffers for capturing the 
 * frames and set the buffer type to support video streaming
 *  
 * Parameters:
 *   None
 *
 * Returns:
 * 	 None 
 */
static void start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < n_buffers; ++i)
    {
        // printf("allocated buffer %d\n", i);
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
            #if (PRINT_ENABLE == 1)
                errno_exit("VIDIOC_QBUF");
            #endif
        }
            
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)){
        #if (PRINT_ENABLE == 1)
            errno_exit("VIDIOC_STREAMON");
        #endif
    }
        
}

/*
 * Uninitializes the memeory map for the camera buffers
 * 
 * Parameters:
 *   None
 *
 * Returns:
 * 	 None 
 */
static void uninit_device(void)
{
    unsigned int i;

    for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap(buffers[i].start, buffers[i].length)){
            #if (PRINT_ENABLE == 1)
                errno_exit("munmap");
            #endif
        }
            

    free(buffers);
}

/*
 * Initializes the memeory map for the camera buffers
 * 
 * Parameters:
 *   None
 *
 * Returns:
 * 	 None 
 */
static void init_mmap(void)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 6;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s does not support "
                            "memory mapping\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            #if (PRINT_ENABLE == 1)
                errno_exit("VIDIOC_REQBUFS");
            #endif
        }
    }

    if (req.count < 2)
    {
        fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
        exit(EXIT_FAILURE);
    }

    buffers = calloc(req.count, sizeof(*buffers));

    if (!buffers)
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)){
            #if (PRINT_ENABLE == 1)
                errno_exit("VIDIOC_QUERYBUF");
            #endif
        }
            

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start =
            mmap(NULL /* start anywhere */,
                 buf.length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start){
            #if (PRINT_ENABLE == 1)
                errno_exit("mmap");
            #endif
        }
            
    }
}

/*
 * Initializes the camera device
 * 
 * The function is responsible for the following functions:
 * 1. Find if the camera has streaming capabilities
 * 2. To set the camera format (including the resolution)
 * 
 * Parameters:
 *   None
 *
 * Returns:
 * 	 None 
 */
static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s is no V4L2 device\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            #if (PRINT_ENABLE == 1)
                errno_exit("VIDIOC_QUERYCAP");
            #endif
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                dev_name);
        exit(EXIT_FAILURE);
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n",
                dev_name);
        exit(EXIT_FAILURE);
    }

    /* Select video input, video standard and tune here. */

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    }
    else
    {
        /* Errors ignored. */
    }

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        fmt.fmt.pix.width = HRES;
        fmt.fmt.pix.height = VRES;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
            #if (PRINT_ENABLE == 1)
                errno_exit("VIDIOC_S_FMT");
            #endif
        }
            
    }
    else
    {
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt)){
            #if (PRINT_ENABLE == 1)
                errno_exit("VIDIOC_G_FMT");
            #endif
        }
            
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    init_mmap();
}

/*
 * Function to close the camera 'file'
 * 
 * The function closes the camera and stores the file descriptor associated
 * or an error code
 * 
 * Parameters:
 *   None
 *
 * Returns:
 * 	 None
 */
static void close_device(void)
{
    if (-1 == close(fd)){
        #if (PRINT_ENABLE == 1)
            errno_exit("close");
        #endif
    }
        

    fd = -1;
}

/*
 * Function to open the camera 'file'
 * 
 * The function opens the camera and stores the file descriptor associated
 * or an error code
 * 
 * Parameters:
 *   None
 *
 * Returns:
 * 	 None
 */
static void open_device(void)
{
    struct stat st;

    if (-1 == stat(dev_name, &st))
    {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode))
    {
        fprintf(stderr, "%s is no device\n", dev_name);
        exit(EXIT_FAILURE);
    }

    fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);

    if (-1 == fd)
    {
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

// Main camera capture logic
// Captures frames till 
int camera_capture()
{
    open_device();
    init_device();
    start_capturing();

    // Keep capturing frames till a SIGINT or SIGTERM signal is not triggered
    // while(is_capture){
    //     capture_frame();
    // }

    // Edit: Capture a single frame
    capture_frame();
    
    is_capture = false;
    stop_capturing();
    uninit_device();
    close_device();
    fprintf(stderr, "\n");

    return 0;
}