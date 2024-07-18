#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#ifdef USE_X11
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#endif

#ifdef USE_WAYLAND
#include <cstdio>
#endif

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __APPLE__
#include <ApplicationServices/ApplicationServices.h>
#endif


int num_leds_vertical = 9;
int num_leds_horizontal = 21;
int border_thickness = 32;
int skip_pixels_during_average = 4;
int max_millis = 1000 / 5;
double brightness = 0.2;
#ifdef USE_X11
char *output_device = ":0";
#endif
#ifdef USE_WAYLAND
char *output_device = NULL;
#endif


int serial_fd;

//g++ -o screenshot_leds_grim_serial screenshot_leds_grim_serial.c $(pkg-config --cflags --libs opencv4)

// Function to initialize serial communication
void init_serial(const char* port) {
    serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        fprintf(stderr, "Error opening %s: %s\n", port, strerror(errno));
        exit(EXIT_FAILURE);
    }

    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }
}

// Function to send color data over serial
void send_colors(cv::Scalar* leftColors, cv::Scalar* rightColors, cv::Scalar* topColors, cv::Scalar* bottomColors) {
    std::ostringstream buffer;
    int index = 0;

    // left
    for (int i = num_leds_vertical-1; i >= 0; i--) {
        buffer << index++ << ":" << (int)leftColors[i][0] << "," << (int)leftColors[i][1] << "," << (int)leftColors[i][2] << ";";
    }
    // top
    for (int i = 0; i < num_leds_horizontal-2; i++) {
        buffer << index++ << ":" << (int)topColors[i][0] << "," << (int)topColors[i][1] << "," << (int)topColors[i][2] << ";";
    }
    // right
    for (int i = 0; i < num_leds_vertical; i++) {
        buffer << index++ << ":" << (int)rightColors[i][0] << "," << (int)rightColors[i][1] << "," << (int)rightColors[i][2] << ";";
    }
    // bottom
    for (int i = num_leds_horizontal-3; i >= 0 ; i--) {
        buffer << index++ << ":" << (int)bottomColors[i][0] << "," << (int)bottomColors[i][1] << "," << (int)bottomColors[i][2] << ";";
    }
    buffer << "\n";

    std::string bufferStr = buffer.str();
    write(serial_fd, bufferStr.c_str(), bufferStr.length());
}

// Capture function definitions based on the platform
#ifdef USE_X11
cv::Mat captureScreen() {
    Display* display = XOpenDisplay(output_device);
    if (!display) {
        fprintf(stderr, "Failed to open X display\n");
        exit(EXIT_FAILURE);
    }
    
    Window root = DefaultRootWindow(display);
    XWindowAttributes attributes = {};
    XGetWindowAttributes(display, root, &attributes);

    int width = attributes.width;
    int height = attributes.height;

    XImage* img = XGetImage(display, root, 0, 0, width, height, AllPlanes, ZPixmap);
    if (!img) {
        fprintf(stderr, "Failed to capture X image\n");
        XCloseDisplay(display);
        exit(EXIT_FAILURE);
    }

    cv::Mat mat(height, width, CV_8UC4, img->data);
    cv::Mat mat_bgr;
    cvtColor(mat, mat_bgr, cv::COLOR_BGRA2BGR); // Convert to BGR

    XDestroyImage(img);
    XCloseDisplay(display);

    return mat_bgr;
}
#endif

#ifdef USE_WAYLAND
cv::Mat captureScreen() {
    // Open a pipe to grim
    std::ostringstream buffer;
    if (output_device != NULL) {
      buffer << "grim -t ppm -o " << output_device << " -";
    }
    else {
      buffer << "grim -t ppm -";
    }
    std::string bufferStr = buffer.str();

    FILE* pipe = popen(buffer.c_str(), "r");
    if (!pipe) {
        fprintf(stderr, "Failed to open pipe to grim\n");
        exit(EXIT_FAILURE);
    }

    // Read the PPM header
    char header[100];
    int width, height, maxval;
    
    // Read the "P6" part
    if (!fgets(header, sizeof(header), pipe)) {
        fprintf(stderr, "Failed to read PPM header\n");
        pclose(pipe);
        exit(EXIT_FAILURE);
    }
    
    if (strncmp(header, "P6", 2) != 0) {
        fprintf(stderr, "Invalid PPM header\n");
        pclose(pipe);
        exit(EXIT_FAILURE);
    }

    // Read width, height, and maxval
    if (fscanf(pipe, "%d %d\n%d\n", &width, &height, &maxval) != 3) {
        fprintf(stderr, "Invalid PPM header format\n");
        pclose(pipe);
        exit(EXIT_FAILURE);
    }

    // Allocate memory for the image
    cv::Mat img(height, width, CV_8UC3);

    // Read the image data
    size_t dataSize = width * height * 3;
    size_t bytesRead = fread(img.data, 1, dataSize, pipe);
    if (bytesRead != dataSize) {
        fprintf(stderr, "Failed to read image data\n");
        pclose(pipe);
        exit(EXIT_FAILURE);
    }

    pclose(pipe);
    return img;
}
#endif

#ifdef _WIN32
cv::Mat captureScreen() {
    HWND hwnd = GetDesktopWindow();
    HDC hdcScreen = GetDC(hwnd);
    HDC hdcMemDC = CreateCompatibleDC(hdcScreen);

    RECT rc;
    GetClientRect(hwnd, &rc);

    int width = rc.right;
    int height = rc.bottom;

    HBITMAP hbmScreen = CreateCompatibleBitmap(hdcScreen, width, height);
    SelectObject(hdcMemDC, hbmScreen);
    BitBlt(hdcMemDC, 0, 0, width, height, hdcScreen, 0, 0, SRCCOPY);

    BITMAPINFOHEADER bi = { sizeof(BITMAPINFOHEADER), width, -height, 1, 32, BI_RGB };
    cv::Mat mat(height, width, CV_8UC4);
    GetDIBits(hdcMemDC, hbmScreen, 0, height, mat.data, (BITMAPINFO*)&bi, cv::DIB_RGB_COLORS);

    DeleteObject(hbmScreen);
    DeleteDC(hdcMemDC);
    ReleaseDC(hwnd, hdcScreen);

    cv::Mat mat_bgr;
    cvtColor(mat, mat_bgr, cv::COLOR_BGRA2BGR); // Convert to BGR

    return mat_bgr;
}
#endif

#ifdef __APPLE__
cv::Mat captureScreen() {
    CGImageRef screenImage = CGDisplayCreateImage(kCGDirectMainDisplay);
    if (!screenImage) {
        fprintf(stderr, "Failed to capture screen image\n");
        exit(EXIT_FAILURE);
    }

    int width = (int)CGImageGetWidth(screenImage);
    int height = (int)CGImageGetHeight(screenImage);

    cv::Mat mat(height, width, CV_8UC4); // CGImage is RGBA

    CGContextRef contextRef = CGBitmapContextCreate(mat.data, width, height, 8, mat.step[0], CGImageGetColorSpace(screenImage), kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
    if (!contextRef) {
        CGImageRelease(screenImage);
        fprintf(stderr, "Failed to create bitmap context\n");
        exit(EXIT_FAILURE);
    }

    CGContextDrawImage(contextRef, CGRectMake(0, 0, width, height), screenImage);
    CGContextRelease(contextRef);
    CGImageRelease(screenImage);

    cv::Mat mat_bgr;
    cvtColor(mat, mat_bgr, cv::COLOR_RGBA2BGR); // Convert to BGR

    return mat_bgr;
}
#endif

// Function to calculate the average color of a segment
cv::Scalar calculateSegmentAverage(const cv::Mat& img, int startX, int startY, int width, int height) {
    cv::Rect region(startX, startY, width, height);
    cv::Mat roi = img(region); // Region of interest

    // Downsample the region to speed up the mean calculation
    cv::Mat downsampled;
    resize(roi, downsampled, cv::Size(), 1.0 / skip_pixels_during_average, 1.0 / skip_pixels_during_average, cv::INTER_NEAREST);

    // Calculate the mean color
    cv::Scalar avgColor = mean(downsampled);

    // Apply brightness
    avgColor[0] *= brightness;
    avgColor[1] *= brightness;
    avgColor[2] *= brightness;

    return avgColor;
}

// Function to calculate the average colors for all segments
void calculateAverageColors(const cv::Mat& img) {
    int segmentHeight = img.rows / num_leds_vertical;
    int segmentWidth = img.cols / num_leds_horizontal;

    cv::Scalar leftColors[num_leds_vertical];
    cv::Scalar rightColors[num_leds_vertical];
    cv::Scalar topColors[num_leds_horizontal-2];
    cv::Scalar bottomColors[num_leds_horizontal-2];

    // Left and Right borders
    for (int i = 0; i < num_leds_vertical; i++) {
        leftColors[i] = calculateSegmentAverage(img, 0, i * segmentHeight, border_thickness, segmentHeight);
        rightColors[i] = calculateSegmentAverage(img, img.cols - border_thickness, i * segmentHeight, border_thickness, segmentHeight);
    }

    // Top and Bottom borders
    for (int i = 1; i < num_leds_horizontal-1; i++) {
        topColors[i-1] = calculateSegmentAverage(img, i * segmentWidth, 0, segmentWidth, border_thickness);
        bottomColors[i-1] = calculateSegmentAverage(img, i * segmentWidth, img.rows - border_thickness, segmentWidth, border_thickness);
    }

    // Send colors over serial
    send_colors(leftColors, rightColors, topColors, bottomColors);
}

long currentMillis() {
	struct timeval tp;

	gettimeofday(&tp, NULL);
	return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

int main(int argc, char *argv[]) {
    const char *help = "Usage: %s <serial_device>\n\n"
                       "Options:\n"
                       "  -h, --help              print this help\n"
                       "  -H, --horizontal-leds   number of horizontal leds (including 2 corner leds of vertical strip)\n"
                       "  -V, --vertical-leds     number of vertical leds\n"
                       "  -t, --border-thickness  border thickness while calculating average color\n"
                       "  -s, --skip-pixels       number of leds to be skipped while calculating average color\n"
                       "  -b, --brightness        led brightness\n"
#if defined USE_X11 || defined USE_WAYLAND
                       "  -o, --output            display output\n"
#endif
                       "  -f, --max-fps           maximum frames per second\n";
                      

    if (argc < 2) {
        fprintf(stderr, help, argv[0]);
        return 1;
    }
    for (int i = 2; i < argc; i+=2) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf(help, argv[0]);
            exit(0);
        }
        else if (!strcmp(argv[i], "-H") || !strcmp(argv[i], "--horizontal-leds")) {
            num_leds_horizontal = atoi(argv[i+1]);
        }
        else if (!strcmp(argv[i], "-V") || !strcmp(argv[i], "--vertical-leds")) {
            num_leds_vertical = atoi(argv[i+1]);
        }
        else if (!strcmp(argv[i], "-t") || !strcmp(argv[i], "--border-thickness")) {
            border_thickness = atoi(argv[i+1]);
        }
        else if (!strcmp(argv[i], "-s") || !strcmp(argv[i], "--skip-pixels")) {
            skip_pixels_during_average = atoi(argv[i+1]);
        }
        else if (!strcmp(argv[i], "-b") || !strcmp(argv[i], "--brightness")) {
            brightness = atof(argv[i+1]);
        }
        else if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--max-fps")) {
            max_millis = (int)(1000.0/atof(argv[i+1]));
        }
#if defined USE_X11 || defined USE_WAYLAND
        else if (!strcmp(argv[i], "-o") || !strcmp(argv[i], "--output")) {
            output_device = argv[i+1];
        }
#endif
        else {
            fprintf(stderr, help, argv[0]);
        }
    }

    init_serial(argv[1]);

    while (1) {
        long start = currentMillis();

        cv::Mat img = captureScreen();
        calculateAverageColors(img);

        long duration = currentMillis() - start;

        if (duration < max_millis) {
            usleep((max_millis - duration) * 1000);
        }

        //printf("FPS: %d\n", 1000 / (currentMillis() - start));
    }

    close(serial_fd);
    return 0;
}
