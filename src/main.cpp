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
int add_horizontal = 0;
int add_vertical = 0;
int border_thickness = 32;
int skip_pixels_during_average = 4;
int max_millis = 1000 / 5;
double brightness = 0.2;

int serial_fd;

// g++ -o screenshot_leds_grim_serial screenshot_leds_grim_serial.c $(pkg-config --cflags --libs opencv4)

// Function to initialize serial communication
void init_serial(const char *port)
{
    serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0)
    {
        fprintf(stderr, "Error opening %s: %s\n", port, strerror(errno));
        exit(EXIT_FAILURE);
    }

    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0)
    {
        fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN] = 0;                         // read doesn't block
    tty.c_cc[VTIME] = 5;                        // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }
}

// Function to send color data over serial
void send_colors(std::vector<cv::Scalar> leftColors,
                 std::vector<cv::Scalar> rightColors,
                 std::vector<cv::Scalar> topColors,
                 std::vector<cv::Scalar> bottomColors)
{
    std::ostringstream buffer;
    int index = 0;

    // left
    for (int i = num_leds_vertical - 1; i >= 0; i--)
    {
        buffer << index++ << ":" << (int)leftColors[i][0] << "," << (int)leftColors[i][1] << "," << (int)leftColors[i][2] << ";";
    }
    // top
    for (int i = 0; i < num_leds_horizontal; i++)
    {
        buffer << index++ << ":" << (int)topColors[i][0] << "," << (int)topColors[i][1] << "," << (int)topColors[i][2] << ";";
    }
    // right
    for (int i = 0; i < num_leds_vertical; i++)
    {
        buffer << index++ << ":" << (int)rightColors[i][0] << "," << (int)rightColors[i][1] << "," << (int)rightColors[i][2] << ";";
    }
    // bottom
    for (int i = num_leds_horizontal - 1; i >= 0; i--)
    {
        buffer << index++ << ":" << (int)bottomColors[i][0] << "," << (int)bottomColors[i][1] << "," << (int)bottomColors[i][2] << ";";
    }
    buffer << "\n";

    std::string bufferStr = buffer.str();
    write(serial_fd, bufferStr.c_str(), bufferStr.length());
}

// Capture function definitions based on the platform
#ifdef USE_X11
cv::Mat captureScreen(const char *x_display)
{
    Display *display = XOpenDisplay(x_display);
    if (!display)
    {
        fprintf(stderr, "Failed to open X display\n");
        exit(EXIT_FAILURE);
    }

    Window root = DefaultRootWindow(display);
    XWindowAttributes attributes = {};
    XGetWindowAttributes(display, root, &attributes);

    int width = attributes.width;
    int height = attributes.height;

    XImage *img = XGetImage(display, root, 0, 0, width, height, AllPlanes, ZPixmap);
    if (!img)
    {
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
cv::Mat captureScreen(const char *wl_display, const char *uid)
{
    // Open a pipe to grim
    std::ostringstream buffer;
    buffer << "WAYLAND_DISPLAY=" << wl_display << " XDG_RUNTIME_DIR=/run/user/" << uid << " grim -t ppm -";
    std::string bufferStr = buffer.str();

    FILE *pipe = popen(bufferStr.c_str(), "r");
    if (!pipe)
    {
        fprintf(stderr, "Failed to open pipe to grim\n");
        exit(EXIT_FAILURE);
    }

    // Read the PPM header
    char header[100];
    int width, height, maxval;

    // Read the "P6" part
    if (!fgets(header, sizeof(header), pipe))
    {
        fprintf(stderr, "Failed to read PPM header\n");
        pclose(pipe);
        exit(EXIT_FAILURE);
    }

    if (strncmp(header, "P6", 2) != 0)
    {
        fprintf(stderr, "Invalid PPM header\n");
        pclose(pipe);
        exit(EXIT_FAILURE);
    }

    // Read width, height, and maxval
    if (fscanf(pipe, "%d %d\n%d\n", &width, &height, &maxval) != 3)
    {
        fprintf(stderr, "Invalid PPM header format\n");
        pclose(pipe);
        exit(EXIT_FAILURE);
    }

    // Allocate memory for the image
    cv::Mat img(height, width, CV_8UC3);

    // Read the image data
    size_t dataSize = width * height * 3;
    size_t bytesRead = fread(img.data, 1, dataSize, pipe);
    if (bytesRead != dataSize)
    {
        fprintf(stderr, "Failed to read image data\n");
        pclose(pipe);
        exit(EXIT_FAILURE);
    }

    pclose(pipe);
    return img;
}
#endif

#ifdef _WIN32
cv::Mat captureScreen()
{
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

    BITMAPINFOHEADER bi = {sizeof(BITMAPINFOHEADER), width, -height, 1, 32, BI_RGB};
    cv::Mat mat(height, width, CV_8UC4);
    GetDIBits(hdcMemDC, hbmScreen, 0, height, mat.data, (BITMAPINFO *)&bi, cv::DIB_RGB_COLORS);

    DeleteObject(hbmScreen);
    DeleteDC(hdcMemDC);
    ReleaseDC(hwnd, hdcScreen);

    cv::Mat mat_bgr;
    cvtColor(mat, mat_bgr, cv::COLOR_BGRA2BGR); // Convert to BGR

    return mat_bgr;
}
#endif

#ifdef __APPLE__
cv::Mat captureScreen()
{
    CGImageRef screenImage = CGDisplayCreateImage(kCGDirectMainDisplay);
    if (!screenImage)
    {
        fprintf(stderr, "Failed to capture screen image\n");
        exit(EXIT_FAILURE);
    }

    int width = (int)CGImageGetWidth(screenImage);
    int height = (int)CGImageGetHeight(screenImage);

    cv::Mat mat(height, width, CV_8UC4); // CGImage is RGBA

    CGContextRef contextRef = CGBitmapContextCreate(mat.data, width, height, 8, mat.step[0], CGImageGetColorSpace(screenImage), kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
    if (!contextRef)
    {
        CGImageRelease(screenImage);
        fprintf(stderr, "Failed to create bitmap context\n");
        exit(EXIT_FAILURE);
    }

    CGContextDrawImage(contextRef, CGRectMake(0, 0, width, height), screenImage);
    CGContextRelease(contextRef);
    CGImageRelease(screenImage);

    // cv::Mat mat_bgr;
    // cvtColor(mat, mat_bgr, cv::COLOR_RGBA2BGR); // Convert to BGR

    return mat;
}
#endif

// Function to calculate the average color of a segment
cv::Scalar calculateSegmentAverage(const cv::Mat &img, int startX, int startY, int width, int height)
{
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
void calculateAverageColors(const cv::Mat &img)
{
    int segmentHeight = img.rows / (num_leds_vertical + 2 * add_vertical);
    int segmentWidth = img.cols / (num_leds_horizontal + 2 * add_horizontal);

    std::vector<cv::Scalar> leftColors(num_leds_vertical);
    std::vector<cv::Scalar> rightColors(num_leds_vertical);
    std::vector<cv::Scalar> topColors(num_leds_horizontal);
    std::vector<cv::Scalar> bottomColors(num_leds_horizontal);

    // Left and Right borders
    for (int i = 0; i < num_leds_vertical; i++)
    {
        leftColors[i] = calculateSegmentAverage(img, 0, (i + add_vertical) * segmentHeight, border_thickness, segmentHeight);
        rightColors[i] = calculateSegmentAverage(img, img.cols - border_thickness, (i + add_vertical) * segmentHeight, border_thickness, segmentHeight);
    }

    // Top and Bottom borders
    for (int i = 0; i < num_leds_horizontal; i++)
    {
        topColors[i] = calculateSegmentAverage(img, (i + add_horizontal) * segmentWidth, 0, segmentWidth, border_thickness);
        bottomColors[i] = calculateSegmentAverage(img, (i + add_horizontal) * segmentWidth, img.rows - border_thickness, segmentWidth, border_thickness);
    }

    // Send colors over serial
    send_colors(leftColors, rightColors, topColors, bottomColors);
}

long currentMillis()
{
    struct timeval tp;

    gettimeofday(&tp, NULL);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

#if defined USE_WAYLAND || defined USE_X11
std::string exec(const char *cmd)
{
    std::array<char, 128> buffer;
    std::string result;

    // Define the type for the deleter
    using pipe_ptr = std::unique_ptr<FILE, int (*)(FILE *)>;

    pipe_ptr pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}
#endif

int main(int argc, char *argv[])
{
    const char *help = "Usage: %s <serial_device>\n\n"
                       "Options:\n"
                       "  -h,  --help              print this help\n"
                       "  -H,  --horizontal-leds   number of horizontal leds\n"
                       "  -aH, --add-horizontal    add virtual leds left and right\n"
                       "  -V,  --vertical-leds     number of vertical leds\n"
                       "  -aV, --add-vertical      add virtual leds top and bottom\n"
                       "  -t,  --border-thickness  border thickness while calculating average color\n"
                       "  -s,  --step-size         step size while calculating average color\n"
                       "  -b,  --brightness        led brightness\n"
                       "  -f,  --max-fps           maximum frames per second\n";

    if (argc < 2)
    {
        fprintf(stderr, help, argv[0]);
        return 1;
    }
    for (int i = 2; i < argc; i += 2)
    {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help"))
        {
            printf(help, argv[0]);
            exit(0);
        }
        else if (!strcmp(argv[i], "-H") || !strcmp(argv[i], "--horizontal-leds"))
        {
            num_leds_horizontal = atoi(argv[i + 1]);
        }
        else if (!strcmp(argv[i], "-aH") || !strcmp(argv[i], "--add-horizontal"))
        {
            add_horizontal = atoi(argv[i + 1]);
        }
        else if (!strcmp(argv[i], "-V") || !strcmp(argv[i], "--vertical-leds"))
        {
            num_leds_vertical = atoi(argv[i + 1]);
        }
        else if (!strcmp(argv[i], "-aV") || !strcmp(argv[i], "--add-vertical"))
        {
            add_vertical = atoi(argv[i + 1]);
        }
        else if (!strcmp(argv[i], "-t") || !strcmp(argv[i], "--border-thickness"))
        {
            border_thickness = atoi(argv[i + 1]);
        }
        else if (!strcmp(argv[i], "-s") || !strcmp(argv[i], "--skip-pixels"))
        {
            skip_pixels_during_average = atoi(argv[i + 1]);
        }
        else if (!strcmp(argv[i], "-b") || !strcmp(argv[i], "--brightness"))
        {
            brightness = atof(argv[i + 1]);
        }
        else if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--max-fps"))
        {
            max_millis = (int)(1000.0 / atof(argv[i + 1]));
        }
        else
        {
            fprintf(stderr, help, argv[0]);
            return 1;
        }
    }

    init_serial(argv[1]);

#ifdef USE_X11
    std::string x_display = exec("ps e $(pgrep -u $(whoami) Xorg 2>/dev/null) | grep -m1 'DISPLAY' | sed 's/.*DISPLAY=\\([^ ]*\\).*/\\1/'");
    x_display.erase(x_display.find_last_not_of(" \n\r\t") + 1);
#endif

#ifdef USE_WAYLAND
    std::string uid = exec("ps aux | grep -m1 'sway\\|wayland' | awk '{print $1}' | xargs id -u");
    uid.erase(uid.find_last_not_of(" \n\r\t") + 1);
    std::stringstream wl_display_cmd;
    wl_display_cmd << "ls /run/user/" << uid << "/wayland-* | head -n 1 | xargs basename";
    std::string wl_display = exec(wl_display_cmd.str().c_str());
    wl_display.erase(wl_display.find_last_not_of(" \n\r\t") + 1);
#endif

    while (1)
    {
        long start = currentMillis();

#ifdef USE_X11
        cv::Mat img = captureScreen(x_display.c_str());
#endif
#ifdef USE_WAYLAND
        cv::Mat img = captureScreen(wl_display.c_str(), uid.c_str());
#endif
#if not(defined USE_X11 || defined USE_WAYLAND)
        cv::Mat img = captureScreen();
#endif
        calculateAverageColors(img);

        long duration = currentMillis() - start;

        if (duration < max_millis)
        {
            usleep((max_millis - duration) * 1000);
        }

        // printf("FPS: %d\n", 1000 / (currentMillis() - start));
    }

    close(serial_fd);
    return 0;
}
