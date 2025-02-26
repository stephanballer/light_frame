#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

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

std::map<std::string, std::string> defaultConfig = {
    {"num_leds_vertical", "9"},
    {"num_leds_horizontal", "21"},
    {"add_horizontal", "0"},
    {"add_vertical", "0"},
    {"border_thickness", "32"},
    {"skip_pixels_during_average", "4"},
    {"max_fps", "5"},
    {"brightness", "0.2"}};

int num_leds_vertical;
int num_leds_horizontal;
int add_horizontal;
int add_vertical;
int border_thickness;
int skip_pixels_during_average;
int min_millis;
double brightness;

int serial_fd;

bool check_serial_connection(int fd) {
    fd_set read_fds;
    struct timeval timeout;
    char buffer;

    // Initialize the file descriptor set
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    // Set the timeout to 0, so select returns immediately
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int result = select(fd + 1, &read_fds, NULL, NULL, &timeout);
    if (result < 0) {
        return false;
    }

    if (FD_ISSET(fd, &read_fds)) {
        // Try reading a single byte to check the connection
        int n = read(fd, &buffer, 1);
        if (n < 0) {
            return false;
        }
    }

    return true;
}

// Function to initialize serial communication
void init_serial(const char *port) {
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

// cfsetospeed(&tty, B115200); // ESP8266
// cfsetispeed(&tty, B115200); // ESP8266
#ifdef __APPLE__
    cfsetospeed(&tty, 921600);  // ESP32
    cfsetispeed(&tty, 921600);  // ESP32
#else
    cfsetospeed(&tty, B921600);  // ESP32
    cfsetispeed(&tty, B921600);  // ESP32
#endif

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                      // disable break processing
    tty.c_lflag = 0;                             // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                             // no remapping, no delays
    tty.c_cc[VMIN] = 0;                          // read doesn't block
    tty.c_cc[VTIME] = 5;                         // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }
}

// Capture function definitions based on the platform
#ifdef USE_X11
cv::Mat captureScreen(const char *x_display) {
    Display *display = XOpenDisplay(x_display);
    if (!display) {
        fprintf(stderr, "Failed to open X display\n");
        exit(EXIT_FAILURE);
    }

    Window root = DefaultRootWindow(display);
    XWindowAttributes attributes = {};
    XGetWindowAttributes(display, root, &attributes);

    int width = attributes.width;
    int height = attributes.height;

    XImage *img = XGetImage(display, root, 0, 0, width, height, AllPlanes, ZPixmap);
    if (!img) {
        fprintf(stderr, "Failed to capture X image\n");
        XCloseDisplay(display);
        exit(EXIT_FAILURE);
    }

    cv::Mat mat(height, width, CV_8UC4, img->data);
    cv::Mat mat_bgr;
    cvtColor(mat, mat_bgr, cv::COLOR_BGRA2BGR);  // Convert to BGR

    XDestroyImage(img);
    XCloseDisplay(display);

    return mat_bgr;
}
#endif

#ifdef USE_WAYLAND
cv::Mat captureScreen(const char *wl_display, const char *uid) {
    // Open a pipe to grim
    std::ostringstream buffer;
    buffer << "WAYLAND_DISPLAY=" << wl_display << " XDG_RUNTIME_DIR=/run/user/" << uid << " grim -t ppm - 2>/dev/null";
    std::string bufferStr = buffer.str();

    FILE *pipe = popen(bufferStr.c_str(), "r");
    if (!pipe) {
        fprintf(stderr, "Failed to open pipe to grim\n");
        exit(EXIT_FAILURE);
    }

    // Read the PPM header
    char header[100];
    int width, height, maxval;

    // Read the "P6" part
    if (!fgets(header, sizeof(header), pipe)) {
        // fprintf(stderr, "Failed to read PPM header\n");
        pclose(pipe);
        return cv::Mat(720, 1280, CV_8UC3, cv::Scalar(0, 0, 0));
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

    BITMAPINFOHEADER bi = {sizeof(BITMAPINFOHEADER), width, -height, 1, 32, BI_RGB};
    cv::Mat mat(height, width, CV_8UC4);
    GetDIBits(hdcMemDC, hbmScreen, 0, height, mat.data, (BITMAPINFO *)&bi, cv::DIB_RGB_COLORS);

    DeleteObject(hbmScreen);
    DeleteDC(hdcMemDC);
    ReleaseDC(hwnd, hdcScreen);

    cv::Mat mat_bgr;
    cvtColor(mat, mat_bgr, cv::COLOR_BGRA2BGR);  // Convert to BGR

    return mat_bgr;
}
#endif

#ifdef __APPLE__
#include <cstdio>

cv::Mat captureScreen() {
    // Create a temporary file to store the screenshot
    char tempFilePath[] = "/tmp/screen_capture_XXXXXX";
    int fileDescriptor = mkstemp(tempFilePath);
    if (fileDescriptor == -1) {
        fprintf(stderr, "Failed to create temporary file\n");
        exit(EXIT_FAILURE);
    }
    close(fileDescriptor);

    // Execute screencapture command to capture the screen to the temporary file
    std::string command = "screencapture -x ";
    command += tempFilePath;
    int result = system(command.c_str());
    if (result != 0) {
        fprintf(stderr, "Failed to execute screencapture command\n");
        unlink(tempFilePath);  // Delete the temporary file
        exit(EXIT_FAILURE);
    }

    // Read the image using OpenCV
    cv::Mat image = cv::imread(tempFilePath);
    if (image.empty()) {
        fprintf(stderr, "Failed to read captured image\n");
        unlink(tempFilePath);  // Delete the temporary file
        exit(EXIT_FAILURE);
    }

    // Delete the temporary file
    unlink(tempFilePath);

    return image;  // OpenCV imread already provides BGR format
}
#endif

// Function to calculate the average color of a segment
cv::Scalar calculateSegmentAverage(const cv::Mat &img, int startX, int startY, int width, int height) {
    cv::Rect region(startX, startY, width, height);
    cv::Mat roi = img(region);  // Region of interest

    // Calculate the mean color
    cv::Scalar avgColor = mean(roi);

    // Apply brightness
    avgColor[0] *= brightness;
    avgColor[1] *= brightness;
    avgColor[2] *= brightness;

    return avgColor;
}

// Function to calculate the average colors for all segments
void sendFrameAverage(const cv::Mat &img) {
    int segmentHeight = img.rows / (num_leds_vertical + 2 * add_vertical);
    int segmentWidth = img.cols / (num_leds_horizontal + 2 * add_horizontal);
    cv::Scalar color;

    uint8_t num_leds = 2 * (num_leds_horizontal + num_leds_vertical);
    uint8_t buffer[3 * num_leds + 4];
    int index = 0;

    buffer[index++] = 0x3a;
    buffer[index++] = 0x12;
    buffer[index++] = 0xf4;
    buffer[index++] = num_leds;

    // top
    for (int i = 0; i < num_leds_horizontal; i++) {
        color = calculateSegmentAverage(img, (i + add_horizontal) * segmentWidth, 0, segmentWidth, border_thickness);
        buffer[index++] = color[0];
        buffer[index++] = color[1];
        buffer[index++] = color[2];
    }

    // right
    for (int i = 0; i < num_leds_vertical; i++) {
        color = calculateSegmentAverage(img, img.cols - border_thickness, (i + add_vertical) * segmentHeight, border_thickness, segmentHeight);
        buffer[index++] = color[0];
        buffer[index++] = color[1];
        buffer[index++] = color[2];
    }

    // bottom
    for (int i = num_leds_horizontal - 1; i >= 0; i--) {
        color = calculateSegmentAverage(img, (i + add_horizontal) * segmentWidth, img.rows - border_thickness, segmentWidth, border_thickness);
        buffer[index++] = color[0];
        buffer[index++] = color[1];
        buffer[index++] = color[2];
    }

    // left
    for (int i = num_leds_vertical - 1; i >= 0; i--) {
        color = calculateSegmentAverage(img, 0, (i + add_vertical) * segmentHeight, border_thickness, segmentHeight);
        buffer[index++] = color[0];
        buffer[index++] = color[1];
        buffer[index++] = color[2];
    }

    write(serial_fd, buffer, index);
}

long currentMillis() {
    struct timeval tp;

    gettimeofday(&tp, NULL);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

#if defined USE_WAYLAND || defined USE_X11
std::string exec(const char *cmd) {
    std::array<char, 128> buffer;
    std::string result;

    // Define the type for the deleter
    using pipe_ptr = std::unique_ptr<FILE, int (*)(FILE *)>;

    pipe_ptr pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        std::cerr << "popen() failed!" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }

    // Check the exit status of the command
    int exit_status = pclose(pipe.release());
    if (exit_status == -1) {
        std::cerr << "Error while closing the pipe!" << std::endl;
        std::exit(EXIT_FAILURE);
    } else if (WIFEXITED(exit_status)) {
        int return_code = WEXITSTATUS(exit_status);
        if (return_code != 0) {
            std::cerr << "Command failed with exit code: " << return_code << std::endl;
            std::exit(EXIT_FAILURE);
        }
    } else {
        std::cerr << "Command did not terminate normally!" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return result;
}
#endif

template <typename T>
T getValueOrDefault(const std::map<std::string, std::string> &config, const std::string &key, const T &defaultValue) {
    try {
        if constexpr (std::is_same<T, int>::value) {
            return std::stoi(config.at(key));
        } else if constexpr (std::is_same<T, double>::value) {
            return std::stod(config.at(key));
        } else {
            return config.at(key);
        }
    } catch (...) {
        return defaultValue;
    }
}

void createDirectoryIfNotExists(const std::string &dirPath) {
#ifdef _WIN32
    if (CreateDirectory(dirPath.c_str(), NULL) || ERROR_ALREADY_EXISTS == GetLastError()) {
        std::cout << "Directory created or already exists: " << dirPath << std::endl;
    } else {
        std::cerr << "Error: Could not create directory " << dirPath << " - " << GetLastError() << std::endl;
        exit(1);
    }
#else
    struct stat st;
    if (stat(dirPath.c_str(), &st) != 0) {
        // Directory does not exist, create it
        if (mkdir(dirPath.c_str(), 0755) != 0) {
            std::cerr << "Error: Could not create directory " << dirPath << std::endl;
            exit(1);
        }
    } else if (!S_ISDIR(st.st_mode)) {
        std::cerr << "Error: " << dirPath << " exists but is not a directory" << std::endl;
        exit(1);
    }
#endif
}

void readConfig(const std::string &configPath, std::map<std::string, std::string> &config) {
    std::ifstream configFile(configPath);

    if (configFile.is_open()) {
        std::string line;
        while (std::getline(configFile, line)) {
            std::istringstream is_line(line);
            std::string key;
            if (std::getline(is_line, key, '=')) {
                std::string value;
                if (std::getline(is_line, value)) {
                    config[key] = value;
                }
            }
        }
    } else {
        // Ensure the directory exists
        std::string dirPath = configPath.substr(0, configPath.find_last_of('/'));
        createDirectoryIfNotExists(dirPath);

        std::ofstream newConfigFile(configPath);
        if (!newConfigFile.is_open()) {
            std::cerr << "Error: Could not create configuration file at " << configPath << std::endl;
            exit(1);
        }
        for (const auto &pair : defaultConfig) {
            newConfigFile << pair.first << "=" << pair.second << std::endl;
            config[pair.first] = pair.second;
        }
    }

    // Set the configuration values using the helper function
    num_leds_vertical = getValueOrDefault(config, "num_leds_vertical", std::stoi(defaultConfig["num_leds_vertical"]));
    num_leds_horizontal = getValueOrDefault(config, "num_leds_horizontal", std::stoi(defaultConfig["num_leds_horizontal"]));
    add_horizontal = getValueOrDefault(config, "add_horizontal", std::stoi(defaultConfig["add_horizontal"]));
    add_vertical = getValueOrDefault(config, "add_vertical", std::stoi(defaultConfig["add_vertical"]));
    border_thickness = getValueOrDefault(config, "border_thickness", std::stoi(defaultConfig["border_thickness"]));
    skip_pixels_during_average = getValueOrDefault(config, "skip_pixels_during_average", std::stoi(defaultConfig["skip_pixels_during_average"]));
    min_millis = (int)(1000.0 / getValueOrDefault(config, "max_fps", std::stoi(defaultConfig["max_fps"])));
    brightness = getValueOrDefault(config, "brightness", std::stod(defaultConfig["brightness"]));
}

void overrideConfigWithArgs(int argc, char *argv[]) {
    for (int i = 2; i < argc; i += 2) {
        if (!strcmp(argv[i], "-H") || !strcmp(argv[i], "--horizontal-leds")) {
            num_leds_horizontal = atoi(argv[i + 1]);
        } else if (!strcmp(argv[i], "-aH") || !strcmp(argv[i], "--add-horizontal")) {
            add_horizontal = atoi(argv[i + 1]);
        } else if (!strcmp(argv[i], "-V") || !strcmp(argv[i], "--vertical-leds")) {
            num_leds_vertical = atoi(argv[i + 1]);
        } else if (!strcmp(argv[i], "-aV") || !strcmp(argv[i], "--add-vertical")) {
            add_vertical = atoi(argv[i + 1]);
        } else if (!strcmp(argv[i], "-t") || !strcmp(argv[i], "--border-thickness")) {
            border_thickness = atoi(argv[i + 1]);
        } else if (!strcmp(argv[i], "-s") || !strcmp(argv[i], "--skip-pixels")) {
            skip_pixels_during_average = atoi(argv[i + 1]);
        } else if (!strcmp(argv[i], "-b") || !strcmp(argv[i], "--brightness")) {
            brightness = atof(argv[i + 1]);
        } else if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--max-fps")) {
            min_millis = (int)(1000.0 / atof(argv[i + 1]));
        } else {
            const char *help =
                "Usage: %s <serial_device>\n\n"
                "Options:\n"
                "  -h,  --help              print this help\n"
                "  -H,  --horizontal-leds   number of horizontal leds\n"
                "  -aH, --add-horizontal    add virtual leds left and right\n"
                "  -V,  --vertical-leds     number of vertical leds\n"
                "  -aV, --add-vertical      add virtual leds top and bottom\n"
                "  -t,  --border-thickness  border thickness while calculating average color\n"
                "  -s,  --skip-pixels       step size while calculating average color\n"
                "  -b,  --brightness        led brightness\n"
                "  -f,  --max-fps           maximum frames per second\n";

            fprintf(stderr, help, argv[0]);
            exit(EXIT_FAILURE);
        }
    }
}

int main(int argc, char *argv[]) {
    const char *help =
        "Usage: %s <serial_device>\n\n"
        "Options:\n"
        "  -h,  --help              print this help\n"
        "  -H,  --horizontal-leds   number of horizontal leds\n"
        "  -aH, --add-horizontal    add virtual leds left and right\n"
        "  -V,  --vertical-leds     number of vertical leds\n"
        "  -aV, --add-vertical      add virtual leds top and bottom\n"
        "  -t,  --border-thickness  border thickness while calculating average color\n"
        "  -s,  --skip-pixels       step size while calculating average color\n"
        "  -b,  --brightness        led brightness\n"
        "  -f,  --max-fps           maximum frames per second\n";

    if (argc < 2) {
        fprintf(stderr, help, argv[0]);
        return 1;
    }

    if (argc == 2 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) {
        printf(help, argv[0]);
        exit(0);
    }

    std::string configPath;

#ifdef _WIN32
    configPath = "C:\\ProgramData\\BacklightCapture\\config.txt";
#elif __APPLE__
    configPath = "/Library/Application Support/BacklightCapture/config.txt";
#else
    configPath = "/etc/backlightcapture/config.txt";
#endif

    std::map<std::string, std::string> config;
    readConfig(configPath, config);
    printf("Loaded config\n");
    overrideConfigWithArgs(argc, argv);

    init_serial(argv[1]);
    printf("Serial connection established\n");

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

    while (1) {
        long start = currentMillis();

        if (!std::ifstream(argv[1]).good() || !check_serial_connection(serial_fd)) {
            fprintf(stderr, "Serial device disconnected. Exiting...\n");
            close(serial_fd);
            exit(EXIT_FAILURE);
        }

#ifdef USE_X11
        cv::Mat img = captureScreen(x_display.c_str());
#endif
#ifdef USE_WAYLAND
        cv::Mat img = captureScreen(wl_display.c_str(), uid.c_str());
#endif
#if defined __APPLE__ || defined _WIN32
        cv::Mat img = captureScreen();
#endif
        sendFrameAverage(img);

        long duration = currentMillis() - start;

        if (duration < min_millis) {
            usleep((min_millis - duration) * 1000);
        }

        // printf("FPS: %d\n", 1000 / (currentMillis() - start));
    }

    close(serial_fd);
    return 0;
}
