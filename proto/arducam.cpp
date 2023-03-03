#include <iostream>
#include <istream>
#include <string>
#include <thread>

#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>

#include <opencv2/opencv.hpp>
#include <ArduCamLib.h>
#include <arducam_config_parser.h>
#define USE_SOFT_TRIGGER

ArduCamCfg cameraCfg;
volatile bool _running = true;
bool save_raw = false;
bool save_flag = true;
int color_mode = 0;

namespace ArduCamUtils {

int list_cameras() {
  ArduCamIndexinfo infos[16];
  int num_cameras = ArduCam_scan(infos);

  printf("num_cameras:%d\n", num_cameras);

  char serial[16];
  for (int i = 0; i < num_cameras; i++) {
    unsigned char *buf = infos[i].u8SerialNum;
    sprintf(serial,
            "%c%c%c%c-%c%c%c%c-%c%c%c%c",
            buf[0],
            buf[1],
            buf[2],
            buf[3],
            buf[4],
            buf[5],
            buf[6],
            buf[7],
            buf[8],
            buf[9],
            buf[10],
            buf[11]);
    printf("index:%4d\tSerial: %s\n", infos[i].u8UsbIndex, serial);
  }

  return num_cameras;
}

cv::Mat JPGToMat(Uint8 *bytes, int length) {
  cv::Mat image = cv::Mat(1, length, CV_8UC1, bytes);
  if (length <= 0) {
    image.data = NULL;
    return image;
  }

  image = imdecode(image, cv::IMREAD_ANYCOLOR);
  return image;
}

cv::Mat YUV422toMat(Uint8 *bytes, int width, int height) {
  cv::Mat image = cv::Mat(height, width, CV_8UC2, bytes);
  cv::cvtColor(image, image, cv::COLOR_YUV2BGR_YUYV);
  return image;
}

cv::Mat separationImage(Uint8 *bytes, int width, int height) {
  int width_d = width << 1;
  unsigned char *temp1, *temp2;
  temp1 = (unsigned char *) malloc(width);
  temp2 = (unsigned char *) malloc(width);

  for (int k = 0; k < height; k++) {
    for (int i = 0, j = 0; i < width_d; i += 2) {
      temp1[j] = bytes[i + (k * width_d)];
      temp2[j++] = bytes[i + 1 + (k * width_d)];
    }
    memcpy(bytes + (k * width_d), temp1, width);
    memcpy(bytes + (k * width_d + width), temp2, width);
  }
  cv::Mat image = cv::Mat(height, width_d, CV_8UC1, bytes);
  free(temp1);
  free(temp2);
  return image;
}

#define RGB565_RED 0xf800
#define RGB565_GREEN 0x07e0
#define RGB565_BLUE 0x001f
cv::Mat RGB565toMat(Uint8 *bytes, int width, int height) {
  unsigned char *temp_data, *ptdata, *data, *data_end;

  data = bytes;
  data_end = bytes + (width * height * 2);

  temp_data =
      (unsigned char *) malloc(cameraCfg.u32Width * cameraCfg.u32Height * 3);
  ptdata = temp_data;

  Uint8 r, g, b;
  while (data < data_end) {
    unsigned short temp;

    temp = (*data << 8) | *(data + 1);
    r = (temp & RGB565_RED) >> 8;
    g = (temp & RGB565_GREEN) >> 3;
    b = (temp & RGB565_BLUE) << 3;

    switch (color_mode) {
      case 1:
        *ptdata++ = r;
        *ptdata++ = g;
        *ptdata++ = b;
        break;
      case 0:
      default:
        *ptdata++ = b;
        *ptdata++ = g;
        *ptdata++ = r;
        break;
    }
    data += 2;
  }

  cv::Mat image = cv::Mat(height, width, CV_8UC3);
  memcpy(image.data, temp_data, cameraCfg.u32Height * cameraCfg.u32Width * 3);
  cv::flip(image, image, 0);
  free(temp_data);
  return image;
}

cv::Mat dBytesToMat(Uint8 *bytes, int bit_width, int width, int height) {
  unsigned char *temp_data = (unsigned char *) malloc(width * height);
  int index = 0;
  for (int i = 0; i < width * height * 2; i += 2) {
    unsigned char temp =
        ((bytes[i + 1] << 8 | bytes[i]) >> (bit_width - 8)) & 0xFF;
    temp_data[index++] = temp;
  }
  cv::Mat image = cv::Mat(height, width, CV_8UC1);
  memcpy(image.data, temp_data, cameraCfg.u32Height * cameraCfg.u32Width);
  free(temp_data);
  return image;
}

cv::Mat BytestoMat(Uint8 *bytes, int width, int height) {
  cv::Mat image = cv::Mat(height, width, CV_8UC1, bytes);
  return image;
}

cv::Mat ConvertImage(ArduCamOutData *frameData) {
  cv::Mat rawImage;
  Uint8 *data = frameData->pu8ImageData;
  int height, width;
  width = cameraCfg.u32Width;
  height = cameraCfg.u32Height;

  switch (cameraCfg.emImageFmtMode) {
    case FORMAT_MODE_RGB:
      rawImage = ArduCamUtils::RGB565toMat(data, width, height);
      break;
    case FORMAT_MODE_RAW_D:
      rawImage = ArduCamUtils::separationImage(data, width, height);
      switch (color_mode) {
        case RAW_RG:
          cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
          break;
        case RAW_GR:
          cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGR2BGR);
          break;
        case RAW_GB:
          cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGB2BGR);
          break;
        case RAW_BG:
          cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerBG2BGR);
          break;
        default:
          cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
          break;
      }
      break;
    case FORMAT_MODE_MON_D:
      rawImage = ArduCamUtils::separationImage(data, width, height);
      break;
    case FORMAT_MODE_JPG:
      rawImage = ArduCamUtils::JPGToMat(data, frameData->stImagePara.u32Size);
      break;
    case FORMAT_MODE_RAW:
      if (cameraCfg.u8PixelBytes == 2) {
        rawImage = ArduCamUtils::dBytesToMat(data,
                                             frameData->stImagePara.u8PixelBits,
                                             width,
                                             height);
      } else {
        rawImage = ArduCamUtils::BytestoMat(data, width, height);
      }
      switch (color_mode) {
        case RAW_RG:
          cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
          // cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2GRAY);
          break;
        case RAW_GR:
          cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGR2BGR);
          break;
        case RAW_GB:
          cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGB2BGR);
          break;
        case RAW_BG:
          cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerBG2BGR);
          break;
        default:
          cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
          break;
      }
      break;
    case FORMAT_MODE_YUV:
      rawImage = ArduCamUtils::YUV422toMat(data, width, height);
      break;
    case FORMAT_MODE_MON:
      if (cameraCfg.u8PixelBytes == 2) {
        rawImage = ArduCamUtils::dBytesToMat(data,
                                             frameData->stImagePara.u8PixelBits,
                                             width,
                                             height);
      } else {
        rawImage = ArduCamUtils::BytestoMat(data, width, height);
      }
      break;
    default:
      if (cameraCfg.u8PixelBytes == 2) {
        rawImage = ArduCamUtils::dBytesToMat(data,
                                             frameData->stImagePara.u8PixelBits,
                                             width,
                                             height);
      } else {
        rawImage = ArduCamUtils::BytestoMat(data, width, height);
      }
      cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2RGB);
      break;
  }

  return rawImage;
}

} // namespace ArduCamUtils

class MT9V034 {
public:
  ArduCamHandle handle;

  MT9V034(const int index) {
    // Set camera configuration
    ArduCamCfg cfg = {};
    cfg.u32Width = 640;
    cfg.u32Height = 480;
    cfg.emI2cMode = I2C_MODE_8_16;
    cfg.emImageFmtMode = FORMAT_MODE_RAW;
    cfg.u32I2cAddr = 0x90; // I2C address of camera
    cfg.u8PixelBits = 10;  // Bit width of the image generated by camera
    cfg.u8PixelBytes = 2;  // Number of bytes per pixel
    cfg.u32TransLvl = 0;

    cameraCfg.u32Width = 640;
    cameraCfg.u32Height = 480;
    cameraCfg.emI2cMode = I2C_MODE_8_16;
    cameraCfg.emImageFmtMode = FORMAT_MODE_RAW;
    cameraCfg.u32I2cAddr = 0x90; // I2C address of camera
    cameraCfg.u8PixelBits = 10;  // Bit width of the image generated by camera
    cameraCfg.u8PixelBytes = 2;  // Number of bytes per pixel
    cameraCfg.u32TransLvl = 0;

    // Open camera
    int ret_val = ArduCam_open(handle, &cfg, index);
    if (ret_val != USB_CAMERA_NO_ERROR) {
      printf("FAIL ret_val: %d\n", ret_val);
      return;
    }

    // Set USB-Shield configurations
    // VRCMD = 0xD7, 0x4600, 0x0100, 1, 0x85
    // VRCMD = 0xD7, 0x4600, 0x0200, 1, 0x00
    // VRCMD = 0xD7, 0x4600, 0x0300, 1, 0xC0
    // VRCMD = 0xD7, 0x4600, 0x0300, 1, 0x40
    // VRCMD = 0xD7, 0x4600, 0x0400, 1, 0x00
    // VRCMD = 0xD7, 0x4600, 0x0A00, 1, 0x02
    // VRCMD = 0xF6, 0x0000, 0x0000, 3, 0x03, 0x04, 0x0C
    uint8_t cmd = 0xD7;
    uint8_t buf_all[7][3] = {{0x85}, {0x00}, {0xC0}, {0x40}, {0x00}, {0x02}};
    uint8_t buf_usb2[3] = {0x03, 0x04, 0x0C};
    ArduCam_setboardConfig(handle, cmd, 0x4600, 0x0100, 1, buf_all[0]);
    ArduCam_setboardConfig(handle, cmd, 0x4600, 0x0200, 1, buf_all[1]);
    ArduCam_setboardConfig(handle, cmd, 0x4600, 0x0300, 1, buf_all[2]);
    ArduCam_setboardConfig(handle, cmd, 0x4600, 0x0300, 1, buf_all[3]);
    ArduCam_setboardConfig(handle, cmd, 0x4600, 0x0400, 1, buf_all[4]);
    ArduCam_setboardConfig(handle, cmd, 0x4600, 0x0A00, 1, buf_all[5]);
    ArduCam_setboardConfig(handle, cmd, 0, 0, 3, buf_usb2);

    // Set MT9V034 camera configurations
    ArduCam_writeSensorReg(handle, 0x03, 480);
    ArduCam_writeSensorReg(handle, 0x04, 640);
    ArduCam_writeSensorReg(handle, 0x0D, 0x320);

    // Set Hardware-trigger mode
    int retval = ArduCam_setMode(handle, EXTERNAL_TRIGGER_MODE);
    if (retval == USB_BOARD_FW_VERSION_NOT_SUPPORT_ERROR) {
      printf("Usb board firmware version not support single mode.\n");
      printf("Fail!\n");
    }

    // Read camera serial number
    unsigned char buf[16];
    ArduCam_readUserData(handle, 0x400 - 16, 16, buf);
    printf("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c\n",
           buf[0],
           buf[1],
           buf[2],
           buf[3],
           buf[4],
           buf[5],
           buf[6],
           buf[7],
           buf[8],
           buf[9],
           buf[10],
           buf[11]);
  }

  virtual ~MT9V034() {
    ArduCam_close(handle);
  }
};

long total_frames[16];
void getAndDisplaySingleFrame(ArduCamHandle handle, int index) {
  printf("Take picture.\n");
  char name[50];
  sprintf(name, "ArduCam%d", index);
  cv::namedWindow(name, cv::WINDOW_AUTOSIZE);

  ArduCamOutData *frameData;

  cv::Mat rawImage;

  Uint32 rtn_val = ArduCam_getSingleFrame(handle, frameData);

  if (rtn_val == USB_CAMERA_NO_ERROR) {
    rawImage = ArduCamUtils::ConvertImage(frameData);
    if (!rawImage.data) {
      std::cout << "Convert image fail,No image data \n";
      return;
    }

    total_frames[index]++;
    if (save_flag) {
      char save_path[50];

      sprintf(save_path, "images%d", index);
      if (access(save_path, F_OK) != 0) {
        if (mkdir(save_path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
          printf("mkdir error!\n");
      }

      printf("Camera%d,save image%ld.jpg.\n", index, total_frames[index]);
      char imageName[50];
      sprintf(imageName, "images%d/image%ld.jpg", index, total_frames[index]);

      if (save_raw) {
        char rawName[50];
        sprintf(rawName, "images%d/image%ld.raw", index, total_frames[index]);
        FILE *file = fopen(rawName, "w");
        fwrite(frameData->pu8ImageData,
               1,
               cameraCfg.u32Width * cameraCfg.u32Height,
               file);
        fclose(file);
      }

      cv::imwrite(imageName, rawImage);
    }

    cv::resize(rawImage,
               rawImage,
               cv::Size(640, 480),
               (0, 0),
               (0, 0),
               cv::INTER_LINEAR);
    cv::imshow(name, rawImage);
    cv::waitKey(50);
    // cv::waitKey(50);
    printf("End display.\n");
  } else {
    printf("Take picture fail,ret_val = %d\n", rtn_val);
  }
}

void signal_handle(int signal) {
  if (SIGINT == signal) {
    _running = false;
  }
  usleep(1000 * 500);
  exit(0);
}

int main(int argc, char **argv) {
  // receive Ctrl + C signal
  signal(SIGINT, signal_handle);
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  // get config file name
  const char *config_file_name;
  if (argc > 1) {
    config_file_name = argv[1];
  } else {
    return 0;
  }

  // const auto camera_num = ArduCamUtils::list_cameras();
  // printf("Found %d devices.\n", camera_num);
  MT9V034 camera{0};

  while (_running) {

    // for (int i = 0; i < camera_num; i++) {
    ArduCamHandle &tempHandle = camera.handle;
    if (tempHandle == NULL) {
      continue;
    }
    Uint32 rtn_val = ArduCam_isFrameReady(tempHandle);
    if (rtn_val == 1) {
      getAndDisplaySingleFrame(tempHandle, 0);
    } else {
#ifdef USE_SOFT_TRIGGER
      ArduCam_softTrigger(tempHandle);
#endif
    }
    cv::waitKey(1);
  }

  cv::destroyAllWindows();
  std::cout << std::endl << "Press ENTER to exit..." << std::endl;
  std::string str_key;
  std::getline(std::cin, str_key);
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return 0;
}

