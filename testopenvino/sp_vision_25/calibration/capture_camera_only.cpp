#include <fmt/core.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  // 创建输出文件夹（修复：使用 create_directories）
  std::filesystem::create_directories(output_folder);

  // 初始化相机（无需 CBoard）
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  tools::logger()->info("默认标定板尺寸为10列7行");
  tools::logger()->info("仅相机内参标定模式（无IMU）");
  tools::logger()->info("输出目录: {}", output_folder);

  int count = 0;
  while (true) {
    camera.read(img, timestamp);

    // 显示图像
    auto img_display = img.clone();

    // 识别标定板
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, cv::Size(10, 7), centers_2d);
    cv::drawChessboardCorners(img_display, cv::Size(10, 7), centers_2d, success);

    // 显示计数和提示
    tools::draw_text(img_display, fmt::format("Saved: {}", count), {40, 40}, {0, 255, 0});
    tools::draw_text(img_display, "Press 's' to save, 'q' to quit", {40, 80}, {0, 255, 255});

    cv::resize(img_display, img_display, {}, 0.5, 0.5);
    cv::imshow("Camera Calibration Capture", img_display);

    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 保存图片（无需四元数文件）
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    cv::imwrite(img_path, img);
    tools::logger()->info("[{}] Saved: {}", count, img_path);
  }

  tools::logger()->info("采集完成，共 {} 张图像", count);
  tools::logger()->info("下一步：运行 calibrate_camera 进行内参标定");

  return 0;
}
