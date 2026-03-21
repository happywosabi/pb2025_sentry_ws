#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

void write_q(const std::string & q_path, double w, double x, double y, double z)
{
  std::ofstream q_file(q_path);
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", w, x, y, z);
  q_file.close();
}

void capture_loop(const std::string & config_path, const std::string & output_folder)
{
  // 读取YAML配置中的标定板参数
  auto yaml = YAML::LoadFile(config_path);
  int pattern_cols = yaml["pattern_cols"].as<int>();
  int pattern_rows = yaml["pattern_rows"].as<int>();
  cv::Size pattern_size(pattern_cols, pattern_rows);

  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;
  while (true) {
    camera.read(img, timestamp);

    auto img_display = img.clone();
    tools::draw_text(img_display, "No CAN mode - manual quaternion input", {40, 40}, {0, 255, 0});
    tools::draw_text(
      img_display, fmt::format("Saved: {} images", count), {40, 80}, {0, 255, 0});

    // 检测棋盘格标定板（仅用于预览）
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findChessboardCorners(img, pattern_size, centers_2d,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    if (success) {
      cv::Mat gray;
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(gray, centers_2d, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }
    cv::drawChessboardCorners(img_display, pattern_size, centers_2d, success);
    cv::resize(img_display, img_display, {}, 0.5, 0.5);

    // 按"s"保存图片（随后在终端输入四元数），按"q"退出程序
    cv::imshow("Press s to save, q to quit", img_display);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 在终端手动输入四元数 (wxyz)
    double w, x, y, z;
    fmt::print("\n[{}] 请输入四元数 (w x y z)，空格分隔: ", count + 1);
    std::cout.flush();
    if (!(std::cin >> w >> x >> y >> z)) {
      tools::logger()->warn("输入格式错误，请重新输入");
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }

    // 验证四元数有效性
    double norm_sq = w * w + x * x + y * y + z * z;
    if (std::abs(norm_sq - 1.0) > 0.05) {
      tools::logger()->warn("四元数模长异常: |q|^2 = {:.4f}，已自动归一化", norm_sq);
      double norm = std::sqrt(norm_sq);
      w /= norm;
      x /= norm;
      y /= norm;
      z /= norm;
    }

    // 保存图片和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, w, x, y, z);
    tools::logger()->info("[{}] Saved in {} (q: w={:.4f} x={:.4f} y={:.4f} z={:.4f})",
      count, output_folder, w, x, y, z);
  }
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  std::filesystem::create_directories(output_folder);

  auto yaml = YAML::LoadFile(config_path);
  int pattern_cols = yaml["pattern_cols"].as<int>();
  int pattern_rows = yaml["pattern_rows"].as<int>();

  tools::logger()->info("无CAN模式：通过键盘手动输入四元数 (wxyz)");
  tools::logger()->info("标定板内角点尺寸: {}列{}行", pattern_cols, pattern_rows);
  tools::logger()->info("使用流程:");
  tools::logger()->info("  1. 调整云台到合适位置，使标定板在画面中可见");
  tools::logger()->info("  2. 按 's' 保存当前图像");
  tools::logger()->info("  3. 在终端输入当前云台四元数 (w x y z)");
  tools::logger()->info("  4. 重复步骤直到采集足够图像（建议15-20张）");
  tools::logger()->info("  5. 按 'q' 退出");

  capture_loop(config_path, output_folder);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
