#include <unitree/robot/go2/video/video_client.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace
{
std::atomic<bool> running{true};

void stopHandler(int)
{
  running = false;
}
}

int main(int argc, char **argv)
{
  const std::string network_interface = argc > 1 ? argv[1] : "eth0";
  const std::string output_path = argc > 2 ? argv[2] : "/tmp/go2_front_camera.jpg";
  const double fps = argc > 3 ? std::max(1.0, std::stod(argv[3])) : 4.0;
  const std::string temporary_path = output_path + ".part";

  std::signal(SIGINT, stopHandler);
  std::signal(SIGTERM, stopHandler);
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  unitree::robot::go2::VideoClient video_client;
  video_client.SetTimeout(1.0f);
  video_client.Init();

  std::vector<uint8_t> image;
  const auto period = std::chrono::duration<double>(1.0 / fps);
  std::cerr << "GO2 front camera bridge: " << network_interface << " -> " << output_path << std::endl;
  while (running)
  {
    const auto started = std::chrono::steady_clock::now();
    image.clear();
    if (video_client.GetImageSample(image) == 0 && image.size() > 4 &&
        image[0] == 0xff && image[1] == 0xd8)
    {
      cv::Mat decoded = cv::imdecode(image, cv::IMREAD_COLOR);
      std::vector<uint8_t> web_image;
      if (!decoded.empty())
      {
        const int target_width = std::min(640, decoded.cols);
        const int target_height = std::max(1, decoded.rows * target_width / decoded.cols);
        cv::Mat resized;
        cv::resize(decoded, resized, cv::Size(target_width, target_height),
                   0.0, 0.0, cv::INTER_AREA);
        cv::imencode(".jpg", resized, web_image, {cv::IMWRITE_JPEG_QUALITY, 68});
      }
      std::ofstream file(temporary_path, std::ios::binary | std::ios::trunc);
      if (file)
      {
        const auto &output = web_image.empty() ? image : web_image;
        file.write(reinterpret_cast<const char *>(output.data()), output.size());
        file.close();
        std::rename(temporary_path.c_str(), output_path.c_str());
      }
    }
    const auto elapsed = std::chrono::steady_clock::now() - started;
    if (elapsed < period)
      std::this_thread::sleep_for(period - elapsed);
  }
  std::remove(temporary_path.c_str());
  return 0;
}
