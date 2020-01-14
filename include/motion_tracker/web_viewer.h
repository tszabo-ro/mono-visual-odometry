#ifndef RestWebViewer_h
#define RestWebViewer_h

#include <memory>
#include <unordered_set>
#include <atomic>

#include <thread>
#include <condition_variable>

#include <cpp-toolkit/primitives_2d.h>

#include <opencv2/core/mat.hpp>

namespace crow
{
  template <typename ... Middlewares>
  class Crow;

  using SimpleApp = Crow<>;
}

namespace cv
{
  class Mat;
}

namespace crow::websocket
{
  struct connection;
}

class WebViewer
{
public:
  WebViewer(std::string ext_interface_name);
  ~WebViewer();

  void run(unsigned int port);
  bool running() const { return running_.load(); }
  void stop();
  void updateFrame(const cv::Mat& frame, const Twist& speed);

private:
  struct WSFrame
  {
    cv::Mat frame;
    Twist speed;
  };

  void updateClients(const WSFrame& stash);

private:

  std::unique_ptr<crow::SimpleApp> app_;
  std::atomic_bool running_;
  std::thread runner_;

  std::mutex connections_lock_;
  std::unordered_set<crow::websocket::connection*> ws_connections_;

  std::thread updater_;
  std::optional<WSFrame> frame_;
  std::mutex frame_lock_;
  std::condition_variable frame_available_;
};

#endif
