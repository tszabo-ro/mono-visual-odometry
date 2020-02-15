#include <optional>
#include <regex>
#include <sys/stat.h>

#include <motion_tracker/web_viewer.h>
#include <crow/app.h>

#include <opencv2/opencv.hpp>

#include <map>
#include <iostream>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <fstream>


static const std::string base64_chars =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
  "abcdefghijklmnopqrstuvwxyz"
  "0123456789+/";

static std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
  std::string ret;
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];

  while (in_len--) {
    char_array_3[i++] = *(bytes_to_encode++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for(i = 0; (i <4) ; i++)
        ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i)
  {
    for(j = i; j < 3; j++)
      char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;

    for (j = 0; (j < i + 1); j++)
      ret += base64_chars[char_array_4[j]];

    while((i++ < 3))
      ret += '=';

  }

  return ret;

}

static std::map<std::string, std::string> getLocalAddresses()
{
  std::map<std::string, std::string> interface_addresses;

  struct ifaddrs * if_addr_struct = nullptr;
  struct ifaddrs * ifa = nullptr;
  void * tmp_addr_ptr =  nullptr;

  getifaddrs(&if_addr_struct);

  for (ifa = if_addr_struct; ifa !=  nullptr; ifa = ifa->ifa_next) {
    if (ifa ->ifa_addr->sa_family == AF_INET) { // check it is IP4
      // is a valid IP4 Address
      tmp_addr_ptr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
      char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmp_addr_ptr, addressBuffer, INET_ADDRSTRLEN);

      std::string key(ifa->ifa_name);
      std::string value(addressBuffer);
      interface_addresses.insert(std::pair<std::string,std::string>(key, value));
    }
  }
  if (if_addr_struct)
  {
    freeifaddrs(if_addr_struct); //remember to free if_addr_struct
  }

  return interface_addresses;
}


WebViewer::~WebViewer()
{
  runner_.join();
  frame_available_.notify_all();
  updater_.join();
}

void WebViewer::run(unsigned int port)
{
  running_ = true;
  runner_ = std::thread([&, port]()
        {
          app_->port(port)
            .run();
          std::cout << "Exit!" << std::endl;
          running_.store(false);
        });

  updater_ = std::thread([&]()
        {
          while (running_)
          {
            std::unique_lock<std::mutex> lock(frame_lock_);
            frame_available_.wait(lock);

            if (frame_)
            {
              auto stash = std::move(frame_.value());
              frame_.reset();
              lock.unlock();

              updateClients(stash);
            }
          }
        });
}

void WebViewer::stop()
{
  app_->stop();
}


static bool fileExists(const std::string& name)
{
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}



static crow::response dispatchResource(const std::string& path, const std::string& server_address)
////////////////
// WARNING: Before using this function, make sure that <path> points to a valid file!
////////////////
{
  std::ifstream file(path);
  std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  return std::regex_replace(str, std::regex("\\$\\{SERVER_ADDR\\}"), server_address);
}

static crow::response findAndDispatchResource(const std::string& path, const std::string& server_address)
{
  if (fileExists(path))
  {
    return dispatchResource(path, server_address);
  }
  if (fileExists(path + ".html"))
  {
    return dispatchResource(path + ".html", server_address);
  }

  return crow::response(404);
}

WebViewer::WebViewer(std::string ext_interface_name)
  : app_(std::make_unique<crow::SimpleApp>())
{
  auto interfaces = getLocalAddresses();
  if (interfaces.count(ext_interface_name) == 0)
  {
    CROW_LOG_ERROR << "Failed to initialize WebViewer. The interface provided (" << ext_interface_name << ") does not exist.";
    exit(0);
  }

  auto interface_addr = interfaces[ext_interface_name];

  CROW_ROUTE((*app_), "/")
    .methods("GET"_method)
      ([interface_addr]()
         {
           std::ifstream index("resources/index.html");
           std::string str((std::istreambuf_iterator<char>(index)), std::istreambuf_iterator<char>());

           return std::regex_replace(str, std::regex("\\$\\{SERVER_ADDR\\}"), interface_addr);
         });

  CROW_ROUTE((*app_), "/ws")
    .websocket()
    .onopen([&](crow::websocket::connection& conn)
              {
                std::lock_guard<std::mutex> _(connections_lock_);
                ws_connections_.insert(&conn);
              })
    .onclose([&](crow::websocket::connection& conn, const std::string& reason)
               {
                 CROW_LOG_INFO << "Connection closed: " << reason;
                 std::lock_guard<std::mutex> _(connections_lock_);
                 ws_connections_.erase(&conn);
               });
//    .onmessage([&](crow::websocket::connection& conn, const std::string& data, bool is_binary)
//                 {
//                   processWebsocketMsg(conn, data, is_binary);
//                 });


  CROW_ROUTE((*app_),"/<string>")
    .methods("GET"_method)
    ([interface_addr](const std::string& resource){
        std::string path = "resources/" + resource;
        if (resource.empty())
        {
          path += "index.html";
        }

        return findAndDispatchResource(path, interface_addr);
    });
}

void WebViewer::updateFrame(const cv::Mat& frame, const std::unordered_map<std::string, std::string>& data)
{
  WSFrame stash;
  stash.frame = frame.clone();
  stash.data = data;

  std::unique_lock<std::mutex> lock(frame_lock_);
  frame_ = std::optional<WSFrame>(std::move(stash));
  lock.unlock();

  frame_available_.notify_all();
}

void WebViewer::updateClients(const WSFrame& stash)
{
  cv::Mat small_img = stash.frame;
  // cv::resize(stash.frame, small_img, cv::Size(0, 0), 0.5, 0.5, cv::INTER_NEAREST);

  std::vector<uchar> image_buffer;
  cv::imencode(".jpeg", small_img, image_buffer, {cv::IMWRITE_JPEG_QUALITY, 30});

  auto img_base64 = base64_encode(image_buffer.data(), image_buffer.size());

  crow::json::wvalue data;
  for (const auto& el : stash.data)
  {
    data[el.first] = el.second;
  }

  crow::json::wvalue response;
  response["stamp"] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  response["img"] = img_base64;
  response["data"] = std::move(data);

  std::lock_guard<std::mutex> _(connections_lock_);
  for (const auto& connection : ws_connections_)
  {
    connection->send_text(dump(response));
  }
}