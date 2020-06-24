#ifndef _SRC_MAPREDUCE_WORKER_H_
#define _SRC_MAPREDUCE_WORDER_H_

#include <rpc/client.h>
#include <rpc/server.h>
#include <rpc/this_server.h>

#include <fstream>
#include <memory>

#include "map_reduce/running_info.h"
#include "util/misc.h"

namespace DAGSfM {

class Worker {
 public:
  Worker() : server_(rpc::server(Worker::kRPCDefaultPort)) {
    kRPCDefaultPort++;

    server_.bind("StopServer", []() { rpc::this_server().stop(); });

    server_.bind("SaveImage", [](const std::string& image_path,
                                 const std::string& image_name,
                                 const std::vector<char>& buffer, int length) {
      if (!colmap::ExistsDir(image_path)) {
        boost::filesystem::create_directories(image_path);
      }

      std::ofstream of(colmap::JoinPaths(image_path, image_name).c_str(),
                       std::ifstream::out | std::ifstream::binary);
      of.write(buffer.data(), length);
      of.close();

      return true;
    });
  }

  inline void InitializeServer() { server_ = std::move(rpc::server(8080)); }

  inline void RunServer() { server_.run(); }

 protected:
  // uint16_t port_;
  static uint16_t kRPCDefaultPort;
  rpc::server server_;
};

class SfMWorker : public Worker {
 public:
  SfMWorker();

  rpc::server& Server();

  void SetIdle(const bool idle);

  void SetInprogress(const bool in_progress);

  void SetCompleted(const bool is_completed);

  void SetTotalImageNum(const size_t image_num);

  void SetRegImageNum(const size_t reg_image_num);

  void SetTotalMatchingPairs(const size_t matching_pairs);

  void SetMatchedPairs(const size_t matched_pairs);

  const SfMRunningInfo GetRunningInfo() const;

 protected:
  SfMRunningInfo info_;
};

}  // namespace DAGSfM

#endif