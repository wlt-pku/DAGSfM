#ifndef _SRC_MAPREDUCE_MAPPER_H_
#define _SRC_MAPREDUCE_MAPPER_H_

namespace DAGSfM {

class Mapper {
 public:
  virtual void Map(const void* input) = 0;
};

}  // namespace DAGSfM

#endif