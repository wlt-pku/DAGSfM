#ifndef _SRC_MAPREDUCE_REDUCER_H_
#define _SRC_MAPREDUCE_REDUCER_H_

namespace DAGSfM {

class Reducer {
 public:
  virtual void Reduce(void* data) = 0;
};

}  // namespace DAGSfM

#endif