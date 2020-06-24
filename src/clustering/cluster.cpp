#include "clustering/cluster.h"

#include <fstream>

namespace DAGSfM {

Cluster::Cluster() {
  igraph_vector_init(&i_edges_, 0);
  igraph_vector_init(&i_weights_, 0);
  cluster_num_ = 1;
}

Cluster::~Cluster() {
  igraph_destroy(&igraph_);
  igraph_vector_destroy(&i_edges_);
  igraph_vector_destroy(&i_weights_);
}

int Cluster::ClusterNum() const { return cluster_num_; }

bool Cluster::InitIGraph(const std::vector<std::pair<int, int>>& edges,
                         const std::vector<int>& weights) {
  // std::vector<int> nodes;
  std::unordered_map<int, int> node_mapper;
  for (uint i = 0; i < edges.size(); i++) {
    auto edge = edges[i];
    nodes_.push_back(edge.first);
    nodes_.push_back(edge.second);
  }

  std::sort(nodes_.begin(), nodes_.end());
  nodes_.erase(std::unique(nodes_.begin(), nodes_.end()), nodes_.end());

  const int n = nodes_.size();
  for (uint i = 0; i < n; i++) {
    node_mapper[nodes_[i]] = i;
  }

  // Filling edges in igraph data format.
  for (uint i = 0; i < edges.size(); i++) {
    auto edge = edges[i];
    igraph_vector_push_back(&i_edges_, node_mapper[edge.first]);
    igraph_vector_push_back(&i_edges_, node_mapper[edge.second]);
    igraph_vector_push_back(&i_weights_, weights[i]);
  }

  // Filling graph in igraph data format.
  igraph_create(&igraph_, &i_edges_, (igraph_integer_t)n, IGRAPH_UNDIRECTED);

  return true;
}

bool Cluster::OutputIGraph(const std::string graph_dir,
                           const std::string image_path) const {
  // Write graph in gml format.
  FILE* file = nullptr;
  file = fopen((graph_dir + "/gml.txt").c_str(), "w");
  // igraph_write_graph_dot(&g, stdout);
  igraph_write_graph_gml(&igraph_, file, 0, "graph in cluster");
  fclose(file);

  std::ofstream out_file(graph_dir + "/labels.txt");
  if (!out_file.is_open()) {
    LOG(WARNING) << graph_dir << "/labels.txt can't be opened!";
    return false;
  }
  for (auto node : nodes_) {
    out_file << labels_.at(node) << " ";
  }
  out_file.close();

  // // Invoke python module.
  // Py_Initialize();

  // if (!Py_IsInitialized()) {
  //     LOG(ERROR) << "initialized error";
  //     return false;
  // }

  // PyObject* draw_graph_module(0);
  // draw_graph_module = PyImport_ImportModule("read_igraph.py");

  // if (!draw_graph_module) {
  //     PyErr_Print();
  //     LOG(WARNING) << "can not find read_igraph.py";
  //     return false;
  // } else {
  //     LOG(INFO) << "open Module";
  // }

  // PyObject *pDict = PyModule_GetDict(draw_graph_module);
  // PyObject *pFunc = PyDict_GetItemString(pDict, "drawIGraph");

  // PyObject* args = PyTuple_New(3);
  // PyObject* arg1 = Py_BuildValue("s", file_path.c_str());
  // PyObject* arg2 = Py_BuildValue("s", image_path.c_str());
  // PyTuple_SetItem(args, 0, arg1);
  // PyTuple_SetItem(args, 1, arg2);

  // PyObject_CallObject(pFunc, args);
  // PyErr_Print();

  // Py_DECREF(pFunc);
  // Py_DECREF(pDict);
  // Py_DECREF(draw_graph_module);
  // Py_Finalize();

  return true;
}

}  // namespace DAGSfM