/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file graph.h
 *  @brief Graph algorithm using boost library
 *  @author: Kai Ni
 *  @date Jan 11, 2010
 */

#pragma once

#include <map>
#include <unordered_map>

#ifdef USING_BOOST_DEV
#define BOOST_NO_HASH  // to pacify the warnings about depricated headers in boost.graph
 #include <boost/graph/graph_traits.hpp>
 #include <boost/graph/adjacency_list.hpp>
 #include <boost/shared_ptr.hpp>
#endif
#include <queue>
#include <vector>
#include <functional>
#include <gtsam/nonlinear/Values.h>

namespace gtsam {

  // type definitions :
#ifdef USING_BOOST_DEV
  /**
   * SDGraph is undirected graph with variable keys and double edge weights
   */
  template<class KEY>
  class SDGraph: public boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
  boost::property<boost::vertex_name_t, KEY>, boost::property<
  boost::edge_weight_t, double> > {
  public:
    typedef typename boost::graph_traits<SDGraph<KEY> >::vertex_descriptor Vertex;
  };

  template<class KEY>
  class SGraph : public boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
      boost::property<boost::vertex_name_t, KEY> > {
  public:
    typedef typename boost::graph_traits<SGraph<KEY> >::vertex_descriptor Vertex;
  };
#else
//    template<class KEY>
//    class Vertex {
//    public:
//        KEY key;
//        Vertex(KEY f):key(f) {}
//    };


    template<class KEY>
    class Edge {
    public:
        KEY from;
        KEY to;
        double weight;
        int index; // 新增字段，用于记录边的插入顺序

        Edge(KEY f, KEY t, double w, int i) : from(f), to(t), weight(w), index(i) {}

        // Define the comparison operator for edges
        bool operator<(const Edge& other) const {
          return weight < other.weight;
        }
    };

// Custom comparator for the priority queue to make it a min-heap based on edge weight
    template<class KEY>
    struct EdgeComparator {
        bool operator()(const Edge<KEY>& edge1, const Edge<KEY>& edge2) {
          return edge1.weight > edge2.weight; // Greater-than results in a min-heap
        }
    };

// Define the priority queue (min-heap) for edges
    template<class KEY>
    using MinHeap = std::priority_queue<Edge<KEY>, std::vector<Edge<KEY>>, EdgeComparator<KEY>>;

// 无向图的简单实现
    template<class KEY>
    class SDGraph {
    public:
        std::vector<KEY> vertices;
        std::vector<Edge<KEY>> edges;
        int edgesCnt=0;
        // The adjacency list will map a KEY to a list of Edges that start with that KEY
        std::unordered_map<KEY, std::vector<Edge<KEY>>> adjList;

        // 添加顶点，如果顶点已存在则返回其索引
        int add_vertex(KEY key) {
          auto it = std::find(vertices.begin(), vertices.end(), key);
          if (it == vertices.end()) {
            vertices.push_back(key);
            return vertices.size() - 1;
          }
          return it - vertices.begin();
        }

        // 添加边
//        void add_edge(Edge<KEY> edge) {
//          edges.emplace_back(edge);
//          adjList[edge.from].emplace_back(edge);
//          // 由于是无向图，我们需要添加一个反向的边
//          std::swap(edge.from, edge.to);
//          adjList[edge.from].emplace_back(edge);
//        }

        void add_edge(KEY from, KEY to, double weight = 1.0) {
          Edge<KEY> edge(from, to, weight, edgesCnt++);
          edges.emplace_back(edge);
          adjList[from].emplace_back(edge);
          // 由于是无向图，我们需要添加一个反向的边
          Edge<KEY> reverse_edge(to, from, weight, edgesCnt++);
          adjList[to].emplace_back(reverse_edge);
        }

        // 获取与给定顶点相连的所有边
        const std::vector<Edge<KEY>>& getEdgesFrom(const KEY& key) const {
          auto it = adjList.find(key);
          if (it != adjList.end()) {
            return it->second;
          } else {
            static const std::vector<Edge<KEY>> emptyList;
            return emptyList;
          }
        }

    };

    // 有向图的简单实现
    template<class KEY>
    class SGraph {
    public:
        std::vector<KEY> vertices;
        std::vector<Edge<KEY>> edges;

    };
#endif

  //typedef boost::graph_traits<SGraph>::vertex_descriptor SVertex;

  /**
   * Map from variable key to parent key
   */
  template<class KEY>
  class PredecessorMap: public std::map<KEY, KEY> {
  public:
    /** convenience insert so we can pass ints for TypedSymbol keys */
    inline void insert(const KEY& key, const KEY& parent) {
      std::map<KEY, KEY>::insert(std::make_pair(key, parent));
    }
  };

  /**
   * Generate a list of keys from a spanning tree represented by its predecessor map
   */
  template<class KEY>
  std::list<KEY> predecessorMap2Keys(const PredecessorMap<KEY>& p_map);

  /**
   * Convert the factor graph to an SDGraph
   * G = Graph type
   * F = Factor type
   * Key = Key type
   */
    template<class G, class F, class KEY> SDGraph<KEY> toBoostGraph(const G& graph);

  /**
   * Build takes a predecessor map, and builds a directed graph corresponding to the tree.
   * G = Graph type
   * V = Vertex type
   */
  template<class G, class V, class KEY>
  std::tuple<G, V, std::map<KEY,V> >  predecessorMap2Graph(const PredecessorMap<KEY>& p_map);

  /**
   * Compose the poses by following the chain specified by the spanning tree
   */
  template<class G, class Factor, class POSE, class KEY>
  std::shared_ptr<Values>
    composePoses(const G& graph, const PredecessorMap<KEY>& tree, const POSE& rootPose);


  /**
   * find the minimum spanning tree using boost graph library
   */
  template<class G, class KEY, class FACTOR2>
  PredecessorMap<KEY> findMinimumSpanningTree(const G& g) ;

  /**
   * Split the graph into two parts: one corresponds to the given spanning tree,
   * and the other corresponds to the rest of the factors
   */
  template<class G, class KEY, class FACTOR2>
  void split(const G& g, const PredecessorMap<KEY>& tree, G& Ab1, G& Ab2) ;


} // namespace gtsam

#include <gtsam/inference/graph-inl.h>
