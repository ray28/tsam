/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file graph-inl.h
 * @brief Graph algorithm using boost library
 * @author Kai Ni
 */

#pragma once

#include <stdexcept>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
//#pragma GCC diagnostic ignored "-Wunneeded-internal-declaration"
#endif

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#ifdef USING_BOOST_DEV
 #include <boost/graph/breadth_first_search.hpp>
 #include <boost/graph/prim_minimum_spanning_tree.hpp>
#else
#include <list>
#include <queue>
#include <unordered_map>
#include <map>
#include <memory>
#include <queue>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#endif

#include <gtsam/inference/graph.h>

namespace gtsam {

/* ************************************************************************* */
#ifdef USING_BOOST_DEV
template <class KEY>
class ordering_key_visitor : public boost::default_bfs_visitor {
public:
  ordering_key_visitor(std::list<KEY>& ordering_in) : ordering_(ordering_in) {}
  template <typename Vertex, typename Graph> void discover_vertex(Vertex v, const Graph& g) const {
    KEY key = boost::get(boost::vertex_name, g, v);
    ordering_.push_front(key);
  }
  std::list<KEY>& ordering_;
};
#else

    template <class KEY, class GRAPH>
    class ordering_key_visitor {
    public:
        ordering_key_visitor(std::list<KEY>& ordering_in) : ordering_(ordering_in) {}

        void bfs(const GRAPH& g, typename GRAPH::vertex_descriptor start_vertex) {
          std::queue<typename GRAPH::vertex_descriptor> queue;
          std::unordered_map<typename GRAPH::vertex_descriptor, bool> visited;

          // 初始化访问状态
          for (const auto& vertex : g.vertices) {
            visited[vertex] = false;
          }

          visited[start_vertex] = true;
          queue.push(start_vertex);

          while (!queue.empty()) {
            auto current_vertex = queue.front();
            queue.pop();

            // 在这里处理顶点
            discover_vertex(current_vertex, g);

            // 遍历所有相邻的顶点
            for (const auto& edge : g.edges) {
              if (edge.first == current_vertex && !visited[edge.second]) {
                visited[edge.second] = true;
                queue.push(edge.second);
              }
            }
          }
        }

        void discover_vertex(typename GRAPH::vertex_descriptor v, const GRAPH& g) const {
          KEY key = g.vertices[v].name; // 假设图的顶点存储了键值
          ordering_.push_front(key);
        }

    private:
        std::list<KEY>& ordering_;
    };
#endif

#ifdef USING_BOOST_DEV
/* ************************************************************************* */
template<class KEY>
std::list<KEY> predecessorMap2Keys(const PredecessorMap<KEY>& p_map) {

  typedef typename SGraph<KEY>::Vertex SVertex;

  SGraph<KEY> g;
  SVertex root;
  std::map<KEY, SVertex> key2vertex;
  std::tie(g, root, key2vertex) = gtsam::predecessorMap2Graph<SGraph<KEY>, SVertex, KEY>(p_map);

  // breadth first visit on the graph
  std::list<KEY> keys;
  ordering_key_visitor<KEY> vis(keys);
  boost::breadth_first_search(g, root, boost::visitor(vis));
  return keys;
}
#else
    template<class KEY>
    std::list<KEY> predecessorMap2Keys(const PredecessorMap<KEY>& p_map) {
      KEY SVertex;

      SGraph<KEY> g;
      KEY root;
      std::map<KEY, KEY> key2vertex;
      std::tie(g, root, key2vertex) = gtsam::predecessorMap2Graph<SGraph<KEY>, SVertex, KEY>(p_map);

      // breadth first visit on the graph
      std::list<KEY> keys;
      ordering_key_visitor<KEY, SGraph<KEY>> vis(keys);

      // 自定义的BFS实现
      std::queue<KEY> queue;
      std::set<KEY> visited;

      visited.insert(root);
      queue.push(root);

      while (!queue.empty()) {
        KEY current = queue.front();
        queue.pop();

        // 在这里处理顶点
        vis.discover_vertex(current, g);

        // 遍历所有相邻的顶点
        for (const auto& edge : g.edges) {
          KEY neighbor = (edge.first == current) ? edge.second : edge.first;
          if (visited.find(neighbor) == visited.end()) {
            visited.insert(neighbor);
            queue.push(neighbor);
          }
        }
      }
      return keys;
    }
#endif

#ifdef USING_BOOST_DEV
/* ************************************************************************* */
template<class G, class F, class KEY>
SDGraph<KEY> toBoostGraph(const G& graph) {
  // convert the factor graph to boost graph
  SDGraph<KEY> g;
  typedef typename boost::graph_traits<SDGraph<KEY> >::vertex_descriptor BoostVertex;
  std::map<KEY, BoostVertex> key2vertex;
  typename G::const_iterator itFactor;

  // Loop over the factors
  for(itFactor=graph.begin(); itFactor!=graph.end(); itFactor++) {

    // Ignore factors that are not binary
    if ((*itFactor)->keys().size() != 2)
      continue;

    // Cast the factor to the user-specified factor type F
    std::shared_ptr<F> factor = std::dynamic_pointer_cast<F>(*itFactor);
    // Ignore factors that are not of type F
    if (!factor) continue;

    // Retrieve the 2 keys (nodes) the factor (edge) is incident on
    KEY key1 = factor->keys()[0];
    KEY key2 = factor->keys()[1];

    BoostVertex v1, v2;

    // If key1 is a new key, add it to the key2vertex map, else get the corresponding vertex id
    if (key2vertex.find(key1) == key2vertex.end()) {
         v1 = add_vertex(key1, g);
         key2vertex.insert(std::pair<KEY,KEY>(key1, v1));
       } else
         v1 = key2vertex[key1];

    // If key2 is a new key, add it to the key2vertex map, else get the corresponding vertex id
    if (key2vertex.find(key2) == key2vertex.end()) {
       v2 = add_vertex(key2, g);
       key2vertex.insert(std::pair<KEY,KEY>(key2, v2));
     } else
       v2 = key2vertex[key2];

    // Add an edge with weight 1.0
    boost::property<boost::edge_weight_t, double> edge_property(1.0);  // assume constant edge weight here
    boost::add_edge(v1, v2, edge_property, g);
  }

  return g;
}
#else
    template<class G, class F, class KEY>
    SDGraph<KEY> toBoostGraph(const G& graph) {
      SDGraph<KEY> g;
      std::map<KEY, int> key2vertex; // Map to store the vertex indices
      typename G::const_iterator itFactor;

      // Loop over the factors
      for(itFactor = graph.begin(); itFactor != graph.end(); itFactor++) {
        // Ignore factors that are not binary
        if ((*itFactor)->keys().size() != 2)
          continue;

        // Cast the factor to the user-specified factor type F
        std::shared_ptr<F> factor = std::dynamic_pointer_cast<F>(*itFactor);
        // Ignore factors that are not of type F
        if (!factor) continue;

        // Retrieve the 2 keys (nodes) the factor (edge) is incident on
        KEY key1 = factor->keys()[0];
        KEY key2 = factor->keys()[1];

        int v1, v2;

        // If key1 is a new key, add it to the key2vertex map, else get the corresponding vertex id
        if (key2vertex.find(key1) == key2vertex.end()) {
          v1 = g.add_vertex(key1);
          key2vertex[key1] = v1;
        } else {
          v1 = key2vertex[key1];
        }

        // If key2 is a new key, add it to the key2vertex map, else get the corresponding vertex id
        if (key2vertex.find(key2) == key2vertex.end()) {
          v2 = g.add_vertex(key2);
          key2vertex[key2] = v2;
        } else {
          v2 = key2vertex[key2];
        }

        // Add an edge with weight 1.0
        g.add_edge(key1, key2, 1.0);
      }

      return g;
    }
#endif

/* ************************************************************************* */
#ifdef USING_BOOST_DEV
template<class G, class V, class KEY>
std::tuple<G, V, std::map<KEY,V> >
predecessorMap2Graph(const PredecessorMap<KEY>& p_map) {

  G g;
  std::map<KEY, V> key2vertex;
  V v1, v2, root;
  bool foundRoot = false;
  for(auto child_parent: p_map) {
    KEY child, parent;
    std::tie(child,parent) = child_parent;
    if (key2vertex.find(child) == key2vertex.end()) {
       v1 = add_vertex(child, g);
       key2vertex.insert(std::make_pair(child, v1));
     } else
       v1 = key2vertex[child];

    if (key2vertex.find(parent) == key2vertex.end()) {
       v2 = add_vertex(parent, g);
       key2vertex.insert(std::make_pair(parent, v2));
     } else
       v2 = key2vertex[parent];

    if (child==parent) {
      root = v1;
      foundRoot = true;
    } else
      boost::add_edge(v2, v1, g); // edge is from parent to child
  }

  if (!foundRoot)
    throw std::invalid_argument("predecessorMap2Graph: invalid predecessor map!");
  else
    return std::tuple<G, V, std::map<KEY, V> >(g, root, key2vertex);
}

/* ************************************************************************* */
template <class V, class POSE, class KEY>
class compose_key_visitor : public boost::default_bfs_visitor {

private:
  std::shared_ptr<Values> config_;

public:

  compose_key_visitor(std::shared_ptr<Values> config_in) {config_ = config_in;}

  template <typename Edge, typename Graph> void tree_edge(Edge edge, const Graph& g) const {
    KEY key_from = boost::get(boost::vertex_name, g, boost::source(edge, g));
    KEY key_to = boost::get(boost::vertex_name, g, boost::target(edge, g));
    POSE relativePose = boost::get(boost::edge_weight, g, edge);
    config_->insert(key_to, config_->at<POSE>(key_from).compose(relativePose));
  }

};
#else
    template<class G, class V, class KEY>
    std::tuple<G, V, std::map<KEY,V> >
    predecessorMap2Graph(const PredecessorMap<KEY>& p_map) {

      G g;
      std::map<KEY, V> key2vertex;
      V root;
      bool foundRoot = false;

      for(const auto& child_parent: p_map) {
        KEY child = child_parent.first;
        KEY parent = child_parent.second;

        V v1 = g.add_vertex_if_not_exists(child, key2vertex);
        V v2 = g.add_vertex_if_not_exists(parent, key2vertex);

        if (child == parent) {
          root = v1;
          foundRoot = true;
        } else {
          g.add_edge(v2, v1); // edge is from parent to child
        }
      }

      if (!foundRoot)
        throw std::invalid_argument("predecessorMap2Graph: invalid predecessor map!");

      return std::make_tuple(g, root, key2vertex);
    }
#endif
/* ************************************************************************* */
#ifdef USING_BOOST_DEV
template<class G, class Factor, class POSE, class KEY>
std::shared_ptr<Values> composePoses(const G& graph, const PredecessorMap<KEY>& tree,
    const POSE& rootPose) {

  //TODO: change edge_weight_t to edge_pose_t
  typedef typename boost::adjacency_list<
    boost::vecS, boost::vecS, boost::directedS,
    boost::property<boost::vertex_name_t, KEY>,
    boost::property<boost::edge_weight_t, POSE> > PoseGraph;
  typedef typename boost::graph_traits<PoseGraph>::vertex_descriptor PoseVertex;
  typedef typename boost::graph_traits<PoseGraph>::edge_descriptor PoseEdge;

  PoseGraph g;
  PoseVertex root;
  std::map<KEY, PoseVertex> key2vertex;
  std::tie(g, root, key2vertex) =
      predecessorMap2Graph<PoseGraph, PoseVertex, KEY>(tree);

  // attach the relative poses to the edges
  PoseEdge edge12, edge21;
  bool found1, found2;
  for(typename G::sharedFactor nl_factor: graph) {

    if (nl_factor->keys().size() > 2)
      throw std::invalid_argument("composePoses: only support factors with at most two keys");

    // e.g. in pose2graph, nonlinear factor needs to be converted to pose2factor
    std::shared_ptr<Factor> factor = std::dynamic_pointer_cast<Factor>(nl_factor);
    if (!factor) continue;

    KEY key1 = factor->key1();
    KEY key2 = factor->key2();

    PoseVertex v1 = key2vertex.find(key1)->second;
    PoseVertex v2 = key2vertex.find(key2)->second;

    POSE l1Xl2 = factor->measured();
    std::tie(edge12, found1) = boost::edge(v1, v2, g);
    std::tie(edge21, found2) = boost::edge(v2, v1, g);
    if (found1 && found2) throw std::invalid_argument ("composePoses: invalid spanning tree");
    if (!found1 && !found2) continue;
    if (found1)
      boost::put(boost::edge_weight, g, edge12, l1Xl2);
    else if (found2)
      boost::put(boost::edge_weight, g, edge21, l1Xl2.inverse());
  }

  // compose poses
  std::shared_ptr<Values> config(new Values);
  KEY rootKey = boost::get(boost::vertex_name, g, root);
  config->insert(rootKey, rootPose);
  compose_key_visitor<PoseVertex, POSE, KEY> vis(config);
  boost::breadth_first_search(g, root, boost::visitor(vis));

  return config;
}
#else
    template<class G, class Factor, class POSE, class KEY>
    std::shared_ptr<Values> composePoses(const G& graph, const PredecessorMap<KEY>& tree,
                                         const POSE& rootPose) {
      std::shared_ptr<Values> config(new Values);

      std::queue<KEY> queue;
      std::map<KEY, bool> visited;

      // 初始化
      KEY rootKey = tree.begin()->first; // 假设树的开始是根节点
      config->insert(rootKey, rootPose);
      queue.push(rootKey);
      visited[rootKey] =    true;
      // 使用BFS遍历图
      while (!queue.empty()) {
        KEY currentKey = queue.front();
        queue.pop();
        POSE currentPose = config->at(currentKey);

        // 遍历所有从当前顶点出发的边
        for (const auto& edge : graph.get_edges_from(currentKey)) {
          if (!visited[edge.to]) {
            // 如果顶点尚未访问，则计算其姿态并将其添加到队列中
            POSE composedPose = currentPose.compose(edge.pose);
            config->insert(edge.to, composedPose);
            queue.push(edge.to);
            visited[edge.to] = true;
          }
        }
      }

      return config;
    }
#endif

/* ************************************************************************* */
#ifdef USING_BOOST_DEV
template<class G, class KEY, class FACTOR2>
PredecessorMap<KEY> findMinimumSpanningTree(const G& fg) {

  // Convert to a graph that boost understands
  SDGraph<KEY> g = gtsam::toBoostGraph<G, FACTOR2, KEY>(fg);

  // find minimum spanning tree
  std::vector<typename SDGraph<KEY>::Vertex> p_map(boost::num_vertices(g));
  prim_minimum_spanning_tree(g, &p_map[0]);

  // convert edge to string pairs
  PredecessorMap<KEY> tree;
  typename SDGraph<KEY>::vertex_iterator itVertex = boost::vertices(g).first;
  for(const typename SDGraph<KEY>::Vertex& vi: p_map){
    KEY key = boost::get(boost::vertex_name, g, *itVertex);
    KEY parent = boost::get(boost::vertex_name, g, vi);
    tree.insert(key, parent);
    itVertex++;
  }
  return tree;
}
#else
    template<class KEY>
    void prim_minimum_spanning_tree(const SDGraph<KEY>& graph, std::vector<KEY>& mst) {
      // 确保图不为空
      if (graph.vertices.empty()) return;
      // 使用一个集合来存储已经访问过的顶点
      std::set<KEY> visited;
      // 使用一个优先队列来存储边，按照权重从小到大排序
      std::vector<Edge<KEY>> pq;

      // 从第一个顶点开始
      KEY start_vertex = graph.vertices.front();
      visited.insert(start_vertex);

      // 将与起始顶点相连的边加入优先队列
      for (const auto& edge : graph.getEdgesFrom(start_vertex)) {
        pq.push_back(edge);
      }

      mst.push_back(start_vertex);

      // 当优先队列不为空时，循环执行
      while (!pq.empty() && mst.size() < graph.vertices.size()) {
        // 取出队列中权重最小的边、
        sort(pq.begin(), pq.end());
        Edge<KEY> edge = pq.front();
        pq.erase(pq.begin());

        // 如果这个顶点已经访问过了，就跳过
        if (visited.find(edge.to) != visited.end()) continue;

        // 将这个顶点加入已访问集合
        visited.insert(edge.to);
        // 将这条边加入最小生成树
        mst.push_back(edge.from);

        // 将与这个顶点相连的边加入优先队列
        for (const auto& next_edge : graph.getEdgesFrom(edge.to)) {
          if (visited.find(next_edge.to) == visited.end()) {
            pq.push_back(next_edge);
          }
        }
      }
    }

    template<class G, class KEY, class FACTOR2>
    PredecessorMap<KEY> findMinimumSpanningTree(const G& fg) {

      // Convert to a graph that boost understands
      SDGraph<KEY> graph = gtsam::toBoostGraph<G, FACTOR2, KEY>(fg);

      // find minimum spanning tree
      std::vector<KEY> p_map;
      prim_minimum_spanning_tree(graph, p_map);

      // convert edge to string pairs
      PredecessorMap<KEY> tree;
      int itVertex = 0;
      for(const KEY& vi: p_map){
        KEY key = graph.vertices[itVertex];
        KEY parent = vi;
        tree.insert(key, parent);
        itVertex++;
      }
      return tree;
    }
#endif

/* ************************************************************************* */
template<class G, class KEY, class FACTOR2>
void split(const G& g, const PredecessorMap<KEY>& tree, G& Ab1, G& Ab2) {

  typedef typename G::sharedFactor F ;

  for(const F& factor: g)
  {
    if (factor->keys().size() > 2)
      throw(std::invalid_argument("split: only support factors with at most two keys"));

    if (factor->keys().size() == 1) {
      Ab1.push_back(factor);
      continue;
    }

    std::shared_ptr<FACTOR2> factor2 = std::dynamic_pointer_cast<
        FACTOR2>(factor);
    if (!factor2) continue;

    KEY key1 = factor2->key1();
    KEY key2 = factor2->key2();
    // if the tree contains the key
    if ((tree.find(key1) != tree.end() &&
       tree.find(key1)->second.compare(key2) == 0) ||
        (tree.find(key2) != tree.end() &&
         tree.find(key2)->second.compare(key1)== 0) )
      Ab1.push_back(factor2);
    else
      Ab2.push_back(factor2);
  }
}

}
