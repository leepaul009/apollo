/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/sub_topo_graph.h"
#include "modules/routing/graph/topo_graph.h"
#include "modules/routing/strategy/a_star_strategy.h"

namespace apollo {
namespace routing {
namespace {

struct SearchNode {
  const TopoNode* topo_node = nullptr;
  double f = std::numeric_limits<double>::max();

  SearchNode() = default;
  explicit SearchNode(const TopoNode* node)
      : topo_node(node), f(std::numeric_limits<double>::max()) {}
  SearchNode(const SearchNode& search_node) = default;

  bool operator<(const SearchNode& node) const {
    // in order to let the top of priority queue is the smallest one!
    return f > node.f;
  }

  bool operator==(const SearchNode& node) const {
    return topo_node == node.topo_node;
  }
};

double GetCostToNeighbor(const TopoEdge* edge) {
  return (edge->Cost() + edge->ToNode()->Cost());
}

const TopoNode* GetLargestNode(const std::vector<const TopoNode*>& nodes) {
  double max_range = 0.0;
  const TopoNode* largest = nullptr;
  for (const auto* node : nodes) {
    const double temp_range = node->EndS() - node->StartS();
    if (temp_range > max_range) {
      max_range = temp_range;
      largest = node;
    }
  }
  return largest;
}

bool AdjustLaneChangeBackward(
    std::vector<const TopoNode*>* const result_node_vec) {
  for (int i = static_cast<int>(result_node_vec->size()) - 2; i > 0; --i) {
    const auto* from_node = result_node_vec->at(i);
    const auto* to_node = result_node_vec->at(i + 1);
    const auto* base_node = result_node_vec->at(i - 1);
    const auto* from_to_edge = from_node->GetOutEdgeTo(to_node);
    if (from_to_edge == nullptr) {
      // may need to recalculate edge,
      // because only edge from origin node to subnode is saved
      from_to_edge = to_node->GetInEdgeFrom(from_node);
    }
    if (from_to_edge == nullptr) {
      AERROR << "Get null ptr to edge:" << from_node->LaneId() << " ("
             << from_node->StartS() << ", " << from_node->EndS() << ")"
             << " --> " << to_node->LaneId() << " (" << to_node->StartS()
             << ", " << to_node->EndS() << ")";
      return false;
    }
    if (from_to_edge->Type() != TopoEdgeType::TET_FORWARD) {
      if (base_node->EndS() - base_node->StartS() <
          from_node->EndS() - from_node->StartS()) {
        continue;
      }
      std::vector<const TopoNode*> candidate_set;
      candidate_set.push_back(from_node);
      const auto& out_edges = base_node->OutToLeftOrRightEdge();
      for (const auto* edge : out_edges) {
        const auto* candidate_node = edge->ToNode();
        if (candidate_node == from_node) {
          continue;
        }
        // 备选节点 存在一条到to_node的边
        if (candidate_node->GetOutEdgeTo(to_node) != nullptr) {
          candidate_set.push_back(candidate_node);
        }
      }
      // 从备选节点中，选择最长的节点
      const auto* largest_node = GetLargestNode(candidate_set);
      if (largest_node == nullptr) {
        return false;
      }
      if (largest_node != from_node) {
        result_node_vec->at(i) = largest_node;
      }
    }
  }
  return true;
}

bool AdjustLaneChangeForward(
    std::vector<const TopoNode*>* const result_node_vec) {
  for (size_t i = 1; i < result_node_vec->size() - 1; ++i) {
    const auto* from_node = result_node_vec->at(i - 1);
    const auto* to_node = result_node_vec->at(i);
    const auto* base_node = result_node_vec->at(i + 1);
    const auto* from_to_edge = from_node->GetOutEdgeTo(to_node);
    if (from_to_edge == nullptr) {
      // may need to recalculate edge,
      // because only edge from origin node to subnode is saved
      from_to_edge = to_node->GetInEdgeFrom(from_node);
    }
    if (from_to_edge == nullptr) {
      AERROR << "Get null ptr to edge:" << from_node->LaneId() << " ("
             << from_node->StartS() << ", " << from_node->EndS() << ")"
             << " --> " << to_node->LaneId() << " (" << to_node->StartS()
             << ", " << to_node->EndS() << ")";
      return false;
    }
    if (from_to_edge->Type() != TopoEdgeType::TET_FORWARD) {
      if (base_node->EndS() - base_node->StartS() <
          to_node->EndS() - to_node->StartS()) {
        continue;
      }
      std::vector<const TopoNode*> candidate_set;
      candidate_set.push_back(to_node);
      const auto& in_edges = base_node->InFromLeftOrRightEdge();
      for (const auto* edge : in_edges) {
        const auto* candidate_node = edge->FromNode();
        if (candidate_node == to_node) {
          continue;
        }
        if (candidate_node->GetInEdgeFrom(from_node) != nullptr) {
          candidate_set.push_back(candidate_node);
        }
      }
      const auto* largest_node = GetLargestNode(candidate_set);
      if (largest_node == nullptr) {
        return false;
      }
      if (largest_node != to_node) {
        result_node_vec->at(i) = largest_node;
      }
    }
  }
  return true;
}

bool AdjustLaneChange(std::vector<const TopoNode*>* const result_node_vec) {
  if (result_node_vec->size() < 3) {
    return true;
  }
  if (!AdjustLaneChangeBackward(result_node_vec)) {
    AERROR << "Failed to adjust lane change backward";
    return false;
  }
  if (!AdjustLaneChangeForward(result_node_vec)) {
    AERROR << "Failed to adjust lane change backward";
    return false;
  }
  return true;
}

bool Reconstruct(
    const std::unordered_map<const TopoNode*, const TopoNode*>& came_from,
    const TopoNode* dest_node, std::vector<NodeWithRange>* result_nodes) {
  std::vector<const TopoNode*> result_node_vec;
  result_node_vec.push_back(dest_node);

  auto iter = came_from.find(dest_node);
  while (iter != came_from.end()) {
    result_node_vec.push_back(iter->second); // 父节点（前驱节点）
    iter = came_from.find(iter->second);
  }
  std::reverse(result_node_vec.begin(), result_node_vec.end());
  if (!AdjustLaneChange(&result_node_vec)) {
    AERROR << "Failed to adjust lane change";
    return false;
  }
  result_nodes->clear();
  for (const auto* node : result_node_vec) {
    result_nodes->emplace_back(node->OriginNode(), node->StartS(),
                               node->EndS());
  }
  return true;
}

}  // namespace



/***************** search space for behavior planner : beg *****************/
// [New] member function to compute search space
// TODO: if int is not enough for range
int AStarStrategy::isNodeInRoute(
    const TopoNode* node,
    const size_t start_route_idx,
    const std::vector<std::string>& road_ids_in_route,
    const bool is_forward) const {

  const auto& road_id = node->RoadId();
  if (is_forward)
    for (size_t i=start_route_idx; i < road_ids_in_route.size(); ++i){
      if (road_id == road_ids_in_route[i]){
        return i;
      }
    }
  else
    for (size_t i=start_route_idx; i >= 0; --i){
      if (road_id == road_ids_in_route[i]){
        return i;
      }
    }
  return -1;
}

// [New] member function to compute search space
bool AStarStrategy::GetSearchSpace(
    const TopoNode* src_node,
    const TopoNode* dest_node,
    const SubTopoGraph* sub_graph,
    const std::vector<NodeWithRange>& result_nodes,
    std::unordered_set<const TopoNode*>& search_space_nodes,
    std::unordered_set<const TopoEdge*>& search_space_edges){

  // get all of road id from routing result
  std::unordered_set<std::string> road_set;
  std::vector<std::string> road_ids_in_route;
  for (const auto& node : result_nodes){
    const auto& road_id = node.GetTopoNode()->RoadId();
    if (road_set.count(road_id) == 0){
      road_set.insert(road_id);
      road_ids_in_route.push_back(road_id);
    }
  }

  struct NodeWithRouteId{
    NodeWithRouteId(const TopoNode* n, size_t i) : topo_node(n), route_idx(i) {}
    const TopoNode* topo_node = nullptr;
    size_t route_idx = 0; // road index in the route result
  };

  bool found_dest = false;
  std::unordered_set<const TopoNode*> forward_search_nodes;
  std::unordered_set<const TopoEdge*> forward_search_edges;
  std::unordered_set<const TopoEdge*> sub_edge_set;
  std::deque<NodeWithRouteId> open_list;
  open_list.emplace_back(src_node, 0);
  forward_search_nodes.insert(open_list.front().topo_node);

  // forward search from src_node to dest_node
  while(!open_list.empty()){
    const auto* cur_node = open_list.front().topo_node;
    const auto cur_route_idx = open_list.front().route_idx;
    open_list.pop_front();

    sub_edge_set.clear();
    for (const auto* out_edge : cur_node->OutToAllEdge()){
      sub_graph->GetSubInEdgesIntoSubGraph(out_edge, &sub_edge_set);
    }

    for (const auto* out_edge : sub_edge_set){
      const auto* to_node = out_edge->ToNode();
      // check if to_node is in route
      int route_idx = isNodeInRoute(to_node, cur_route_idx, road_ids_in_route, true);
      if (route_idx == -1){
        continue;
      }
      forward_search_edges.insert(out_edge);
      // ignore to_node that is already visited
      if (forward_search_nodes.count(to_node) == 0){
        open_list.emplace_back(to_node, route_idx);
        forward_search_nodes.insert(to_node);
        if (to_node == dest_node){
          found_dest = true;
        }
      }
    } // end for
  } // end while
  if (!found_dest){
    AERROR << "[GetSearchSpace] failed to find goal when search forward.";
  }

  // backward search
  found_dest = false;
  search_space_nodes.clear();
  search_space_edges.clear();
  open_list.emplace_back(dest_node, road_ids_in_route.size() - 1);
  search_space_nodes.insert(dest_node);
  while(!open_list.empty()){
    const auto* cur_node = open_list.front().topo_node;
    const auto cur_route_idx = open_list.front().route_idx;
    open_list.pop_front();

    sub_edge_set.clear();
    for (const auto* in_edge : cur_node->InFromAllEdge()){
      sub_graph->GetSubOutEdgesIntoSubGraph(in_edge, &sub_edge_set);
    }

    for (const auto* in_edge : sub_edge_set){
      // if in_edge not in forward_search, then it should be ignored
      if (forward_search_edges.count(in_edge) == 0){
        continue;
      }
      const auto* in_node = in_edge->FromNode();
      // check if in_node in route
      int route_index = isNodeInRoute(in_node, cur_route_idx, road_ids_in_route, false); 
      if (route_index == -1){
        continue;
      }
      search_space_edges.insert(in_edge);
      // check if in_node in forward searched node
      if (forward_search_nodes.count(in_node) > 0 && 
          search_space_nodes.count(in_node) == 0){
        open_list.emplace_back(in_node, route_index);
        search_space_nodes.insert(in_node);
        if (in_node == src_node){
          found_dest = true;
        }
      }
    } // end for
  } // end while
  if (!found_dest){
    AERROR << "[GetSearchSpace] failed to get goal when search forward.";
  }
  return true;
}

// [New] member function to compute search space
bool AStarStrategy::GetParallelSearchSpace(
    const TopoNode* src_node, 
    const TopoNode* dest_node, 
    const SubTopoGraph* sub_graph,
    const std::vector<NodeWithRange>& result_nodes,
    std::vector<std::vector<NodeWithRange>>& search_space){
  
  // get all nodes and edges which belong to road-level-route
  std::unordered_set<const TopoNode*> search_space_nodes;
  std::unordered_set<const TopoEdge*> search_space_edges;
  if (!GetSearchSpace(src_node, dest_node, sub_graph, result_nodes,
                      search_space_nodes, search_space_edges)){
    AERROR << "[GetParallelSearchSpace] failed to get search space.";
    return false;
  }

  struct IsLeft{
    bool operator()(const TopoNode* a, const TopoNode* b) const {
      for (const auto* e : a->OutToRightEdge()){
        if (e->ToNode() == b){
          return true;
        }
      }
      for (const auto* e : b->OutToLeftEdge()){
        if (e->ToNode() == a){
          return true;
        }
      }
      // TODO: step for parallel but unconnected nodes
      // as parallel laneSegment ID has order(left->right: -1, -2...)
      size_t pos_a = a->LaneId().find_last_of("_");
      size_t pos_b = b->LaneId().find_last_of("_");
      // TODO: what if cannot find "_"
      if (pos_a == std::string::npos || pos_b == std::string::npos) {
        return false;
      }
      int lid_a = atoi(a->LaneId().substr(pos_a+1).c_str());
      int lid_b = atoi(b->LaneId().substr(pos_b+1).c_str());
      if (lid_a > lid_b){
        return true;
      }
      return false;
    }
  };

  // block: a set of parallel nodes(lane segments)
  std::vector<const TopoNode*> block;
  std::vector<const TopoNode*> prev_block;
  std::unordered_set<const TopoEdge*> next_edges;
  std::unordered_set<const TopoNode*> visited_nodes;
  bool found_dest = false;

  if (!found_dest){
    block.clear();
    visited_nodes.clear();

    // 1. get successors of current block as candidate block
    if (search_space.empty()){
      block.push_back(src_node);
      visited_nodes.insert(src_node);
    } else {
      // get candidates from previous block
      // TODO: how to deal with graph with blackedlist
      // that node in the middle is a subNode
      for (const auto* prev_node : prev_block){
        next_edges.clear();
        for (const auto* e : prev_node->OutToSucEdge()){
          sub_graph->GetSubInEdgesIntoSubGraph(e, &next_edges);
        }

        for (const auto* e : next_edges){
          const auto* to_node = e->ToNode();
          if (visited_nodes.count(to_node) > 0 || 
              search_space_nodes.count(to_node) == 0 ||
              search_space_edges.count(e) == 0){
            continue;
          }
          block.push_back(to_node);
          visited_nodes.insert(to_node);
          if (to_node == dest_node){
            found_dest = true;
          }
        }
      }
      // TODO: check corner case for sort
      // nodes of block should be arranged from left to right
      std::sort(block.begin(), block.end(), IsLeft());
    }

    // 2. extend current block with its neighbor nodes
    for (auto it = block.begin(); it != block.end();){
      const auto* cur_node = *it;
      next_edges.clear();
      for (const auto* e : cur_node->OutToLeftOrRightEdge()){
        sub_graph->GetSubInEdgesIntoSubGraph(e, &next_edges);
      }
      // TODO: check if a orig node left/right-connected with more than 2 sub-nodes
      // 起点、终点node的情况，都是sub-node左右相连，不存在这种情况
      const TopoNode* left; 
      const TopoNode* right;
      for (const auto* e : next_edges){
        const auto* to_node = e->ToNode();
        
        if (search_space_edges.count(e) == 0 ||
            search_space_nodes.count(to_node) == 0 ||
            visited_nodes.count(to_node) != 0){
          continue;
        }
        if (to_node == dest_node){
          found_dest = true;
        }

        if (e->Type() == TopoEdgeType::TET_LEFT){
          left = to_node;
        } else {
          right = to_node;
        }
        visited_nodes.insert(to_node);
      }

      if (left != nullptr || right != nullptr){
        if (right != nullptr){
          it = block.insert(it + 1, right);
        }
        if (left != nullptr){
          it = block.insert(it - (right != nullptr? 1 : 0), left);
        }
        // start from left node for next step
      } else {
        it++;
      }
    }
    
    // 3. add into parallel search space
    // search_space.push_back(std::vector<NodeWithRange>());
    search_space.emplace_back();
    for (const auto* node : block){
      search_space.back().emplace_back(node->OriginNode(), 
                                       node->StartS(), node->EndS());
    }
    prev_block = block;
  }

  return true;
}

/***************** search space for behavior planner : end *****************/


AStarStrategy::AStarStrategy(bool enable_change)
    : change_lane_enabled_(enable_change) {}

void AStarStrategy::Clear() {
  closed_set_.clear();
  open_set_.clear();
  came_from_.clear();
  enter_s_.clear();
  g_score_.clear();
}

double AStarStrategy::HeuristicCost(const TopoNode* src_node,
                                    const TopoNode* dest_node) {
  const auto& src_point = src_node->AnchorPoint();
  const auto& dest_point = dest_node->AnchorPoint();
  double distance = std::fabs(src_point.x() - dest_point.x()) +
                    std::fabs(src_point.y() - dest_point.y());
  return distance;
}

bool AStarStrategy::Search(const TopoGraph* graph,
                           const SubTopoGraph* sub_graph,
                           const TopoNode* src_node, const TopoNode* dest_node,
                           std::vector<NodeWithRange>* const result_nodes) {
  Clear();
  AINFO << "Start A* search algorithm.";

  // 用来选取当前{from_node+f}的容器
  std::priority_queue<SearchNode> open_set_detail;

  SearchNode src_search_node(src_node);
  src_search_node.f = HeuristicCost(src_node, dest_node);
  open_set_detail.push(src_search_node);

  // 和open_set_detail维护相同节点
  open_set_.insert(src_node);
  g_score_[src_node] = 0.0;
  enter_s_[src_node] = src_node->StartS(); // node->s

  SearchNode current_node;
  std::unordered_set<const TopoEdge*> next_edge_set;
  std::unordered_set<const TopoEdge*> sub_edge_set;
  while (!open_set_detail.empty()) {
    current_node = open_set_detail.top();
    const auto* from_node = current_node.topo_node;
    // 0. 找到终点，完成route（内部的节点都是origNode）
    if (current_node.topo_node == dest_node) {
      if (!Reconstruct(came_from_, from_node, result_nodes)) {
        AERROR << "Failed to reconstruct route.";
        return false;
      }
      return true;
    }
    open_set_.erase(from_node);
    open_set_detail.pop();
    // 忽略访问过的节点
    if (closed_set_.count(from_node) != 0) {
      // if showed before, just skip...
      continue;
    }
    closed_set_.emplace(from_node);

    // if residual_s is less than FLAGS_min_length_for_lane_change, only move forward
    // 1. 找到“邻居节点” （控制routing是否要考虑变道）
    const auto& neighbor_edges =
        (GetResidualS(from_node) > FLAGS_min_length_for_lane_change &&
         change_lane_enabled_)
            ? from_node->OutToAllEdge()
            : from_node->OutToSucEdge();
    double tentative_g_score = 0.0;
    next_edge_set.clear();
    for (const auto* edge : neighbor_edges) {
      sub_edge_set.clear();
      // 没有black_listed_lane/road的情况下，sub_edge或者就是原始edge，
      // 或者是终点sub_node的前驱sub_edge。
      sub_graph->GetSubInEdgesIntoSubGraph(edge, &sub_edge_set);
      next_edge_set.insert(sub_edge_set.begin(), sub_edge_set.end());
    }

    // 2. 访问每一个“邻居节点”，更新其信息
    for (const auto* edge : next_edge_set) {
      const auto* to_node = edge->ToNode();
      
      // “访问过的”节点，不会被重新放进open_set_detail（不会再次被访问）
      if (closed_set_.count(to_node) == 1) {
        continue;
      }
      if (GetResidualS(edge, to_node) < FLAGS_min_length_for_lane_change) {
        continue;
      }

      // 1. 计算cost
      // cost = from_node_cost + edge_cost + to_node_cost (这里cost是routing map自带的)
      tentative_g_score =
          g_score_[current_node.topo_node] + GetCostToNeighbor(edge);
      if (edge->Type() != TopoEdgeType::TET_FORWARD) {
        tentative_g_score -= (edge->FromNode()->Cost() + edge->ToNode()->Cost()) / 2;
      }
      // f = cost2come + 启发式cost2go
      double f = tentative_g_score + HeuristicCost(to_node, dest_node);
      // 如果to_node已经被作为“邻居”，并且to_node的新cost更大，则略过此to_node
      if (open_set_.count(to_node) != 0 && f >= g_score_[to_node]) {
        continue;
      }

      // 2. 重置enter_s_
      // if to_node is reached by forward, reset enter_s to start_s
      if (edge->Type() == TopoEdgeType::TET_FORWARD) {
        enter_s_[to_node] = to_node->StartS();
      } else {
        // else, add enter_s with FLAGS_min_length_for_lane_change
        // 用投影的方法，计算to_node上的起始位置。to_node是一个变道node
        double to_node_enter_s =
            (enter_s_[from_node] + FLAGS_min_length_for_lane_change) /
            from_node->Length() * to_node->Length();
        // enter s could be larger than end_s but should be less than length
        to_node_enter_s = std::min(to_node_enter_s, to_node->Length());
        // if enter_s is larger than end_s and to_node is dest_node
        // ？？应该不可能发生
        if (to_node_enter_s > to_node->EndS() && to_node == dest_node) {
          continue;
        }
        enter_s_[to_node] = to_node_enter_s;
      }

      // 3. 更新“邻居节点”的cost
      g_score_[to_node] = f;
      SearchNode next_node(to_node);
      next_node.f = f;
      // 如果to_node已经被作为“邻居”，并且其cost更小，则会插入一个相同node、不同f的next_node到open_set_detail里
      // 如果，f更好的node被访问了，那么f更差的node，因为在closed_set_里，此后不会再被访问了
      open_set_detail.push(next_node);
      // 记录每个“邻居节点”的父节点，用于回溯
      came_from_[to_node] = from_node;
      if (open_set_.count(to_node) == 0) {
        open_set_.insert(to_node);
      }
    }
  }
  AERROR << "Failed to find goal lane with id: " << dest_node->LaneId();
  return false;
}

double AStarStrategy::GetResidualS(const TopoNode* node) {
  double start_s = node->StartS();
  const auto iter = enter_s_.find(node);
  if (iter != enter_s_.end()) {
    // 如果enter_s_在输入node的范围之外，则residual_s=0
    if (iter->second > node->EndS()) {
      // TODO: 合适发生？？
      return 0.0;
    }
    start_s = iter->second;
  } else {
    AWARN << "lane " << node->LaneId() << "(" << node->StartS() << ", "
          << node->EndS() << "not found in enter_s map";
  }
  double end_s = node->EndS();
  const TopoNode* succ_node = nullptr;
  for (const auto* edge : node->OutToAllEdge()) {
    if (edge->ToNode()->LaneId() == node->LaneId()) {
      succ_node = edge->ToNode();
      break;
    }
  }
  if (succ_node != nullptr) {
    end_s = succ_node->EndS();
  }
  return (end_s - start_s);
}

double AStarStrategy::GetResidualS(const TopoEdge* edge,
                                   const TopoNode* to_node) {
  if (edge->Type() == TopoEdgeType::TET_FORWARD) {
    return std::numeric_limits<double>::max();
  }
  double start_s = to_node->StartS();
  const auto* from_node = edge->FromNode();
  const auto iter = enter_s_.find(from_node);
  if (iter != enter_s_.end()) {
    double temp_s = iter->second / from_node->Length() * to_node->Length();
    start_s = std::max(start_s, temp_s);
  } else {
    AWARN << "lane " << from_node->LaneId() << "(" << from_node->StartS()
          << ", " << from_node->EndS() << "not found in enter_s map";
  }
  double end_s = to_node->EndS();
  const TopoNode* succ_node = nullptr;
  for (const auto* edge : to_node->OutToAllEdge()) {
    // 找到在相同lane上的to_node（这个node只有一个，是succ节点）
    if (edge->ToNode()->LaneId() == to_node->LaneId()) {
      succ_node = edge->ToNode();
      break;
    }
  }
  if (succ_node != nullptr) {
    end_s = succ_node->EndS();
  }
  return (end_s - start_s);
}

}  // namespace routing
}  // namespace apollo
