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

#include "modules/routing/graph/sub_topo_graph.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "modules/routing/graph/range_utils.h"

namespace apollo {
namespace routing {

namespace {

const double MIN_DIFF_LENGTH = 0.1e-6;             // in meters
const double MIN_INTERNAL_FOR_NODE = 0.01;         // in meters
const double MIN_POTENTIAL_LANE_CHANGE_LEN = 3.0;  // in meters

bool IsCloseEnough(double s1, double s2) {
  return std::fabs(s1 - s2) < MIN_DIFF_LENGTH;
}

void MergeBlockRange(const TopoNode* topo_node,
                     const std::vector<NodeSRange>& origin_range,
                     std::vector<NodeSRange>* block_range) {
  std::vector<NodeSRange> sorted_origin_range;
  sorted_origin_range.insert(sorted_origin_range.end(), origin_range.begin(),
                             origin_range.end());
  sort(sorted_origin_range.begin(), sorted_origin_range.end());
  int cur_index = 0;
  int total_size = static_cast<int>(sorted_origin_range.size());
  while (cur_index < total_size) {
    NodeSRange range(sorted_origin_range[cur_index]);
    ++cur_index;
    while (cur_index < total_size &&
           range.MergeRangeOverlap(sorted_origin_range[cur_index])) {
      ++cur_index;
    }
    if (range.EndS() < topo_node->StartS() ||
        range.StartS() > topo_node->EndS()) {
      continue;
    }
    range.SetStartS(std::max(topo_node->StartS(), range.StartS()));
    range.SetEndS(std::min(topo_node->EndS(), range.EndS()));
    block_range->push_back(std::move(range));
  }
}

void GetSortedValidRange(const TopoNode* topo_node,
                         const std::vector<NodeSRange>& origin_range,
                         std::vector<NodeSRange>* valid_range) {
  std::vector<NodeSRange> block_range;
  MergeBlockRange(topo_node, origin_range, &block_range);
  double start_s = topo_node->StartS();
  double end_s = topo_node->EndS();
  std::vector<double> all_value;
  all_value.push_back(start_s);
  for (const auto& range : block_range) {
    all_value.push_back(range.StartS());
    all_value.push_back(range.EndS());
  }
  all_value.push_back(end_s);
  for (size_t i = 0; i < all_value.size(); i += 2) {
    NodeSRange new_range(all_value[i], all_value[i + 1]);
    valid_range->push_back(std::move(new_range));
  }
}

bool IsReachable(const TopoNode* from_node, const TopoNode* to_node) {
  double start_s = to_node->StartS() / to_node->Length() * from_node->Length();
  start_s = std::max(start_s, from_node->StartS());
  double end_s = to_node->EndS() / to_node->Length() * from_node->Length();
  end_s = std::min(end_s, from_node->EndS());
  return (end_s - start_s > MIN_POTENTIAL_LANE_CHANGE_LEN);
}

}  // namespace

SubTopoGraph::SubTopoGraph(
    const std::unordered_map<const TopoNode*, std::vector<NodeSRange> >&
        black_map) {
  std::vector<NodeSRange> valid_range;
  for (const auto& map_iter : black_map) {
    valid_range.clear();
    // 当前node有一些ranges，找到这些ranges的exclusive_ranges（range之外的区域组成的新ranges）
    GetSortedValidRange(map_iter.first, map_iter.second, &valid_range);
    // 这里第一个输入是原始的node，
    InitSubNodeByValidRange(map_iter.first, valid_range);
  }

  for (const auto& map_iter : black_map) {
    InitSubEdge(map_iter.first);
  }

  // 是否多此一举??
  for (const auto& map_iter : black_map) {
    AddPotentialEdge(map_iter.first);
  }
}

SubTopoGraph::~SubTopoGraph() {}

// 输出的sub_edges的种类和输入的edge的种类保持一致
void SubTopoGraph::GetSubInEdgesIntoSubGraph(
    const TopoEdge* edge,
    std::unordered_set<const TopoEdge*>* const sub_edges) const {
  const auto* from_node = edge->FromNode();
  const auto* to_node = edge->ToNode();
  std::unordered_set<TopoNode*> sub_nodes;
  // 1.
  // from_node是sub_node，或者to_node是sub_node，或者to_node是原始节点、但不包含sub_nodes，
  // 则输入的edge直接作为sub_edges输出。(edge的from_node或者sub_node为次节点，则此edge为次边)
  // 如果from_node、to_node都不是subNode，并且可以找到to_node的subNode，则执行下一步操作（2.）
  if (from_node->IsSubNode() || to_node->IsSubNode() ||
      !GetSubNodes(to_node, &sub_nodes)) {
    sub_edges->insert(edge);
    return;
  }
  // 2.
  // 如果to_node内部含有sub_nodes，则找到指向这些sub_nodes的sub_edges，并且满足sub_edge的from_node匹配。
  // 一种可能：from_node和to_node都是原始node，to_node是goal所在node（含有sub_nodes），
  // edge也是原始edge，但需要找到的是sub_edge(sub_edge:从from_node到sub_node_of_to_node)
  for (const auto* sub_node : sub_nodes) {
    for (const auto* in_edge : sub_node->InFromAllEdge()) {
      if (in_edge->FromNode() == from_node) {
        sub_edges->insert(in_edge);
      }
    }
  }
}

void SubTopoGraph::GetSubOutEdgesIntoSubGraph(
    const TopoEdge* edge,
    std::unordered_set<const TopoEdge*>* const sub_edges) const {
  const auto* from_node = edge->FromNode();
  const auto* to_node = edge->ToNode();
  std::unordered_set<TopoNode*> sub_nodes;
  if (from_node->IsSubNode() || to_node->IsSubNode() ||
      !GetSubNodes(from_node, &sub_nodes)) {
    sub_edges->insert(edge);
    return;
  }
  // get sub_from_node's sub_edge whose to_node is "target_to_node"
  for (const auto* sub_node : sub_nodes) {
    for (const auto* out_edge : sub_node->OutToAllEdge()) {
      if (out_edge->ToNode() == to_node) {
        sub_edges->insert(out_edge);
      }
    }
  }
}

const TopoNode* SubTopoGraph::GetSubNodeWithS(const TopoNode* topo_node,
                                              double s) const {
  const auto& map_iter = sub_node_range_sorted_map_.find(topo_node);
  if (map_iter == sub_node_range_sorted_map_.end()) {
    return topo_node;
  }
  const auto& sorted_vec = map_iter->second;
  // sorted vec can't be empty!
  int index = BinarySearchForStartS(sorted_vec, s);
  if (index < 0) {
    return nullptr;
  }
  return sorted_vec[index].GetTopoNode();
}

// create sub node(with each valid range) and sub edge(if two adjacent sub node is close enough, they will have a sub-edge)
void SubTopoGraph::InitSubNodeByValidRange(
    const TopoNode* topo_node, const std::vector<NodeSRange>& valid_range) {
  // Attention: no matter topo node has valid_range or not,
  // create map value first;
  // sub_node_range_sorted_map_: node -> node(with_range)
  // sub_node_map_: node -> sub_nodes
  auto& sub_node_vec = sub_node_range_sorted_map_[topo_node];
  auto& sub_node_set = sub_node_map_[topo_node];

  // 为原始节点 topo_node 建立sub_node
  std::vector<TopoNode*> sub_node_sorted_vec;
  for (const auto& range : valid_range) {
    if (range.Length() < MIN_INTERNAL_FOR_NODE) {
      continue;
    }
    std::shared_ptr<TopoNode> sub_topo_node_ptr;
    sub_topo_node_ptr.reset(new TopoNode(topo_node, range));
    sub_node_vec.emplace_back(sub_topo_node_ptr.get(), range);
    sub_node_set.insert(sub_topo_node_ptr.get());
    sub_node_sorted_vec.push_back(sub_topo_node_ptr.get());
    topo_nodes_.push_back(std::move(sub_topo_node_ptr)); // maintain created node
  }

  // 为 sub_node 创建边（当前后两个节点间距为0,则建立sub-edge，否则没有边，相当于断开连接）
  for (size_t i = 1; i < sub_node_sorted_vec.size(); ++i) {
    auto* pre_node = sub_node_sorted_vec[i - 1];
    auto* next_node = sub_node_sorted_vec[i];
    if (IsCloseEnough(pre_node->EndS(), next_node->StartS())) {
      Edge edge;
      edge.set_from_lane_id(topo_node->LaneId());
      edge.set_to_lane_id(topo_node->LaneId());
      // 因为前后两个新节点往往在同一个graph node（lane）里
      edge.set_direction_type(Edge::FORWARD);
      edge.set_cost(0.0);
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(new TopoEdge(edge, pre_node, next_node));
      pre_node->AddOutEdge(topo_edge_ptr.get());
      next_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr)); // maintain created edge
    }
  }
}

void SubTopoGraph::InitSubEdge(const TopoNode* topo_node) {
  std::unordered_set<TopoNode*> sub_nodes;
  // 得到 原始节点的 sub node
  if (!GetSubNodes(topo_node, &sub_nodes)) {
    return;
  }

  // 条件符合时，邻居节点有sub_node则为两者的sub_node创建sub_edge，
  // 如果邻居节点没有sub_node，直接和邻居节点建立sub_edge，但是因为邻居节点是“原始节点”，所以他不会存储sub_edge
  for (auto* sub_node : sub_nodes) {
    InitInSubNodeSubEdge(sub_node, topo_node->InFromAllEdge());
    InitOutSubNodeSubEdge(sub_node, topo_node->OutToAllEdge());
  }
}

// 创建 指向sub_node的sub_edge
// input: 
//   origin_edge: 指向“次节点sub_node的origNode”的edges（inEdges）
void SubTopoGraph::InitInSubNodeSubEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* in_edge : origin_edge) {
    // 原始节点的前驱节点（from_node）有sub_nodes时:
    // 如果from_node的sub_nodes 很靠近 输入的sub_node，则创建sub_edge(sub_from_node -> input_sub_node)
    if (GetSubNodes(in_edge->FromNode(), &other_sub_nodes)) {
      for (auto* sub_from_node : other_sub_nodes) {
        // 对于变道关系，overlap考虑变道是否有足够的纵向范围。对于前后关系，overlap考虑两个节点是否紧挨着（止、起位置的差异极小）
        // 如果 sub_from_node 很靠近 sub_node，则为他们建立sub_edge
        if (!sub_from_node->IsOverlapEnough(sub_node, in_edge)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(in_edge->PbEdge(), sub_from_node, sub_node));
        // 每个sub_node会在内部存储sub_edge
        sub_node->AddInEdge(topo_edge_ptr.get());
        sub_from_node->AddOutEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    }
    // 原始节点的前驱节点（from_node）没有sub_nodes时：
    // 如果from_node 很靠近 输入的sub_nod，则建立sub_edge(from_node -> input_sub_node)
    else if (in_edge->FromNode()->IsOverlapEnough(sub_node, in_edge)) {
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(in_edge->PbEdge(), in_edge->FromNode(), sub_node));
      // 原始节点不会存储sub_edge
      sub_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

// 创建 离开sub_node的sub_edge
// input: 
//   origin_edge: 离开“次节点sub_node的origNode”的edges（outEdges）
void SubTopoGraph::InitOutSubNodeSubEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* out_edge : origin_edge) {
    if (GetSubNodes(out_edge->ToNode(), &other_sub_nodes)) {
      for (auto* sub_to_node : other_sub_nodes) {
        if (!sub_node->IsOverlapEnough(sub_to_node, out_edge)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(out_edge->PbEdge(), sub_node, sub_to_node));
        sub_node->AddOutEdge(topo_edge_ptr.get());
        sub_to_node->AddInEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else if (sub_node->IsOverlapEnough(out_edge->ToNode(), out_edge)) {
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(out_edge->PbEdge(), sub_node, out_edge->ToNode()));
      sub_node->AddOutEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

bool SubTopoGraph::GetSubNodes(
    const TopoNode* node,
    std::unordered_set<TopoNode*>* const sub_nodes) const {
  const auto& iter = sub_node_map_.find(node);
  if (iter == sub_node_map_.end()) {
    return false;
  }
  sub_nodes->clear();
  sub_nodes->insert(iter->second.begin(), iter->second.end());
  return true;
}

void SubTopoGraph::AddPotentialEdge(const TopoNode* topo_node) {
  std::unordered_set<TopoNode*> sub_nodes;
  if (!GetSubNodes(topo_node, &sub_nodes)) {
    return;
  }
  for (auto* sub_node : sub_nodes) {
    AddPotentialInEdge(sub_node, topo_node->InFromLeftOrRightEdge());
    AddPotentialOutEdge(sub_node, topo_node->OutToLeftOrRightEdge());
  }
}

void SubTopoGraph::AddPotentialInEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* in_edge : origin_edge) {
    if (GetSubNodes(in_edge->FromNode(), &other_sub_nodes)) {
      for (auto* sub_from_node : other_sub_nodes) {
        if (sub_node->GetInEdgeFrom(sub_from_node) != nullptr) {
          continue; // 存在边
        }
        if (!IsReachable(sub_from_node, sub_node)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(in_edge->PbEdge(), sub_from_node, sub_node));
        sub_node->AddInEdge(topo_edge_ptr.get());
        sub_from_node->AddOutEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else {
      if (sub_node->GetInEdgeFrom(in_edge->FromNode()) != nullptr) {
        continue; // 存在边
      }
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(in_edge->PbEdge(), in_edge->FromNode(), sub_node));
      sub_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

void SubTopoGraph::AddPotentialOutEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* out_edge : origin_edge) {
    if (GetSubNodes(out_edge->ToNode(), &other_sub_nodes)) {
      for (auto* sub_to_node : other_sub_nodes) {
        if (sub_node->GetOutEdgeTo(sub_to_node) != nullptr) {
          continue;
        }
        if (!IsReachable(sub_node, sub_to_node)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(out_edge->PbEdge(), sub_node, sub_to_node));
        sub_node->AddOutEdge(topo_edge_ptr.get());
        sub_to_node->AddInEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else {
      if (sub_node->GetOutEdgeTo(out_edge->ToNode()) != nullptr) {
        continue;
      }
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(out_edge->PbEdge(), sub_node, out_edge->ToNode()));
      sub_node->AddOutEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

}  // namespace routing
}  // namespace apollo
