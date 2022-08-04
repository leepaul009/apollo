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
#include "modules/routing/core/result_generator.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "cyber/common/log.h"
#include "modules/common/util/map_util.h"

namespace apollo {
namespace routing {

using apollo::common::util::ContainsKey;

bool IsCloseEnough(double value_1, double value_2) {
  static constexpr double kEpsilon = 1e-6;
  return std::fabs(value_1 - value_2) < kEpsilon;
}

const NodeWithRange& GetLargestRange(
    const std::vector<NodeWithRange>& node_vec) {
  ACHECK(!node_vec.empty());
  size_t result_idx = 0;
  double result_range_length = 0.0;
  for (size_t i = 0; i < node_vec.size(); ++i) {
    if (node_vec[i].Length() > result_range_length) {
      result_range_length = node_vec[i].Length();
      result_idx = i;
    }
  }
  return node_vec[result_idx];
}

/***************** search space for behavior planner:beg *****************/
// generate final routing response
bool ResultGenerator::GenerateRoutingResponse(
    const std::string& map_version, 
    const RoutingRequest& request,
    const std::vector<NodeWithRange>& nodes,
    // const std::unordered_set<const TopoNode*>& search_space_nodes,
    // const std::unordered_set<const TopoEdge*>& search_space_edges,
    // const std::vector<std::vector<const TopoNode*>>& search_space,
    const std::vector<std::vector<NodeWithRange>>& search_space,
    const TopoRangeManager& range_manager, 
    RoutingResponse* const result) {
  
  // extract RoadSegment
  std::vector<std::vector<PassageInfo>> road_segments;
  if (!ExtractBasicRoadSegments(nodes, search_space, &road_segments)){
    return false;
  }

  if (!FillRoutingResponse(road_segments, result)){
    return false;
  }

  result->set_map_version(map_version);
  result->mutable_measurement()->set_distance(CalculateDistance(nodes));
  result->mutable_routing_request()->CopyFrom(request);
  return true;
}

// RoadSegment
bool ResultGenerator::ExtractBasicRoadSegments(
  const std::vector<NodeWithRange>& nodes,
  const std::vector<std::vector<NodeWithRange>>& search_space,
  std::vector<std::vector<PassageInfo>>* const road_segments){

  road_segments->clear();
  std::vector<std::vector<std::vector<NodeWithRange>>> roads; // temp container

  // for high-way case without blacklist,
  // only first and last block has partial range, 
  // while other block should have full range.

  std::vector<std::vector<NodeWithRange>> blocks;
  blocks.push_back(search_space[0]);

  for (size_t i = 1; i < search_space.size(); ++i){
    const auto& prev_block = search_space[i-1];
    const auto& curr_block = search_space[i];

    // compare curr_block with prev_block
    // a) size equal, 1-1 connected(with suc relation):
    bool match_prev = false;
    if (prev_block.size() == curr_block.size()){
      bool is_connected = false;
      for (size_t j = 0; j < prev_block.size(); ++j){
        const auto* prev_node = prev_block[j].GetTopoNode();
        const auto* curr_node = curr_block[j].GetTopoNode();
        
        is_connected = false;
        const auto& suc_edges = prev_node->OutToSucEdge();
        if (const auto* e : suc_edges){
          if (e->ToNode() == curr_node){
            is_connected = true;
            break;
          }
        }
        // if prev_node isn't connected(suc) to curr_node 
        if (!is_connected){
          break;
        }
      }
      match_prev = is_connected;
    }
    // if current block match prev then merge them into same road segment. 
    // otherwise, current block belong to a new road segment.
    if (match_prev) {
      blocks.push_back(curr_block);
    } else {
      roads.push_back(blocks);
      blocks.clear();
      blocks.push_back(curr_block);
    }
  }
  // proccess final road segment
  roads.push_back(blocks);

  // re-orgnize each road segment
  for (const auto& road : roads){
    // get first_block and size from road;
    // size is passage size, create passage(PassageInfo): corresp nodes -> passage
    // create passages as road segment, and insert it into road_segments
    
    std::vector<PassageInfo> passages;
    size_t num_passages = road[0].size();
    for (size_t i = 0; i < num_passages; ++i) {
      std::vector<NodeWithRange> nodes_of_passage;
      for (const auto& block : road) {
        // block[i] is NodeWithRange
        nodes_of_passage.push_back(block[i]);
      }
      // auto change_lane_type = LEFT;
      passages.emplace_back(nodes_of_passage, FORWARD);
    }
    road_segments->push_back(std::move(passages));
  }
  return true;
}


bool ResultGenerator::FillRoutingResponse(
  const std::vector<std::vector<PassageInfo>>& road_segments,
  RoutingResponse* const result){
  
  for (const auto& passages : road_segments){
    auto* road = result->add_road();
    const auto& road_id = passages[0].nodes[0].RoadId();
    road->set_id(road_id);

    for (const auto& passage : passages) {
      auto* passage = road->add_passage();
      LaneNodesToPassageRegion(
        passage.nodes.cbegin(), passage.nodes.cend(), passage);
      passage->set_change_lane_type(FORWARD);
      passage->set_can_exit(true);
    }
  }
  reutrn true;
}

/***************** search space for behavior planner:end *****************/

// 对于passage内的节点，他们之间的edge都是forward关系，
// 前一个passage的最后节点，和后一个passage的首个节点，他们之间是变道关系
bool ResultGenerator::ExtractBasicPassages(
    const std::vector<NodeWithRange>& nodes,
    std::vector<PassageInfo>* const passages) {
  ACHECK(!nodes.empty());
  passages->clear();

  std::vector<NodeWithRange> nodes_of_passage;
  nodes_of_passage.push_back(nodes.at(0));

  for (size_t i = 1; i < nodes.size(); ++i) 
  {
    // 得到节点i-1到节点i的边
    auto edge =
        nodes.at(i - 1).GetTopoNode()->GetOutEdgeTo(nodes.at(i).GetTopoNode());
    if (edge == nullptr) {
      AERROR << "Get null pointer to edge from " << nodes.at(i - 1).LaneId()
             << " to " << nodes.at(i).LaneId();
      return false;
    }
    // 如果节点i-1到节点i的边 是“变道”，则点i-1为止的节点为一个passage，节点i为下一个passage的开始节点
    // 保存当前passage，标记“它到下一个passage的关系”是左或右变道
    if (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT) {
      auto change_lane_type = LEFT;
      if (edge->Type() == TET_RIGHT) {
        change_lane_type = RIGHT;
      }
      passages->emplace_back(nodes_of_passage, change_lane_type);
      nodes_of_passage.clear();
    }
    nodes_of_passage.push_back(nodes.at(i));
  }
  passages->emplace_back(nodes_of_passage, FORWARD);
  return true;
}

bool ResultGenerator::IsReachableFromWithChangeLane(
    const TopoNode* from_node, const PassageInfo& to_nodes,
    NodeWithRange* reachable_node) {
  for (const auto& to_node : to_nodes.nodes) {
    auto edge = to_node.GetTopoNode()->GetInEdgeFrom(from_node);
    // 存在一条从from_node到to_node的edge，并且其类型是变道
    if (edge != nullptr &&
        (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT)) {
      *reachable_node = to_node;
      return true;
    }
  }
  return false;
}

bool ResultGenerator::IsReachableToWithChangeLane(
    const TopoNode* to_node, const PassageInfo& from_nodes,
    NodeWithRange* reachable_node) {
  for (const auto& from_node : from_nodes.nodes) {
    auto edge = from_node.GetTopoNode()->GetOutEdgeTo(to_node);
    if (edge != nullptr &&
        (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT)) {
      *reachable_node = from_node;
      return true;
    }
  }
  return false;
}

void ResultGenerator::ExtendBackward(const TopoRangeManager& range_manager,
                                     const PassageInfo& prev_passage,
                                     PassageInfo* const curr_passage) {
  std::unordered_set<const TopoNode*> node_set_of_curr_passage;
  for (const auto& node : curr_passage->nodes) {
    node_set_of_curr_passage.insert(node.GetTopoNode());
  }
  auto& front_node = curr_passage->nodes.front();
  // if front node starts at middle
  if (!IsCloseEnough(front_node.StartS(), 0.0)) {
    if (!range_manager.Find(front_node.GetTopoNode())) {
      if (IsCloseEnough(prev_passage.nodes.front().StartS(), 0.0)) {
        front_node.SetStartS(0.0);
      } else {
        double temp_s = prev_passage.nodes.front().StartS() /
                        prev_passage.nodes.front().FullLength() *
                        front_node.FullLength();
        front_node.SetStartS(temp_s);
      }
    } else {
      return;
    }
  }

  bool allowed_to_explore = true;
  while (allowed_to_explore) {
    std::vector<NodeWithRange> pred_set;
    for (const auto& edge :
         curr_passage->nodes.front().GetTopoNode()->InFromPreEdge()) {
      const auto& pred_node = edge->FromNode();

      // if pred node has been inserted
      if (ContainsKey(node_set_of_curr_passage, pred_node)) {
        continue;
      }
      // if pred node is reachable from prev passage
      NodeWithRange reachable_node(pred_node, 0, 1);
      if (IsReachableToWithChangeLane(pred_node, prev_passage,
                                      &reachable_node)) {
        const auto* pred_range = range_manager.Find(pred_node);
        if (pred_range != nullptr && !pred_range->empty()) {
          double black_s_end = pred_range->back().EndS();
          if (!IsCloseEnough(black_s_end, pred_node->Length())) {
            pred_set.emplace_back(pred_node, black_s_end, pred_node->Length());
          }
        } else {
          pred_set.emplace_back(pred_node, 0.0, pred_node->Length());
        }
      }
    }
    if (pred_set.empty()) {
      allowed_to_explore = false;
    } else {
      allowed_to_explore = true;
      const auto& node_to_insert = GetLargestRange(pred_set);
      curr_passage->nodes.insert(curr_passage->nodes.begin(), node_to_insert);
      node_set_of_curr_passage.emplace(node_to_insert.GetTopoNode());
    }
  }
}

void ResultGenerator::ExtendForward(const TopoRangeManager& range_manager,
                                    const PassageInfo& next_passage,
                                    PassageInfo* const curr_passage) {
  // 维护了当前passage的节点、以及向前方扩展的节点
  std::unordered_set<const TopoNode*> node_set_of_curr_passage;
  for (const auto& node : curr_passage->nodes) {
    node_set_of_curr_passage.insert(node.GetTopoNode());
  }

  // 1. 如果back_node是次节点，则更新它的endS
  auto& back_node = curr_passage->nodes.back();
  if (!IsCloseEnough(back_node.EndS(), back_node.FullLength())) { // 说明back_node是次节点
    if (!range_manager.Find(back_node.GetTopoNode())) { // 次节点不可能在range_manager里（只存储起止节点和black_listed节点）

      if (IsCloseEnough(next_passage.nodes.back().EndS(),
                        next_passage.nodes.back().FullLength())) {
        back_node.SetEndS(back_node.FullLength());
      } else {
        double adjusted_end_s = next_passage.nodes.back().EndS() /
                                next_passage.nodes.back().FullLength() *
                                back_node.FullLength();
        if (adjusted_end_s > back_node.StartS()) {
          adjusted_end_s = std::min(adjusted_end_s, back_node.FullLength());
          back_node.SetEndS(adjusted_end_s);
          ADEBUG << "ExtendForward: orig_end_s[" << back_node.EndS()
                 << "] adjusted_end_s[" << adjusted_end_s << "]";
        }
      }
    } else {
      return;
    }
  }

  // 2. 扩展curr_passage
  bool allowed_to_explore = true;
  while (allowed_to_explore) {
    std::vector<NodeWithRange> succ_set;
    // 访问curr_passage的末尾节点 的后继节点(往往只有一个)
    for (const auto& edge :
         curr_passage->nodes.back().GetTopoNode()->OutToSucEdge()) 
    {
      const auto& succ_node = edge->ToNode();
      // if succ node has been inserted
      // 已经被扩展的后继节点 需要被忽略
      if (ContainsKey(node_set_of_curr_passage, succ_node)) {
        continue;
      }
      // if next passage is reachable from succ node
      NodeWithRange reachable_node(succ_node, 0, 1.0);
      // 可以找到从succ_node到next_passage的edge（edge对应的next_passage上的节点、即reachable_node）
      if (IsReachableFromWithChangeLane(succ_node, next_passage,
                                        &reachable_node)) {
        const auto* succ_range = range_manager.Find(succ_node);
        if (succ_range != nullptr && !succ_range->empty()) {
          double black_s_start = succ_range->front().StartS();
          if (!IsCloseEnough(black_s_start, 0.0)) {
            succ_set.emplace_back(succ_node, 0.0, black_s_start);
          }
        } else {
          if (IsCloseEnough(reachable_node.EndS(),
                            reachable_node.FullLength())) {
            succ_set.emplace_back(succ_node, 0.0, succ_node->Length());
          } else {
            double push_end_s = reachable_node.EndS() /
                                reachable_node.FullLength() *
                                succ_node->Length();
            succ_set.emplace_back(succ_node, 0.0, push_end_s);
          }
        }
      }
    } // end for loop for to_node
    if (succ_set.empty()) {
      allowed_to_explore = false;
    } else {
      allowed_to_explore = true;
      const auto& node_to_insert = GetLargestRange(succ_set);
      // 在cur_passage末尾加入新的节点，以继续扩展
      curr_passage->nodes.push_back(node_to_insert);
      node_set_of_curr_passage.emplace(node_to_insert.GetTopoNode());
    }
  }
}

void ResultGenerator::ExtendPassages(const TopoRangeManager& range_manager,
                                     std::vector<PassageInfo>* const passages) {
  int passage_num = static_cast<int>(passages->size());
  for (int i = 0; i < passage_num; ++i) {
    // passage is not last, update(extend) it with next_passage
    if (i < passage_num - 1) {
      ExtendForward(range_manager, passages->at(i + 1), &(passages->at(i)));
    }
    // passage is not first, update(extend) it with prev_passage
    if (i > 0) {
      ExtendBackward(range_manager, passages->at(i - 1), &(passages->at(i)));
    }
  }
  // 很多多余操作，但又是必要的，比如后一条passage扩展后，前一个passage也会被扩展（前一次forloop覆盖不了这种情况）
  // TODO: 但是有更好的办法解决这种问题 => 直接把“平行lane segment”都放入routing结果里
  for (int i = passage_num - 1; i >= 0; --i) {
    if (i < passage_num - 1) {
      ExtendForward(range_manager, passages->at(i + 1), &(passages->at(i)));
    }
    if (i > 0) {
      ExtendBackward(range_manager, passages->at(i - 1), &(passages->at(i)));
    }
  }
}

void LaneNodesToPassageRegion(
    const std::vector<NodeWithRange>::const_iterator begin,
    const std::vector<NodeWithRange>::const_iterator end,
    Passage* const passage) {
  for (auto it = begin; it != end; ++it) {
    LaneSegment* seg = passage->add_segment(); // LaneSegment定义在proto中
    seg->set_id(it->LaneId());
    seg->set_start_s(it->StartS());
    seg->set_end_s(it->EndS());
  }
}

void LaneNodesToPassageRegion(const std::vector<NodeWithRange>& nodes,
                              Passage* const passage) {
  return LaneNodesToPassageRegion(nodes.begin(), nodes.end(), passage);
}

double CalculateDistance(const std::vector<NodeWithRange>& nodes) {
  double distance = nodes.at(0).EndS() - nodes.at(0).StartS();
  for (size_t i = 1; i < nodes.size(); ++i) {
    auto edge =
        nodes.at(i - 1).GetTopoNode()->GetOutEdgeTo(nodes.at(i).GetTopoNode());
    if (edge == nullptr || edge->Type() != TET_FORWARD) {
      continue;
    }
    distance += nodes.at(i).EndS() - nodes.at(i).StartS();
  }
  return distance;
}

void PrintDebugInfo(const std::string& road_id,
                    const std::vector<std::vector<NodeWithRange>>& nodes) {
  AINFO << "road id: " << road_id;
  for (size_t j = 0; j < nodes.size(); ++j) {
    AINFO << "\tPassage " << j;
    for (const auto& node : nodes[j]) {
      AINFO << "\t\t" << node.LaneId() << "   (" << node.StartS() << ", "
            << node.EndS() << ")";
    }
  }
}

// generate final routing response
bool ResultGenerator::GeneratePassageRegion(
    const std::string& map_version, 
    const RoutingRequest& request,
    const std::vector<NodeWithRange>& nodes,
    const TopoRangeManager& range_manager, 
    RoutingResponse* const result) {
  if (!GeneratePassageRegion(nodes, range_manager, result)) {
    return false;
  }

  result->set_map_version(map_version);
  result->mutable_measurement()->set_distance(CalculateDistance(nodes));
  result->mutable_routing_request()->CopyFrom(request);
  return true;
}

bool ResultGenerator::GeneratePassageRegion(
    const std::vector<NodeWithRange>& nodes, // result node from routing
    const TopoRangeManager& range_manager,   // 维护了node到range的映射
    RoutingResponse* const result) {
  std::vector<PassageInfo> passages;
  // 用LC_edge给节点分组，分其成为不同的passage
  if (!ExtractBasicPassages(nodes, &passages)) {
    return false;
  }
  // 目前，ExtractBasicPassages只在两个node是变道关系时，才生成新的passage，
  // 根据后一个passage扩展前一个passage（其实就是增加平行节点）
  /* 三车道case：
  * | | |o|    |x|x|o|
  * | | |o| => |x|x|o|
  * |o|o|o|    |o|o|o|, where 'x' is extended node
  */
  ExtendPassages(range_manager, &passages);

  CreateRoadSegments(passages, result);

  return true;
}

void ResultGenerator::AddRoadSegment(
    const std::vector<PassageInfo>& passages,
    const std::pair<std::size_t, std::size_t>& start_index, // passage id 和 node id?
    const std::pair<std::size_t, std::size_t>& end_index,
    RoutingResponse* result) {
  auto* road = result->add_road();

  // 创建新road（实际上是RoadSegment），赋予当前节点的road_id
  road->set_id(passages[start_index.first].nodes[start_index.second].RoadId());

  for (std::size_t i = start_index.first;
      i <= end_index.first && i < passages.size(); ++i)
  {
    auto* passage = road->add_passage();

    // 1. 得到当前passage-i的首节点，
    // start_index的开始节点是其记录的节点，end_index、和之间的passage的开始节点是第一个
    const size_t node_start_index =
        (i == start_index.first ?
          std::max((std::size_t)0, start_index.second) 
          : 0);
    const auto node_begin_iter = passages[i].nodes.cbegin() + node_start_index;
    ADEBUG<< "start node: " << node_begin_iter->LaneId() << ": "
           << node_begin_iter->StartS() << "; " << node_begin_iter->EndS();
    
    // 2. 得到当前passage-i的末尾节点，
    // start_index、和之间的passage的结尾节点是最后一个，end_index的结尾结点是其记录的
    const size_t node_end_index =
         (i == end_index.first ?
         std::min(end_index.second, passages[i].nodes.size() - 1) :
         passages[i].nodes.size() - 1);
    const auto node_last_iter = passages[i].nodes.cbegin() + node_end_index;
    ADEBUG << "last node: " << node_last_iter->LaneId() << ": "
           << node_last_iter->StartS() << "; " << node_last_iter->EndS();
    
    // 把passage-i上的start到end的所有节点放入passage-i中，一个节点作为一个LaneSegment
    auto node_end_iter = node_last_iter + 1;
    LaneNodesToPassageRegion(node_begin_iter, node_end_iter, passage);

    // passage(start_index)和passage(end_index)的id如果相等，则他们是forward关系；
    // 不相等则是变道关系
    if (start_index.first == end_index.first) {
      // 只有一个passage时
      passage->set_change_lane_type(FORWARD);
      passage->set_can_exit(true);
    } else {
      // type：除了最后一个passage是forward，其他passage是变道类型
      passage->set_change_lane_type(passages[i].change_lane_type);
      // 最后一个passage的can_exit=true
      passage->set_can_exit(i == end_index.first);
    }
  }
}

void ResultGenerator::CreateRoadSegments(
    const std::vector<PassageInfo>& passages, RoutingResponse* result) 
{
  ACHECK(!passages.empty()) << "passages empty";
  NodeWithRange fake_node_range(passages.front().nodes.front());
  bool in_change_lane = false;
  std::pair<std::size_t, std::size_t> start_index(0, 0);
  for (std::size_t i = 0; i < passages.size(); ++i) 
  {
    const auto& curr_nodes = passages[i].nodes;
    for (std::size_t j = 0; j < curr_nodes.size(); ++j) 
    {
      // 如果 能在passage[i+1]上找到node 满足此node有“变道edge”至cur_node[j]
      // 或者 能在passage[i-1]上找到node 满足cur_node[j] -LC-> node
      if ((i + 1 < passages.size() &&
           IsReachableToWithChangeLane(curr_nodes[j].GetTopoNode(),
                                       passages[i + 1], &fake_node_range)) ||
          (i > 0 &&
           IsReachableFromWithChangeLane(curr_nodes[j].GetTopoNode(),
                                         passages[i - 1], &fake_node_range))) {
        if (!in_change_lane) {
          start_index = {i, j};
          in_change_lane = true;
        }
      } else { // 在前一个、后一个passage中找不到变道关系
        if (in_change_lane) {
          ADEBUG << "start_index(" << start_index.first << ", "
                 << start_index.second
                 << ") end_index(" << i << ", " << j - 1 << ")";
          AddRoadSegment(passages, start_index, {i, j - 1}, result);
        }
        // 为 passage[i].nodes[j]创建road_segment
        ADEBUG << "start_index(" << i << ", " << j
               << ") end_index(" << i << ", " << j << ")";
        AddRoadSegment(passages, {i, j}, {i, j}, result);
        in_change_lane = false;
      }
    }
  }
  if (in_change_lane) {
    ADEBUG << "start_index(" << start_index.first << ", " << start_index.second
           << ") end_index(" << passages.size() - 1 << ", "
           << passages.back().nodes.size() - 1 << ")";
    // 当上面除了首节点外，其余节点都能找到变道edge，则需在最后节点处新建RoadSegment
    AddRoadSegment(passages, start_index,
                   {passages.size() - 1, passages.back().nodes.size() - 1},
                   result);
  }
}

}  // namespace routing
}  // namespace apollo
