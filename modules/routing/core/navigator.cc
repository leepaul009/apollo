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

#include "modules/routing/core/navigator.h"

#include "cyber/common/file.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/sub_topo_graph.h"
#include "modules/routing/strategy/a_star_strategy.h"

namespace apollo {
namespace routing {

namespace {

using apollo::common::ErrorCode;

bool ShowRequestInfo(const RoutingRequest& request, const TopoGraph* graph) {
  for (const auto& wp : request.waypoint()) {
    const auto* node = graph->GetNode(wp.id());
    if (node == nullptr) {
      AERROR << "Way node is not found in topo graph! ID: " << wp.id();
      return false;
    }
    AINFO << "Way point:\tlane id: " << wp.id() << " s: " << wp.s()
          << " x: " << wp.pose().x() << " y: " << wp.pose().y()
          << " length: " << node->Length();
  }

  for (const auto& bl : request.blacklisted_lane()) {
    const auto* node = graph->GetNode(bl.id());
    if (node == nullptr) {
      AERROR << "Black list node is not found in topo graph! ID: " << bl.id();
      return false;
    }
    AINFO << "Black point:\tlane id: " << bl.id()
          << " start_s: " << bl.start_s() << " end_s: " << bl.end_s()
          << " length: " << node->Length();
  }

  return true;
}

// 用request的waypoint点的id（lane_id）在graph中找到对应的node（唯一的）加入way_nodes
// 将waypoint点的s值加入way_s
bool GetWayNodes(const RoutingRequest& request, const TopoGraph* graph,
                 std::vector<const TopoNode*>* const way_nodes,
                 std::vector<double>* const way_s) {
  for (const auto& point : request.waypoint()) {
    // 用lane_id(point.id)查找node
    const auto* cur_node = graph->GetNode(point.id());
    if (cur_node == nullptr) {
      AERROR << "Cannot find way point in graph! Id: " << point.id();
      return false;
    }
    way_nodes->push_back(cur_node);
    way_s->push_back(point.s());
  }
  return true;
}

void SetErrorCode(const common::ErrorCode& error_code_id,
                  const std::string& error_string,
                  common::StatusPb* const error_code) {
  error_code->set_error_code(error_code_id);
  error_code->set_msg(error_string);
  if (error_code_id == common::ErrorCode::OK) {
    ADEBUG << error_string.c_str();
  } else {
    AERROR << error_string.c_str();
  }
}

void PrintDebugData(const std::vector<NodeWithRange>& nodes) {
  AINFO << "Route lane id\tis virtual\tstart s\tend s";
  for (const auto& node : nodes) {
    AINFO << node.GetTopoNode()->LaneId() << "\t"
          << node.GetTopoNode()->IsVirtual() << "\t" << node.StartS() << "\t"
          << node.EndS();
  }
}

}  // namespace

Navigator::Navigator(const std::string& topo_file_path) {
  Graph graph;
  if (!cyber::common::GetProtoFromFile(topo_file_path, &graph)) {
    AERROR << "Failed to read topology graph from " << topo_file_path;
    return;
  }

  graph_.reset(new TopoGraph());
  // load输入graph的node和edge(edge可为空)
  if (!graph_->LoadGraph(graph)) {
    AINFO << "Failed to init navigator graph failed! File path: "
          << topo_file_path;
    return;
  }
  black_list_generator_.reset(new BlackListRangeGenerator);
  result_generator_.reset(new ResultGenerator);
  is_ready_ = true;
  AINFO << "The navigator is ready.";
}

Navigator::~Navigator() {}

bool Navigator::IsReady() const { return is_ready_; }

void Navigator::Clear() { topo_range_manager_.Clear(); }

bool Navigator::Init(const RoutingRequest& request, 
                     const TopoGraph* graph,
                     std::vector<const TopoNode*>* const way_nodes,
                     std::vector<double>* const way_s) {
  Clear();
  // 找到request的waypoint点，在graph中对应的节点way_nodes和位置way_s
  if (!GetWayNodes(request, graph_.get(), way_nodes, way_s)) {
    AERROR << "Failed to find search terminal point in graph!";
    return false;
  }
  // request记录这black_listed_road、lane，在graph中找到对应的node，
  // 然后，在topo_range_manager_中，记录这些node和对应的range，
  // range的设置：road的节点取（0，node.length），lane的节点取（node.start, node.end）
  // TODO: range的选择，有何差别??
  black_list_generator_->GenerateBlackMapFromRequest(request, graph_.get(),
                                                     &topo_range_manager_);
  return true;
}

bool Navigator::MergeRoute(
    const std::vector<NodeWithRange>& node_vec,
    std::vector<NodeWithRange>* const result_node_vec) const {
  for (const auto& node : node_vec) {
    if (result_node_vec->empty() ||
        result_node_vec->back().GetTopoNode() != node.GetTopoNode()) {
      result_node_vec->push_back(node);
    } else {
      if (result_node_vec->back().EndS() < node.StartS()) {
        AERROR << "Result route is not continuous.";
        return false;
      }
      result_node_vec->back().SetEndS(node.EndS());
    }
  }
  return true;
}

/***************** search space for behavior planner : beg *****************/
// [New] search lane level routing result and search space
bool Navigator::SearchRouteAndSearchSpaceByStrategy(
    const TopoGraph* graph, 
    const std::vector<const TopoNode*>& way_nodes,
    const std::vector<double>& way_s,
    std::vector<NodeWithRange>* const result_nodes,
    std::vector<std::vector<NodeWithRange>>* const search_space) const {
  std::unique_ptr<Strategy> strategy_ptr;
  strategy_ptr.reset(new AStarStrategy(FLAGS_enable_change_lane_in_result));

  result_nodes->clear();
  std::vector<NodeWithRange> node_vec;
  for (size_t i = 1; i < way_nodes.size(); ++i) {
    const auto* way_start = way_nodes[i - 1];
    const auto* way_end = way_nodes[i];
    double way_start_s = way_s[i - 1];
    double way_end_s = way_s[i];

    TopoRangeManager full_range_manager = topo_range_manager_;
    black_list_generator_->AddBlackMapFromTerminal(
        way_start, way_end, way_start_s, way_end_s, &full_range_manager);

    SubTopoGraph sub_graph(full_range_manager.RangeMap());
    const auto* start = sub_graph.GetSubNodeWithS(way_start, way_start_s);
    if (start == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_start->LaneId() << ", s:" << way_start_s;
      return false;
    }
    const auto* end = sub_graph.GetSubNodeWithS(way_end, way_end_s);
    if (end == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_end->LaneId() << ", s:" << way_end_s;
      return false;
    }

    std::vector<NodeWithRange> cur_result_nodes;
    if (!strategy_ptr->Search(graph, &sub_graph, start, end,
                              &cur_result_nodes)) {
      AERROR << "Failed to search route with waypoint from " << start->LaneId()
             << " to " << end->LaneId();
      return false;
    }
    node_vec.insert(node_vec.end(), cur_result_nodes.begin(),
                    cur_result_nodes.end());
    
    // compute search space based on road level routing result
    std::vector<std::vector<NodeWithRange>> cur_search_space;
    if (!strategy_ptr->GetParallelSearchSpace(start, end, &sub_graph, 
                                              cur_result_nodes, 
                                              cur_search_space)) {
      AERROR << "Failed to get search space with waypoint from " << start->LaneId()
             << " to " << end->LaneId();
      return false;
    }
    // TODO: find corner case for merge
    search_space->insert(search_space->end(), 
                         cur_search_space.begin(),
                         cur_search_space.end());
  }
  
  if (!MergeRoute(node_vec, result_nodes)) {
    AERROR << "Failed to merge route.";
    return false;
  }
  return true;
}

// [New]
bool Navigator::SearchRouteAndSearchSpace(const RoutingRequest& request,
                            RoutingResponse* const response) {
  // 1. preparation and validance checking
  if (!ShowRequestInfo(request, graph_.get())) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_REQUEST,
                 "Error encountered when reading request point!",
                 response->mutable_status());
    return false;
  }

  if (!IsReady()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY, "Navigator is not ready!",
                 response->mutable_status());
    return false;
  }
  std::vector<const TopoNode*> way_nodes;
  std::vector<double> way_s;
  // init way_nodes and way_s based on request's waypoint
  if (!Init(request, graph_.get(), &way_nodes, &way_s)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY,
                 "Failed to initialize navigator!", response->mutable_status());
    return false;
  }

  // 2. search routing proposal and search space
  std::vector<NodeWithRange> result_nodes;
  std::vector<std::vector<NodeWithRange>> search_space;
  if (!SearchRouteAndSearchSpaceByStrategy(
      graph_.get(), way_nodes, way_s, &result_nodes, &search_space)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to find route with request!",
                 response->mutable_status());
    return false;
  }

  if (result_nodes.empty()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE, "Failed to result nodes!",
                 response->mutable_status());
    return false;
  }
  result_nodes.front().SetStartS(request.waypoint().begin()->s());
  result_nodes.back().SetEndS(request.waypoint().rbegin()->s());

  // 3. fill routing response
  if (!result_generator_->GenerateRoutingResponse(
          graph_->MapVersion(), request, result_nodes, 
          search_space, topo_range_manager_, response)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to generate passage regions based on result lanes",
                 response->mutable_status());
    return false;
  }

  SetErrorCode(ErrorCode::OK, "Success!", response->mutable_status());

  PrintDebugData(result_nodes);
  return true;
}

/***************** search space for behavior planner : end *****************/

// input:
//    way_nodes, way_s: 通过request在graph找到的
bool Navigator::SearchRouteByStrategy(
    const TopoGraph* graph, const std::vector<const TopoNode*>& way_nodes,
    const std::vector<double>& way_s,
    std::vector<NodeWithRange>* const result_nodes) const {
  std::unique_ptr<Strategy> strategy_ptr;
  strategy_ptr.reset(new AStarStrategy(FLAGS_enable_change_lane_in_result));

  result_nodes->clear();
  std::vector<NodeWithRange> node_vec;
  for (size_t i = 1; i < way_nodes.size(); ++i) {
    const auto* way_start = way_nodes[i - 1];
    const auto* way_end = way_nodes[i];
    double way_start_s = way_s[i - 1];
    double way_end_s = way_s[i];

    TopoRangeManager full_range_manager = topo_range_manager_;
    // Navigator初始化时，black_list_generator_已经在topo_range_manager_中
    // 记录了request的black_listed_road/lane
    // 找到开始节点的直接、间接left、right关系的后继节点，
    // 找到结尾节点的直接、间接left、right关系的前驱节点，插入临时的full_range_manager
    black_list_generator_->AddBlackMapFromTerminal(
        way_start, way_end, way_start_s, way_end_s, &full_range_manager);

    SubTopoGraph sub_graph(full_range_manager.RangeMap());
    // 从sub_node中选择一个合适的作为start
    const auto* start = sub_graph.GetSubNodeWithS(way_start, way_start_s);
    if (start == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_start->LaneId() << ", s:" << way_start_s;
      return false;
    }
    const auto* end = sub_graph.GetSubNodeWithS(way_end, way_end_s);
    if (end == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_end->LaneId() << ", s:" << way_end_s;
      return false;
    }

    std::vector<NodeWithRange> cur_result_nodes;
    if (!strategy_ptr->Search(graph, &sub_graph, start, end,
                              &cur_result_nodes)) {
      AERROR << "Failed to search route with waypoint from " << start->LaneId()
             << " to " << end->LaneId();
      return false;
    }

    node_vec.insert(node_vec.end(), cur_result_nodes.begin(),
                    cur_result_nodes.end());
  }

  if (!MergeRoute(node_vec, result_nodes)) {
    AERROR << "Failed to merge route.";
    return false;
  }
  return true;
}

bool Navigator::SearchRoute(const RoutingRequest& request,
                            RoutingResponse* const response) {
  // 1. preparation
  // 确认request.waypoint.id(lane)存在于graph_
  if (!ShowRequestInfo(request, graph_.get())) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_REQUEST,
                 "Error encountered when reading request point!",
                 response->mutable_status());
    return false;
  }

  // 确认已经加载graph
  if (!IsReady()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY, "Navigator is not ready!",
                 response->mutable_status());
    return false;
  }
  std::vector<const TopoNode*> way_nodes;
  std::vector<double> way_s;
  // 通过request的waypoint点，在graph中找到对应的way_nodes和way_s
  if (!Init(request, graph_.get(), &way_nodes, &way_s)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY,
                 "Failed to initialize navigator!", response->mutable_status());
    return false;
  }

  // 2. search
  std::vector<NodeWithRange> result_nodes;
  if (!SearchRouteByStrategy(graph_.get(), way_nodes, way_s, &result_nodes)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to find route with request!",
                 response->mutable_status());
    return false;
  }
  if (result_nodes.empty()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE, "Failed to result nodes!",
                 response->mutable_status());
    return false;
  }
  result_nodes.front().SetStartS(request.waypoint().begin()->s());
  result_nodes.back().SetEndS(request.waypoint().rbegin()->s());

  // 3. create response
  // 扩展result_nodes（添加一些并行node），
  // 以RoadSegment/Passage/LaneSegment的结构重构routing结果，存储在response中
  if (!result_generator_->GeneratePassageRegion(
          graph_->MapVersion(), request, result_nodes, topo_range_manager_,
          response)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to generate passage regions based on result lanes",
                 response->mutable_status());
    return false;
  }
  SetErrorCode(ErrorCode::OK, "Success!", response->mutable_status());

  PrintDebugData(result_nodes);
  return true;
}

}  // namespace routing
}  // namespace apollo
