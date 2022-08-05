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

#include "modules/map/pnc_map/pnc_map.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_cat.h"
#include "google/protobuf/text_format.h"

#include "modules/map/proto/map_id.pb.h"

#include "cyber/common/log.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/routing/common/routing_gflags.h"

DEFINE_double(
    look_backward_distance, 50,
    "look backward this distance when creating reference line from routing");

DEFINE_double(look_forward_short_distance, 180,
              "short look forward this distance when creating reference line "
              "from routing when ADC is slow");
DEFINE_double(
    look_forward_long_distance, 250,
    "look forward this distance when creating reference line from routing");

namespace apollo {
namespace hdmap {

using apollo::common::PointENU;
using apollo::common::VehicleState;
using apollo::common::util::PointFactory;
using apollo::routing::RoutingResponse;

namespace {

// Maximum lateral error used in trajectory approximation.
const double kTrajectoryApproximationMaxError = 2.0;

}  // namespace

PncMap::PncMap(const HDMap *hdmap) : hdmap_(hdmap) {}

const hdmap::HDMap *PncMap::hdmap() const { return hdmap_; }

LaneWaypoint PncMap::ToLaneWaypoint(
    const routing::LaneWaypoint &waypoint) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(waypoint.id()));
  ACHECK(lane) << "Invalid lane id: " << waypoint.id();
  return LaneWaypoint(lane, waypoint.s());
}

double PncMap::LookForwardDistance(double velocity) {
  auto forward_distance = velocity * FLAGS_look_forward_time_sec;

  return forward_distance > FLAGS_look_forward_short_distance
             ? FLAGS_look_forward_long_distance
             : FLAGS_look_forward_short_distance;
}

LaneSegment PncMap::ToLaneSegment(const routing::LaneSegment &segment) const {
  // get LaneInf
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
  ACHECK(lane) << "Invalid lane id: " << segment.id();
  return LaneSegment(lane, segment.start_s(), segment.end_s());
}

void PncMap::UpdateNextRoutingWaypointIndex(int cur_index) {
  // 1. cur_index非法时的处理
  if (cur_index < 0) {
    next_routing_waypoint_index_ = 0;
    return;
  }
  if (cur_index >= static_cast<int>(route_indices_.size())) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
    return;
  }
  // 因为新的routing response传来时，next_routing_waypoint_index_被设置为0，
  // Search backwards when the car is driven backward on the route.
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index >
             cur_index) {
    --next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index ==
             cur_index &&
         adc_waypoint_.s <
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    --next_routing_waypoint_index_;
  }
  // Search forwards
  // 
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index <
             cur_index) {
    ++next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         cur_index ==
             routing_waypoint_index_[next_routing_waypoint_index_].index &&
         adc_waypoint_.s >=
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    ++next_routing_waypoint_index_;
  }
  if (next_routing_waypoint_index_ >= routing_waypoint_index_.size()) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
  }
}

std::vector<routing::LaneWaypoint> PncMap::FutureRouteWaypoints() const {
  const auto &waypoints = routing_.routing_request().waypoint();
  return std::vector<routing::LaneWaypoint>(
      waypoints.begin() + next_routing_waypoint_index_, waypoints.end());
}

// 更新routing的范围，即range_start_和range_end_，记录第一个node(最左)到最后一个节点（最右）
// TODO: range_start_和range_end_的范围是否可能超界？
void PncMap::UpdateRoutingRange(int adc_index) {
  // Track routing range.
  range_lane_ids_.clear();
  range_start_ = std::max(0, adc_index - 1);
  range_end_ = range_start_;
  while (range_end_ < static_cast<int>(route_indices_.size())) {
    const auto &lane_id = route_indices_[range_end_].segment.lane->id().id();
    if (range_lane_ids_.count(lane_id) != 0) {
      break;
    }
    range_lane_ids_.insert(lane_id);
    ++range_end_;
  }
}

// 从routing中查找到（离ego状态）最近的道路点adc_waypoint，
// 根据adc_waypoint来在route_indices_中查找到当前行驶至了routing的哪条LaneSegment上，
// 并将“LaneSegment的index”赋给adc_route_index_
bool PncMap::UpdateVehicleState(const VehicleState &vehicle_state) {
  if (!ValidateRouting(routing_)) {
    AERROR << "The routing is invalid when updating vehicle state.";
    return false;
  }
  if (!adc_state_.has_x() ||
      (common::util::DistanceXY(adc_state_, vehicle_state) >
       FLAGS_replan_lateral_distance_threshold +
           FLAGS_replan_longitudinal_distance_threshold)) {
    // Position is reset, but not replan.
    next_routing_waypoint_index_ = 0;
    adc_route_index_ = -1;
    stop_for_destination_ = false;
  }

  adc_state_ = vehicle_state;
  if (!GetNearestPointFromRouting(vehicle_state, &adc_waypoint_)) {
    AERROR << "Failed to get waypoint from routing with point: "
           << "(" << vehicle_state.x() << ", " << vehicle_state.y() << ", "
           << vehicle_state.z() << ").";
    return false;
  }
  // 根据adc_waypoint来在route_indices_中查找到当前行驶至了routing的哪条LaneSegment上
  int route_index = GetWaypointIndex(adc_waypoint_);
  if (route_index < 0 ||
      route_index >= static_cast<int>(route_indices_.size())) {
    AERROR << "Cannot find waypoint: " << adc_waypoint_.DebugString();
    return false;
  }

  // Track how many routing request waypoints the adc have passed.
  // 计算下一个routing waypoint点（导航必经点？），并赋给next_routing_waypoint_index_
  UpdateNextRoutingWaypointIndex(route_index);
  adc_route_index_ = route_index;
  UpdateRoutingRange(adc_route_index_);

  if (routing_waypoint_index_.empty()) {
    AERROR << "No routing waypoint index.";
    return false;
  }

  // next_routing_waypoint_index_是终点所在LaneSegment（topoNode）
  if (next_routing_waypoint_index_ == routing_waypoint_index_.size() - 1) {
    stop_for_destination_ = true;
  }
  return true;
}

bool PncMap::IsNewRouting(const routing::RoutingResponse &routing) const {
  return IsNewRouting(routing_, routing);
}

bool PncMap::IsNewRouting(const routing::RoutingResponse &prev,
                          const routing::RoutingResponse &routing) {
  if (!ValidateRouting(routing)) {
    ADEBUG << "The provided routing is invalid.";
    return false;
  }
  return !common::util::IsProtoEqual(prev, routing);
}

/************* Step 1 *************/
// 每当有新的routing结果时，该方法会用最新的RoutingResponse对PncMap里的数据进行更新
bool PncMap::UpdateRoutingResponse(const routing::RoutingResponse &routing) {
  range_lane_ids_.clear();
  route_indices_.clear();
  all_lane_ids_.clear();

  for (int road_index = 0; road_index < routing.road_size(); ++road_index) {
    const auto &road_segment = routing.road(road_index);
    for (int passage_index = 0; passage_index < road_segment.passage_size();
         ++passage_index) {
      const auto &passage = road_segment.passage(passage_index);
      for (int lane_index = 0; lane_index < passage.segment_size();
           ++lane_index) {
        // 这里的id是routing模块里node的id，也是hdmap的lane的id？
        all_lane_ids_.insert(passage.segment(lane_index).id());
        route_indices_.emplace_back();
        // convert routing's LaneSegment to path's LaneSegment
        route_indices_.back().segment =
            ToLaneSegment(passage.segment(lane_index));
        if (route_indices_.back().segment.lane == nullptr) {
          AERROR << "Failed to get lane segment from passage.";
          return false;
        }
        route_indices_.back().index = {road_index, passage_index, lane_index};
      }
    }
  }

  range_start_ = 0;
  range_end_ = 0;
  adc_route_index_ = -1;
  next_routing_waypoint_index_ = 0;
  // 将routing结果,即route_indices_中的lane_id存放进range_lane_ids_中
  // route_indices_中的lane_id应该是唯一的
  UpdateRoutingRange(adc_route_index_);

  // routing_request的waypoint是导航起始、中间、终止点（至少有2个）。
  // 将waypoint的信息存储在routing_waypoint_index_里。
  routing_waypoint_index_.clear();
  const auto &request_waypoints = routing.routing_request().waypoint(); // repeated LaneWaypoint
  if (request_waypoints.empty()) {
    AERROR << "Invalid routing: no request waypoints.";
    return false;
  }
  int i = 0;
  for (size_t j = 0; j < route_indices_.size(); ++j) {
    while (i < request_waypoints.size() &&
           RouteSegments::WithinLaneSegment(route_indices_[j].segment,
                                            request_waypoints.Get(i))) {
      routing_waypoint_index_.emplace_back(
          LaneWaypoint(route_indices_[j].segment.lane, // hdmap的LaneInfo
                       request_waypoints.Get(i).s()),
          j); // 将waypoint对应的lane信息、s值和laneSegment序号存储在routing_waypoint_index_中。
      ++i;
    }
  }
  routing_ = routing;
  adc_waypoint_ = LaneWaypoint();
  stop_for_destination_ = false;
  return true;
}

const routing::RoutingResponse &PncMap::routing_response() const {
  return routing_;
}

bool PncMap::ValidateRouting(const RoutingResponse &routing) {
  const int num_road = routing.road_size();
  if (num_road == 0) {
    AERROR << "Route is empty.";
    return false;
  }
  if (!routing.has_routing_request() ||
      routing.routing_request().waypoint_size() < 2) {
    AERROR << "Routing does not have request.";
    return false;
  }
  for (const auto &waypoint : routing.routing_request().waypoint()) {
    if (!waypoint.has_id() || !waypoint.has_s()) {
      AERROR << "Routing waypoint has no lane_id or s.";
      return false;
    }
  }
  return true;
}

int PncMap::SearchForwardWaypointIndex(int start,
                                       const LaneWaypoint &waypoint) const {
  int i = std::max(start, 0);
  while (
      i < static_cast<int>(route_indices_.size()) &&
      !RouteSegments::WithinLaneSegment(route_indices_[i].segment, waypoint)) {
    ++i;
  }
  return i;
}

int PncMap::SearchBackwardWaypointIndex(int start,
                                        const LaneWaypoint &waypoint) const {
  int i = std::min(static_cast<int>(route_indices_.size() - 1), start);
  while (i >= 0 && !RouteSegments::WithinLaneSegment(route_indices_[i].segment,
                                                     waypoint)) {
    --i;
  }
  return i;
}

int PncMap::NextWaypointIndex(int index) const {
  if (index >= static_cast<int>(route_indices_.size() - 1)) {
    return static_cast<int>(route_indices_.size()) - 1;
  } else if (index < 0) {
    return 0;
  } else {
    return index + 1;
  }
}

int PncMap::GetWaypointIndex(const LaneWaypoint &waypoint) const {
  int forward_index = SearchForwardWaypointIndex(adc_route_index_, waypoint);
  if (forward_index >= static_cast<int>(route_indices_.size())) {
    return SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  }
  if (forward_index == adc_route_index_ ||
      forward_index == adc_route_index_ + 1) {
    return forward_index;
  }
  auto backward_index = SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  if (backward_index < 0) {
    return forward_index;
  }

  return (backward_index + 1 == adc_route_index_) ? backward_index
                                                  : forward_index;
}

bool PncMap::PassageToSegments(routing::Passage passage,
                               RouteSegments *segments) const {
  CHECK_NOTNULL(segments);
  segments->clear();
  for (const auto &lane : passage.segment()) { // segment() return lane segment
    auto lane_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << lane.id();
      return false;
    }
    // segment的endS有可能比“所在lane的总长”要短，这种情况，segment对应着topoGraph的次节点
    segments->emplace_back(lane_ptr, std::max(0.0, lane.start_s()),
                           std::min(lane_ptr->total_length(), lane.end_s()));
  }
  return !segments->empty();
}

// routing::RoadSegment defined in routing/proto/routing.pb.h
std::vector<int> PncMap::GetNeighborPassages(const routing::RoadSegment &road,
                                             int start_passage) const {
  CHECK_GE(start_passage, 0);
  CHECK_LE(start_passage, road.passage_size());
  std::vector<int> result;
  const auto &source_passage = road.passage(start_passage);
  result.emplace_back(start_passage);
  if (source_passage.change_lane_type() == routing::FORWARD) {
    return result;
  }
  if (source_passage.can_exit()) {  // No need to change lane
    return result;
  }

  // get source_segments(LaneSegments) from input passage  
  RouteSegments source_segments;
  if (!PassageToSegments(source_passage, &source_segments)) {
    AERROR << "Failed to convert passage to segments";
    return result;
  }
  // 如果“下一个route waypoint”在当前passage上，则不必再找“邻居”了
  if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
      source_segments.IsWaypointOnSegment(
          routing_waypoint_index_[next_routing_waypoint_index_].waypoint)) {
    ADEBUG << "Need to pass next waypoint[" << next_routing_waypoint_index_
           << "] before change lane";
    return result;
  }

  // 只考虑input passage的“直接邻居”
  std::unordered_set<std::string> neighbor_lanes;
  if (source_passage.change_lane_type() == routing::LEFT) {
    for (const auto &segment : source_segments) {
      for (const auto &left_id :
           segment.lane->lane().left_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(left_id.id());
      }
    }
  } else if (source_passage.change_lane_type() == routing::RIGHT) {
    for (const auto &segment : source_segments) {
      for (const auto &right_id :
           segment.lane->lane().right_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(right_id.id());
      }
    }
  }

  // we only consider neighbor lanes that are inside road(RoadSegment)
  for (int i = 0; i < road.passage_size(); ++i) {
    if (i == start_passage) {
      continue;
    }
    const auto &target_passage = road.passage(i);
    for (const auto &segment : target_passage.segment()) {
      if (neighbor_lanes.count(segment.id())) {
        result.emplace_back(i);
        break;
      }
    }
  }
  return result;
}

/************* Step 2(main functionality): *************/
// 
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              std::list<RouteSegments> *const route_segments) {
  double look_forward_distance =
      LookForwardDistance(vehicle_state.linear_velocity());
  double look_backward_distance = FLAGS_look_backward_distance;
  return GetRouteSegments(vehicle_state, look_backward_distance,
                          look_forward_distance, route_segments);
}


/************************* for behavior planner:beg *************************/
// [New]
// TODO: consider all rather than search space
std::vector<LaneInfoConstPtr> PncMap::GetSuccessors(
  LaneInfoConstPtr lane) const {

  std::vector<LaneInfoConstPtr> result;
  if (lane->lane().successor_id().empty()) {
    return result;
  }
  // hdmap::Id preferred_id = lane->lane().successor_id(0);
  // return lane_id's type is 'Id', lane_id.id()'s type is 'string'
  for (const auto &lane_id : lane->lane().successor_id()) {
    //if (all_lane_ids_.count(lane_id.id()) != 0) { 
      result.push_back(hdmap_->GetLaneById(lane_id));      
    //}
  }
  
  return result;
}

// [New]
// TODO: consider all rather than search space
std::vector<LaneInfoConstPtr> PncMap::GetPredecessors(
  LaneInfoConstPtr lane) const {

  std::vector<LaneInfoConstPtr> result;
  if (lane->lane().predecessor_id().empty()) {
    return result;
  }
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    //if (all_lane_ids_.count(lane_id.id()) != 0) {
      result.push_back(hdmap_->GetLaneById(lane_id));    
    //}
  }
  return result;
}

// [New]
std::vector<LaneInfoConstPtr> PncMap::GetLeftAndRightNeighbors(
  LaneInfoConstPtr lane) const {
  
  std::vector<LaneInfoConstPtr> result;
  if (lane->lane().left_neighbor_forward_lane_id().empty() &&
      lane->lane().right_neighbor_forward_lane_id().empty()) {
    return result;
  }
  for (const auto &lane_id : lane->lane().left_neighbor_forward_lane_id()) { // Id
    // if (all_lane_ids_.count(lane_id.id()) != 0) {
      result.push_back(hdmap_->GetLaneById(lane_id)); 
    // }
  }
  for (const auto &lane_id : lane->lane().right_neighbor_forward_lane_id()) { // Id
    // if (all_lane_ids_.count(lane_id.id()) != 0) {
      result.push_back(hdmap_->GetLaneById(lane_id)); 
    // }
  }
  return result;
}

// [New]
bool PncMap::GetPossibleSequencesFromLaneSegment(
  const LaneSegment& lane_segment,
  const double horizon,
  const bool is_forward,
  std::list<RouteSegments>& sequences) {
  
  sequences.clear();
  double epsilon = 1e-3;
  std::vector<std::pair<RouteSegments, double>> open_stack;

  // TODO: check input lane_seg is valid
  // LaneSegment length might be partial of map_lane_segment
  double remaining_h = horizon - lane_segment.Length();
  if (remaining_h - epsilon <= 0.0) {
    sequences.emplace_back();
    sequences.back().push_back(lane_segment);
    return true;
  }
  RouteSegments r_seg;
  r_seg.push_back(lane_segment);
  open_stack.emplace_back(r_seg, remaining_h);
  
  while(!open_stack.empty()) {
    auto curr_sequence = std::move(open_stack.back().first);
    remaining_h = open_stack.back().second;
    open_stack.pop_back();
    // find continous lane segments
    auto continous_lane_segments = is_forward ? 
      GetSuccessors(curr_sequence.back().lane) : GetPredecessors(curr_sequence.back().lane);

    for (auto& cont_ls : continous_lane_segments) {
      // check if continue lane segment existed in search space
      if (all_lane_ids_.count(cont_ls->id().id()) == 0) {
        continue;
      }
      // each continous laneSegment maintain its own lane sequence.
      RouteSegments next_seq(curr_sequence);
      next_seq.emplace_back(cont_ls, 0.0, cont_ls->total_length());

      double remaining_next = remaining_h - next_seq.back().Length();
      if (remaining_next - epsilon <= 0.0) {
        // curr_sequence has positive remaining horizon,
        // while next_sequence has negative. stop searching forward/backward.
        sequences.emplace_back(std::move(next_seq));
        continue;
      }
      open_stack.emplace_back(std::move(next_seq), remaining_next); // TODO: don't copy
    }

    // can not find continous lane segments, insert current sequence into output
    if (continous_lane_segments.size() == 0) {
      sequences.emplace_back(std::move(curr_sequence)); // TODO: don't copy
      continue;
    }
  } // end while

  return true;
}

// [New]
bool PncMap::ExtendLaneSequencesForLaneMerge(
  const double horizon, // TODO: current not use
  std::list<RouteSegments>& sequences) {

  if (sequences.empty()) {
    return true;
  }
  std::list<RouteSegments> additional_seqs;
  for (const auto& sequence : sequences) {
    // ignore the first lane segment
    if (sequence.size() <= 1) {
      continue;
    }
    // as we ignore first lane segment, the second and forward lane segment
    // is not partial lane segment(where start_s!=0, end_s!=lane_seg_length)
    for (auto it = sequence.begin()+1; it != sequence.end(); it++) {
      auto predecessors = GetPredecessors(it->lane);      
      // create a new sequence for right predecessor.
      for (const auto& p : predecessors) {
        // TODO: if we should check nullptr?
        if (all_lane_ids_.count(p->id().id()) > 0) {
          continue;
        }
        // find predecessor not in search space.
        additional_seqs.emplace_back();
        auto& seq = additional_seqs.back(); // RouteSegments
        seq.emplace_back(p, 0.0, p->total_length());
        seq.insert(seq.end(), it, sequence.end());
      }
      // TODO: find corner case (error case)
    }
  }
  // merge with additional sequence
  if (!additional_seqs.empty()) {
    sequences.insert(sequences.end(), 
                     additional_seqs.begin(), 
                     additional_seqs.end());
  }
  return true;
}

// [New]
bool PncMap::GetRouteSegmentsInSearchSpace(
  const std::array<int, 3>& ego_route_index,
  const double horizon,
  const double backward_extend_horizon,
  std::list<RouteSegments>& lane_sequences) {

  double epsilon = 1e-3;
  // distance_from_ego: from ego position to start of current laneSegment
  struct LaneNode{
    LaneNode (LaneInfoConstPtr p, double s, double d, bool r) 
      : lane_ptr(p), s_value(s), distance_from_ego(d), is_root(r) {}
    LaneInfoConstPtr lane_ptr;
    double s_value;
    double distance_from_ego;
    bool is_root;
  };

  std::map<std::string, LaneNode> visited_nodes;
  std::vector<LaneNode> open_stack;
  // TODO: check if adc_waypoint_ is valid
  open_stack.emplace_back(adc_waypoint_.lane, adc_waypoint_.s, 0.0, true);

  while(!open_stack.empty()) {
    auto curr_node = open_stack.back();
    open_stack.pop_back();
    auto lane_id = curr_node.lane_ptr->id().id(); // std::string

    auto it = visited_nodes.find(lane_id);
    if (it != visited_nodes.end()){
      // current node is not root if it can be access rather than lane change
      auto& node = it->second;
      node.is_root = node.is_root && curr_node.is_root;
      continue;
    }
    visited_nodes.insert(std::make_pair(lane_id, curr_node));

    for (auto neighbor : GetLeftAndRightNeighbors(curr_node.lane_ptr)) {
      if (all_lane_ids_.count(neighbor->id().id()) == 0) {
        continue;
      }
      // TODO: check if we can use curr_node.s_value instead (suppose 
      // neighbor share same length with current)
      open_stack.emplace_back(neighbor, curr_node.s_value, 
                              curr_node.distance_from_ego, true);
    }
    // check if remainning horizon is valid
    double dist_from_ego_to_succ = curr_node.distance_from_ego + 
                                   curr_node.lane_ptr->total_length() - curr_node.s_value;
    if (dist_from_ego_to_succ - epsilon >= horizon) {
      continue;
    }
    for (auto successor : GetSuccessors(curr_node.lane_ptr)) {
      if (all_lane_ids_.count(successor->id().id()) == 0) {
        continue;
      }
      // TODO: check if successor length valid
      open_stack.emplace_back(successor, 0.0, dist_from_ego_to_succ, false);
    }

  } // end while

  std::vector<LaneNode> roots;
  for (const auto& it : visited_nodes) {
    if (it.second.is_root) {
      roots.push_back(it.second);
    }
  }

  for (const auto& root : roots) {
    // search possible lane sequences from root
    auto ls = LaneSegment(root.lane_ptr, root.s_value, root.lane_ptr->total_length());
    auto remaining_h = horizon - root.distance_from_ego;
    std::list<RouteSegments> seqs;
    if (!GetPossibleSequencesFromLaneSegment(ls, remaining_h, true, seqs)) {
      AERROR << "Failed to find possible lane sequences from lane segment " << root.lane_ptr->id().id();
      return false;
    }
    if (seqs.empty()) {
      continue;
    }
    // search lane_segment(not in search space) backward, extend seqs
    ExtendLaneSequencesForLaneMerge(backward_extend_horizon, seqs);
    // merge list_sequence to final list_sequence.
    lane_sequences.insert(lane_sequences.end(), seqs.begin(), seqs.end());
    // TODO: check if successful
  }

  return true;
}

// [New]
bool PncMap::GetRouteSegmentsInHorizon(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments_candidates) {
  // 1. update adc_waypoint_ and adc_route_index_ by using 
  // matched lane segment information in routing result.
  if (!UpdateVehicleState(vehicle_state)) {
    AERROR << "Failed to update vehicle state in pnc_map.";
    return false;
  }
  // Vehicle has to be this close to lane center before considering change
  // lane
  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first.";
    return false;
  }

  // 2. get ego's route_index
  const auto &route_index = route_indices_[adc_route_index_].index; // return std::array<int, 3>
  const int road_index = route_index[0];
  const int passage_index = route_index[1];
  const auto &road = routing_.road(road_index);
  // get all possible lane sequences within horizon(forward_length)
  double backward_extend_horizon = 50.0;
  std::list<RouteSegments> drive_passages;
  GetRouteSegmentsInSearchSpace(route_index, 
                                forward_length, 
                                backward_extend_horizon, 
                                drive_passages);
  
  // 3. fill addiational information for each driving passage
  int index = -1;
  for (auto& segments : drive_passages) {
    ++index;
    // segments == lane segments == lane sequence

    // 3.1 preparation
    // TODO: mark route segment if ego can be driven into or not.
    // route segment can't driven is only used for agent association.
    bool is_ego_on_segments = (segments.at(0).lane == adc_waypoint_.lane);
    const PointENU nearest_point = 
      is_ego_on_segments
        ? adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s)
        : PointFactory::ToPointENU(adc_state_);
    
    // sl based on segments, segment_waypoint based on lane
    // TODO: if we can not interpolate nearest_point on segments
    // (extrapolate also means fail to interpolate), the projection fails.
    // In this case, we reset sl point with {0, 0}.
    common::SLPoint sl;
    LaneWaypoint segment_waypoint; // not use
    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
      // ADEBUG << "Failed to get projection from point: "
      //        << nearest_point.ShortDebugString();
      // TODO: if it works with set s=0 and l=0 
      sl.set_s(0.0);
      sl.set_l(0.0);
    }

    // 3.2 extend segments
    // TODO: corner case of lane sepration in front
    route_segments_candidates->emplace_back();
    auto& route_segments = route_segments_candidates->back();
    const auto last_waypoint = segments.LastWaypoint();
    if (!ExtendSegments(segments,
                        sl.s() - backward_length,
                        sl.s() + forward_length, 
                        &route_segments)) {
      AERROR << "Failed to extend segments with s=" << sl.s()
             << ", backward: " << backward_length
             << ", forward: " << forward_length;
      return false;
    }

    // 3.3 store information
    // if last point of originating segments is located on extended segments
    if (route_segments.IsWaypointOnSegment(last_waypoint)) {
      route_segments.SetRouteEndWaypoint(last_waypoint);
    }
    
    // TODO: features not use any more: CanExit, NextAction, PreviousAction
    route_segments.SetCanExit(true);
    route_segments.SetNextAction(routing::FORWARD);

    // TODO: check unique
    const std::string route_segment_id = absl::StrCat(road_index, "_", index);
    route_segments.SetId(route_segment_id);
    route_segments.SetStopForDestination(stop_for_destination_);

    if (is_ego_on_segments) {
      route_segments.SetIsOnSegment(true);
      route_segments.SetPreviousAction(routing::FORWARD);
    } else {
      route_segments.SetPreviousAction(routing::FORWARD);
    }
  }
  return !route_segments_candidates->empty();
}

/************************* for behavior planner:end *************************/


// RouteSegments 继承自 vector<LaneSegment>
// extract segments from routing
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments) {
  // 1.
  // 从routing中查找到（离ego状态）最近的道路点adc_waypoint，
  // 根据adc_waypoint来在route_indices_中查找到当前行驶至了routing的哪条LaneSegment上，
  // 并将“LaneSegment的index”赋给adc_route_index_
  if (!UpdateVehicleState(vehicle_state)) {
    AERROR << "Failed to update vehicle state in pnc_map.";
    return false;
  }
  // Vehicle has to be this close to lane center before considering change
  // lane
  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first.";
    return false;
  }

  // 2. 已经得到ego位置对应的route_index，
  // 根据这个route_index寻找road和passage，在以此找到这个passage的“邻居”
  const auto &route_index = route_indices_[adc_route_index_].index;
  const int road_index = route_index[0]; // RoadSegment
  const int passage_index = route_index[1]; // RoadSegment's Passage
  const auto &road = routing_.road(road_index);
  // Raw filter to find all neighboring passages
  // passage的邻居（变道关系的邻居，并且属于当前road）
  auto drive_passages = GetNeighborPassages(road, passage_index);

  // 3.
  // 一个passage对应一个route_segments(segments belong to this passage)
  // 因为drive_passages只考虑“变道邻居”，没有考虑“forward邻居”，
  for (const int index : drive_passages) {
    const auto &passage = road.passage(index);
    RouteSegments segments;
    // RouteSegments就是将Passage中LaneSegments，存进其数组里，并记录起始终止的s值
    if (!PassageToSegments(passage, &segments)) {
      ADEBUG << "Failed to convert passage to lane segments.";
      continue;
    }

    // 3.1 check validance
    const PointENU nearest_point =
        index == passage_index
            ? adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s)
            : PointFactory::ToPointENU(adc_state_);
    common::SLPoint sl;
    LaneWaypoint segment_waypoint;
    // sl定义了相对于segments的点(位置)，segment_waypoint定义了相对于lane的点(位置)
    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
      ADEBUG << "Failed to get projection from point: "
             << nearest_point.ShortDebugString();
      continue;
    }
    if (index != passage_index) {
      if (!segments.CanDriveFrom(adc_waypoint_)) {
        ADEBUG << "You cannot drive from current waypoint to passage: "
               << index;
        continue;
      }
    }

    // 3.2 extend segments
    // 新建、扩展route_segments（route_segments is passage that contains LaneSegment, or topoNode）
    // 这里的segments的扩展，可以包括一些不在routing范围内的LaneSegment
    // TODO: 不在routing范围内的“扩展”存在一些问题，例如：前方分叉的laneSegment
    route_segments->emplace_back();
    const auto last_waypoint = segments.LastWaypoint();
    if (!ExtendSegments(segments, sl.s() - backward_length,
                        sl.s() + forward_length, &route_segments->back())) {
      AERROR << "Failed to extend segments with s=" << sl.s()
             << ", backward: " << backward_length
             << ", forward: " << forward_length;
      return false;
    }

    // 3.3 store information
    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
      route_segments->back().SetRouteEndWaypoint(last_waypoint);
    }
    route_segments->back().SetCanExit(passage.can_exit());
    route_segments->back().SetNextAction(passage.change_lane_type());
    const std::string route_segment_id = absl::StrCat(road_index, "_", index);
    route_segments->back().SetId(route_segment_id);
    route_segments->back().SetStopForDestination(stop_for_destination_);
    // 
    if (index == passage_index) {
      route_segments->back().SetIsOnSegment(true);
      route_segments->back().SetPreviousAction(routing::FORWARD);
    } else if (sl.l() > 0) {
      route_segments->back().SetPreviousAction(routing::RIGHT);
    } else {
      route_segments->back().SetPreviousAction(routing::LEFT);
    }
  }
  return !route_segments->empty();
}

bool PncMap::GetNearestPointFromRouting(const VehicleState &state,
                                        LaneWaypoint *waypoint) const {
  const double kMaxDistance = 10.0;  // meters.
  const double kHeadingBuffer = M_PI / 10.0;
  waypoint->lane = nullptr;
  std::vector<LaneInfoConstPtr> lanes;
  const auto point = PointFactory::ToPointENU(state);
  // 这里，apollo用distance和heading来筛选符合要求的lane，
  // TODO: 但是，我们可以事先组织lane，预先计算point(ego位置)和lane的association，以此找到关联的lane
  const int status =
      hdmap_->GetLanesWithHeading(point, kMaxDistance, state.heading(),
                                  M_PI / 2.0 + kHeadingBuffer, &lanes);
  ADEBUG << "lanes:" << lanes.size();
  if (status < 0) {
    AERROR << "Failed to get lane from point: " << point.ShortDebugString();
    return false;
  }
  if (lanes.empty()) {
    AERROR << "No valid lane found within " << kMaxDistance
           << " meters with heading " << state.heading();
    return false;
  }

  // 筛选合法的lane（合法的lane或者存在于range_lane_ids_，或者存在于all_lane_ids_）
  std::vector<LaneInfoConstPtr> valid_lanes;
  std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
               [&](LaneInfoConstPtr ptr) {
                 return range_lane_ids_.count(ptr->lane().id().id()) > 0;
               });
  if (valid_lanes.empty()) {
    std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
                 [&](LaneInfoConstPtr ptr) {
                   return all_lane_ids_.count(ptr->lane().id().id()) > 0;
                 });
  }

  // Get nearest_waypoints for current position
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto &lane : valid_lanes) {
    if (range_lane_ids_.count(lane->id().id()) == 0) {
      continue;
    }
    {
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        AERROR << "fail to get projection";
        return false;
      }
      // Use large epsilon to allow projection diff
      static constexpr double kEpsilon = 0.5;
      if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
        continue;
      }
    }
    double distance = 0.0;
    common::PointENU map_point =
        lane->GetNearestPoint({point.x(), point.y()}, &distance);
    // 选择一个离ego point最近的lane，以此生成waypoint
    if (distance < min_distance) {
      min_distance = distance;
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({map_point.x(), map_point.y()}, &s, &l)) {
        AERROR << "Failed to get projection for map_point: "
               << map_point.DebugString();
        return false;
      }
      waypoint->lane = lane;
      waypoint->s = s;
    }
    ADEBUG << "distance" << distance;
  }
  if (waypoint->lane == nullptr) {
    AERROR << "Failed to find nearest point: " << point.ShortDebugString();
  }
  return waypoint->lane != nullptr;
}

// GetRouteSuccessor和GetRoutePredecessor优先选择“在routing内的”后继、前驱LaneInfo
LaneInfoConstPtr PncMap::GetRouteSuccessor(LaneInfoConstPtr lane) const {
  if (lane->lane().successor_id().empty()) {
    return nullptr;
  }
  hdmap::Id preferred_id = lane->lane().successor_id(0);
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (range_lane_ids_.count(lane_id.id()) != 0) {
      preferred_id = lane_id;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

LaneInfoConstPtr PncMap::GetRoutePredecessor(LaneInfoConstPtr lane) const {
  if (lane->lane().predecessor_id().empty()) {
    return nullptr;
  }

  std::unordered_set<std::string> predecessor_ids;
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    predecessor_ids.insert(lane_id.id());
  }

  hdmap::Id preferred_id = lane->lane().predecessor_id(0);
  for (size_t i = 1; i < route_indices_.size(); ++i) {
    auto &lane = route_indices_[i].segment.lane->id();
    if (predecessor_ids.count(lane.id()) != 0) {
      preferred_id = lane;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}


bool PncMap::ExtendSegments(const RouteSegments &segments,
                            const common::PointENU &point, double look_backward,
                            double look_forward,
                            RouteSegments *extended_segments) {
  common::SLPoint sl;
  LaneWaypoint waypoint;
  if (!segments.GetProjection(point, &sl, &waypoint)) {
    AERROR << "point: " << point.ShortDebugString() << " is not on segment";
    return false;
  }
  return ExtendSegments(segments, sl.s() - look_backward, sl.s() + look_forward,
                        extended_segments);
}

// 参数start_s、end_s指相对于segments上的位置
bool PncMap::ExtendSegments(const RouteSegments &segments,
                            double start_s,
                            double end_s,
                            RouteSegments *const truncated_segments) const {
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }
  CHECK_NOTNULL(truncated_segments);
  // 复制RouteSegments的各种属性
  truncated_segments->SetProperties(segments);

  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }
  // record all unique MapLaneSegment of RouteSegments
  std::unordered_set<std::string> unique_lanes;
  static constexpr double kRouteEpsilon = 1e-3;
  // Extend the trajectory towards the start of the trajectory.
  // 当start_s为负数时，需要扩展到当前laneInfo的前一条laneInfo(可能不在routing内)
  if (start_s < 0) {
    const auto &first_segment = *segments.begin();
    auto lane = first_segment.lane;
    // extend_s is the length we need to extend backward
    // s is the remainning length we can extend backward on current MapLaneSegment
    double s = first_segment.start_s;
    double extend_s = -start_s;
    std::vector<LaneSegment> extended_lane_segments;
    while (extend_s > kRouteEpsilon) {
      if (s <= kRouteEpsilon) {
        // 有可能找到“不在routing内的”laneInfo
        lane = GetRoutePredecessor(lane);
        if (lane == nullptr ||
            unique_lanes.find(lane->id().id()) != unique_lanes.end()) {
          break;
        }
        s = lane->total_length();
      } else {
        // "length" is the length we can extend on "lane"
        const double length = std::min(s, extend_s);
        extended_lane_segments.emplace_back(lane, s - length, s);
        extend_s -= length;
        s -= length;
        unique_lanes.insert(lane->id().id());
      }
    }
    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }
  
  bool found_loop = false;
  double router_s = 0;
  // visit all LaneSegment of (const)RouteSegments
  for (const auto &lane_segment : segments) 
  {
    // router_s: accumulated length of all lane_segments visited previously
    // start_s(或end_s) - router_s: 表示start_s(或end_s)在当前LS的坐标，
    // + lane_segment.start_s: 表示把“当前LS的坐标”转成“当前MapLS的坐标”，
    // 因为lane_segment.start_s、end_s表示的就是“当前MapLS的坐标”，这样比较才有意义。
    const double adjusted_start_s = std::max(
        start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s =
        std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    
    if (adjusted_start_s < adjusted_end_s) 
    {
      // merge current LaneSegment with previous if two belong to same MapLS 
      if (!truncated_segments->empty() &&
          truncated_segments->back().lane->id().id() == lane_segment.lane->id().id()) 
      {
        truncated_segments->back().end_s = adjusted_end_s;
      }
      else if (unique_lanes.find(lane_segment.lane->id().id()) == unique_lanes.end()) 
      {
        truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s,
                                         adjusted_end_s);
        unique_lanes.insert(lane_segment.lane->id().id());
      }
      else
      {
        found_loop = true;
        break;
      }
    }
    // router_s上累加当前LaneSegment的长度（可能是partial长度）。
    router_s += (lane_segment.end_s - lane_segment.start_s);
    if (router_s > end_s) {
      break;
    }
  } // end for loop to LaneSegment
  if (found_loop) {
    return true;
  }
  // Extend the trajectory towards the end of the trajectory.
  // truncated_segments里面的每个segment，似乎对应一条unique lane，以下步骤只是更新last lane的end_s
  if (router_s < end_s && !truncated_segments->empty()) {
    auto &back = truncated_segments->back();
    if (back.lane->total_length() > back.end_s) {
      double origin_end_s = back.end_s;
      back.end_s =
          std::min(back.end_s + end_s - router_s, back.lane->total_length());
      router_s += back.end_s - origin_end_s;
    }
  }
  auto last_lane = segments.back().lane;
  // 如果end_s大于当前LaneSegments的总长，则向后扩展
  while (router_s < end_s - kRouteEpsilon) {
    last_lane = GetRouteSuccessor(last_lane);
    if (last_lane == nullptr ||
        unique_lanes.find(last_lane->id().id()) != unique_lanes.end()) {
      break;
    }
    const double length = std::min(end_s - router_s, last_lane->total_length());
    truncated_segments->emplace_back(last_lane, 0, length);
    unique_lanes.insert(last_lane->id().id());
    router_s += length;
  }
  return true;
}

void PncMap::AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s,
                                const double end_s,
                                std::vector<MapPathPoint> *const points) {
  if (points == nullptr || start_s >= end_s) {
    return;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points->emplace_back(lane->points()[i], lane->headings()[i],
                           LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points->emplace_back(segment.start() + segment.unit_direction() *
                                                   (start_s - accumulate_s),
                             lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points->emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}

}  // namespace hdmap
}  // namespace apollo
