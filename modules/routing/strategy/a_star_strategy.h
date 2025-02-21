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

#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/routing/strategy/strategy.h"

namespace apollo {
namespace routing {

class AStarStrategy : public Strategy {
 public:
  explicit AStarStrategy(bool enable_change);
  ~AStarStrategy() = default;

  virtual bool Search(const TopoGraph* graph, const SubTopoGraph* sub_graph,
                      const TopoNode* src_node, const TopoNode* dest_node,
                      std::vector<NodeWithRange>* const result_nodes);

/***************** search space for behavior planner:beg *****************/
  /**
   * @brief [New]
   * @param start_route_idx the road index from which searching will start
   * @return the route index of matched road of the node, return -1 if no match
   */
  int isNodeInRoute(
    const TopoNode* node,
    const size_t start_route_idx,
    const std::vector<std::string>& road_ids_in_route,
    const bool is_forward) const;
  
  /**
   * @brief [New] search space is a set of lane segments that ego can drive in.
   * @param[out] search_space_nodes nodes of search space
   * @param[out] search_space_edges edges of search space
   */
  bool GetSearchSpace(
    const TopoNode* src_node,
    const TopoNode* dest_node,
    const SubTopoGraph* sub_graph,
    const std::vector<NodeWithRange>& result_nodes,
    std::unordered_set<const TopoNode*>& search_space_nodes,
    std::unordered_set<const TopoEdge*>& search_space_edges);

  /**
   * @brief [New]
   */
  virtual bool GetParallelSearchSpace(
    const TopoNode* src_node, 
    const TopoNode* dest_node, 
    const SubTopoGraph* sub_graph,
    const std::vector<NodeWithRange>& result_nodes,
    std::vector<std::vector<NodeWithRange>>& search_space);
/***************** search space for behavior planner:end *****************/

 private:
  void Clear();
  double HeuristicCost(const TopoNode* src_node, const TopoNode* dest_node);
  double GetResidualS(const TopoNode* node);
  double GetResidualS(const TopoEdge* edge, const TopoNode* to_node);

 private:
  bool change_lane_enabled_;
  std::unordered_set<const TopoNode*> open_set_;
  std::unordered_set<const TopoNode*> closed_set_;
  std::unordered_map<const TopoNode*, const TopoNode*> came_from_;
  std::unordered_map<const TopoNode*, double> g_score_;
  std::unordered_map<const TopoNode*, double> enter_s_;
};

}  // namespace routing
}  // namespace apollo
