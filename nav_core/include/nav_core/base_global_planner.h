/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_CORE_BASE_GLOBAL_PLANNER_H
#define NAV_CORE_BASE_GLOBAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace nav_core {
  /**
   * @class BaseGlobalPlanner （全局规划器，所有实现全局规划器插件的父类）
   * @brief Provides an interface for global planners used in navigation. 
   *        All global planners written as plugins for the navigation stack must adhere to this interface.
   */
  class BaseGlobalPlanner{
    public:
      /**
       * @brief Given a goal pose in the world, compute a plan.(根据给定的全局目标点来计算生成一个计划)
       * @param start The start pose （其实位姿）
       * @param goal The goal pose （目标位姿）
       * @param plan The plan... filled by the planner （规划器的实例对象）
       * @return True if a valid plan was found, false otherwise
       */
      virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @param cost The plans calculated cost（该条计划的成本）
       * @return True if a valid plan was found, false otherwise
       */
      virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
                            double& cost)
      {
        cost = 0;
        return makePlan(start, goal, plan);
      }

      /**
       * @brief  Initialization function for the BaseGlobalPlanner（全局计划器的初始化函数）
       * @param  name The name of this planner （全局规划器的名称）
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) = 0;

      /**
       * @brief  Virtual destructor for the interface（析构函数）
       */
      virtual ~BaseGlobalPlanner(){}

    protected:
      BaseGlobalPlanner(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_BASE_GLOBAL_PLANNER_H