/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, PickNik LLC
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
*   * Neither the name of PickNik LLC nor the names of its
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
*********************************************************************/

/* Author: Henning Kayser */
/* Description: Support for storing and loading planner data for PRM-based planners. */

#pragma once

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/tools/config/SelfConfig.h>
#include <type_traits>

namespace ompl
{
namespace geometric
{
class PRMPersist : public PRM
{
public:
  PRMPersist(const base::PlannerData& data, bool starStrategy = false) : PRM(data.getSpaceInformation(), starStrategy)
  {
    if (data.numVertices() > 0)
    {
      std::map<unsigned int, Vertex> vertices;
      const auto& si = data.getSpaceInformation();
      const auto& getOrCreateVertex = [&](unsigned int vertex_index) {
        if (!vertices.count(vertex_index))
        {
          const auto& data_vertex = data.getVertex(vertex_index);
          Vertex graph_vertex = boost::add_vertex(g_);
          stateProperty_[graph_vertex] = si->cloneState(data_vertex.getState());
          totalConnectionAttemptsProperty_[graph_vertex] = 1;
          successfulConnectionAttemptsProperty_[graph_vertex] = 0;
          vertices[vertex_index] = graph_vertex;
        }
        return vertices.at(vertex_index);
      };

      specs_.multithreaded = false;
      nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
      specs_.multithreaded = true;
      nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });

      for (size_t vertex_index = 0; vertex_index < data.numVertices(); ++vertex_index)
      {
        Vertex m = getOrCreateVertex(vertex_index);
        std::vector<unsigned int> neighbor_indices;
        data.getEdges(vertex_index, neighbor_indices);
        if (neighbor_indices.empty())
        {
          disjointSets_.make_set(m);
        }
        else
        {
          for (const unsigned int neighbor_index : neighbor_indices)
          {
            Vertex n = getOrCreateVertex(neighbor_index);
            totalConnectionAttemptsProperty_[n]++;
            successfulConnectionAttemptsProperty_[n]++;
            base::Cost weight;
            data.getEdgeWeight(vertex_index, neighbor_index, &weight);
            const Graph::edge_property_type properties(weight);
            boost::add_edge(m, n, properties, g_);
            uniteComponents(m, n);
          }
        }
        nn_->add(m);
      }
    }
  };
  PRMPersist(const base::SpaceInformationPtr& si, bool starStrategy = false) : PRM(si, starStrategy){};
};
class PRMStarPersist : public PRMPersist
{
public:
  PRMStarPersist(const base::PlannerData& data) : PRMPersist(data, true){};
  PRMStarPersist(const base::SpaceInformationPtr& si) : PRMPersist(si, true){};
};

class LazyPRMPersist : public LazyPRM
{
public:
  LazyPRMPersist(const base::PlannerData& data, bool starStrategy = false)
    : LazyPRM(data.getSpaceInformation(), starStrategy)
  {
    if (data.numVertices() > 0)
    {
      std::map<unsigned int, Vertex> vertices;
      const auto& si = data.getSpaceInformation();
      const auto& getOrCreateVertex = [&](unsigned int vertex_index) {
        if (!vertices.count(vertex_index))
        {
          const auto& data_vertex = data.getVertex(vertex_index);
          Vertex graph_vertex = boost::add_vertex(g_);
          stateProperty_[graph_vertex] = si->cloneState(data_vertex.getState());
          vertexValidityProperty_[graph_vertex] = VALIDITY_UNKNOWN;
          unsigned long int newComponent = componentCount_++;
          vertexComponentProperty_[graph_vertex] = newComponent;
          vertices[vertex_index] = graph_vertex;
        }
        return vertices.at(vertex_index);
      };

      specs_.multithreaded = false;
      nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
      specs_.multithreaded = true;
      nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });

      for (size_t vertex_index = 0; vertex_index < data.numVertices(); ++vertex_index)
      {
        Vertex m = getOrCreateVertex(vertex_index);
        std::vector<unsigned int> neighbor_indices;
        data.getEdges(vertex_index, neighbor_indices);
        for (const unsigned int neighbor_index : neighbor_indices)
        {
          Vertex n = getOrCreateVertex(neighbor_index);
          base::Cost weight;
          data.getEdgeWeight(vertex_index, neighbor_index, &weight);
          const Graph::edge_property_type properties(weight);
          const Edge& edge = boost::add_edge(m, n, properties, g_).first;
          edgeValidityProperty_[edge] = VALIDITY_UNKNOWN;
          uniteComponents(m, n);
        }
        nn_->add(m);
      }
    }
  };
  LazyPRMPersist(const base::SpaceInformationPtr& si, bool starStrategy = false) : LazyPRM(si, starStrategy){};
};
class LazyPRMStarPersist : public LazyPRMPersist
{
public:
  LazyPRMStarPersist(const base::PlannerData& data) : LazyPRMPersist(data, true){};
  LazyPRMStarPersist(const base::SpaceInformationPtr& si) : LazyPRMPersist(si, true){};
};
}
}
namespace
{
template <typename T>
inline ompl::base::Planner* allocatePersistingPlanner(const ompl::base::PlannerData& data)
{
  return NULL;
};
template <>
inline ompl::base::Planner* allocatePersistingPlanner<ompl::geometric::PRMPersist>(const ompl::base::PlannerData& data)
{
  return new ompl::geometric::PRMPersist(data);
};
template <>
inline ompl::base::Planner*
allocatePersistingPlanner<ompl::geometric::PRMStarPersist>(const ompl::base::PlannerData& data)
{
  return new ompl::geometric::PRMStarPersist(data);
};
template <>
inline ompl::base::Planner*
allocatePersistingPlanner<ompl::geometric::LazyPRMPersist>(const ompl::base::PlannerData& data)
{
  return new ompl::geometric::LazyPRMPersist(data);
};
template <>
inline ompl::base::Planner*
allocatePersistingPlanner<ompl::geometric::LazyPRMStarPersist>(const ompl::base::PlannerData& data)
{
  return new ompl::geometric::LazyPRMStarPersist(data);
};
}
