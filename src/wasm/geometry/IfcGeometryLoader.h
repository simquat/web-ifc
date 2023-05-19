/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.  */

#pragma once

#include <unordered_map>
#include <vector>
#include <optional>
#include <glm/glm.hpp>

#include "../parsing/IfcLoader.h"
#include "../schema/IfcSchemaManager.h"
#include "representation/geometry.h"


 // This class takes care of loading raw unprocessed geometry data from the IFC loader

namespace webifc::geometry
{

  class IfcGeometryLoader 
  {
  public:
    IfcGeometryLoader(const webifc::parsing::IfcLoader &loader, webifc::utility::LoaderErrorHandler &errorHandler,const webifc::schema::IfcSchemaManager &schemaManager,uint16_t circleSegments);
    template<size_t N> glm::mat<N+1,N+1,double> GetPlacement(const uint32_t expressID) const;
    template<size_t N> glm::vec<N,double> GetCartesianPoint(const uint32_t expressID) const;
    template<size_t N> std::vector<glm::vec<N,double>> GetIfcCartesianPointList(const uint32_t expressID) const;
    std::optional<glm::dvec4> GetColor(const uint32_t expressID) const;
    template<size_t N> IfcCurve<N>& GetCurve(uint32_t expressID, const IfcCurve<N> curve = {}, const bool edge=false, const int sameSense = -1, const int trimSense = -1, const std::optional<IfcTrimmingSelect<N>> startTrim = {}, const std::optional<IfcTrimmingSelect<N>> endTrim = {}) const;
    template<size_t N> IfcProfile<N> GetProfile(uint32_t expressID) const;
    template<size_t N> IfcBound<N> GetBound(const uint32_t expressID) const;
    template<size_t N> IfcAlignment<N> GetAlignment(const uint32_t expressID, const IfcAlignment<N> alignment = IfcAlignment<N>(), const glm::dmat4 transform = glm::dmat4(1)) const;
    const std::unordered_map<uint32_t, std::vector<uint32_t>> &GetRelVoids() const;
    const std::unordered_map<uint32_t, std::vector<uint32_t>> &GetRelVoidRels() const;
    const std::unordered_map<uint32_t, std::vector<uint32_t>> &GetRelAggregates() const;
    const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> &GetStyledItems() const;
    const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> &GetRelMaterials() const;
    const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> &GetMaterialDefinitions() const;
    double GetLinearScalingFactor() const;
  private:
    template<size_t N> IfcLoop<N> GetLoop(const uint32_t expressID) const;
    glm::dvec3 GetVector(const uint32_t expressID) const;
    template<size_t N> IfcCurve<N> GetOrientedEdge(uint32_t expressID) const;
    template<size_t N> IfcCurve<N> GetEdge(uint32_t expressID) const;
    IfcCurve<2> GetAlignmentCurve(uint32_t expressID) const;
    glm::dvec3 GetVertexPoint(uint32_t expressID) const;
    template<size_t N> IfcTrimmingSelect<N> GetTrimSelect(std::vector<uint32_t> &tapeOffsets) const;
    std::vector<IfcSegmentIndexSelect> ReadCurveIndices() const;
    const webifc::parsing::IfcLoader &_loader;
    webifc::utility::LoaderErrorHandler &_errorHandler;
    const webifc::schema::IfcSchemaManager &_schemaManager;
    const std::unordered_map<uint32_t, std::vector<uint32_t>> _relVoidRel;
    const std::unordered_map<uint32_t, std::vector<uint32_t>> _relVoids;
    const std::unordered_map<uint32_t, std::vector<uint32_t>> _relAggregates;
    const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> _styledItems;
    const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> _relMaterials;
    const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> _materialDefinitions;
    double _linearScalingFactor = 1;
    double _squaredScalingFactor = 1;
    double _cubicScalingFactor = 1;
    double _angularScalingFactor = 1;
    uint16_t _circleSegments;
    std::unordered_map<uint32_t, std::vector<uint32_t>> PopulateRelVoidsMap();
    std::unordered_map<uint32_t, std::vector<uint32_t>> PopulateRelVoidsRelMap();
    std::unordered_map<uint32_t, std::vector<uint32_t>> PopulateRelAggregatesMap();
    std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> PopulateStyledItemMap();
    std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> PopulateRelMaterialsMap();
    std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> PopulateMaterialDefinitionsMap();
    void ReadLinearScalingFactor();
    double ConvertPrefix(const std::string &prefix);
  };
  
}