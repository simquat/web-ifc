/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.   */

#include "IfcGeometryLoader.h"
#include "operations/curve-utils.h"
#ifdef DEBUG_DUMP_SVG
    #include "../test/io_helpers.h"
#endif


namespace webifc::geometry
{

  IfcGeometryLoader::IfcGeometryLoader(const webifc::parsing::IfcLoader &loader, webifc::utility::LoaderErrorHandler &errorHandler,const webifc::schema::IfcSchemaManager &schemaManager, uint16_t circleSegments) 
    :_loader(loader),_errorHandler(errorHandler),_schemaManager(schemaManager), _relVoidRel(PopulateRelVoidsRelMap()), _relVoids(PopulateRelVoidsMap()), _relAggregates(PopulateRelAggregatesMap()), 
    _styledItems(PopulateStyledItemMap()), _relMaterials(PopulateRelMaterialsMap()), _materialDefinitions(PopulateMaterialDefinitionsMap()), _circleSegments(circleSegments)
  {
    ReadLinearScalingFactor();
  }

  template<size_t N> 
  IfcAlignment<N> IfcGeometryLoader::GetAlignment(const uint32_t expressID, const IfcAlignment<N> alignment, const glm::dmat4 transform) const
    {
      auto lineID = _loader.ExpressIDToLineID(expressID);
      auto &line = _loader.GetLine(lineID);

      switch (line.ifcType)
      {
      case schema::IFCALIGNMENT:
      {
        _loader.MoveToArgumentOffset(line, 5);
        uint32_t localPlacement = 0;
        if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          localPlacement = _loader.GetRefArgument();
        }

        glm::dmat4 transform_t = glm::dmat4(1);
        if (localPlacement != 0 && _loader.IsValidExpressID(localPlacement))
        {
          transform_t = GetPlacement<N>(localPlacement);
        }

        
        auto &relAggVector = GetRelAggregates();
        if (relAggVector.count(line.expressID) == 1) 
        {
          auto &relAgg = relAggVector.at(line.expressID);
          for (auto expressID : relAgg)
          {
            alignment = GetAlignment(expressID, alignment, transform * transform_t);
          }
        }

        break;
      }
      case schema::IFCALIGNMENTHORIZONTAL:
      {
        _loader.MoveToArgumentOffset(line, 5);
        uint32_t localPlacement = 0;
        if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          localPlacement = _loader.GetRefArgument();
        }

        glm::dmat4 transform_t = glm::dmat4(1);
        if (localPlacement != 0 && _loader.IsValidExpressID(localPlacement))
        {
          transform_t = GetPlacement<N>(localPlacement);
        }

        auto &relAggVector = GetRelAggregates();
        if (relAggVector.count(line.expressID) == 1)
        { 
          auto &relAgg = relAggVector.at(line.expressID);
          for (auto expressID : relAgg)
          {
            alignment.Horizontal.curves.push_back(GetAlignmentCurve(expressID));
          }

          for (size_t i = 0; i < alignment.Horizontal.curves.size(); i++)
          {
            for (size_t j = 0; j < alignment.Horizontal.curves[i].size(); j++)
            {
              alignment.Horizontal.curves[i][j] =
                glm::dvec4(alignment.Horizontal.curves[i][j].x, alignment.Horizontal.curves[i][j].y, 0, 1) * transform * transform_t;
            }
          }
        }

        break;
      }
      case schema::IFCALIGNMENTVERTICAL:
      {
        _loader.MoveToArgumentOffset(line, 5);
        uint32_t localPlacement = 0;
        if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          localPlacement = _loader.GetRefArgument();
        }

        glm::dmat4 transform_t = glm::dmat4(1);
        if (localPlacement != 0 && _loader.IsValidExpressID(localPlacement))
        {
          transform_t = GetPlacement<3>(localPlacement);
        }

        auto &relAggVector = GetRelAggregates();
        if (relAggVector.count(line.expressID) == 1) 
        {
          auto &relAgg = relAggVector.at(line.expressID);
          for (auto expressID : relAgg)
          {
            alignment.Vertical.curves.push_back(GetAlignmentCurve(expressID));
          }

          for (size_t i = 0; i < alignment.Vertical.curves.size(); i++)
          {
            for (size_t j = 0; j < alignment.Vertical.curves[i].size(); j++)
            {
              alignment.Vertical.curves[i][j] =
                glm::dvec4(alignment.Vertical.curves[i][j].x,alignment.Vertical.curves[i][j].y, 0, 1) * transform * transform_t;
            }
          }
        }

        break;
      }
      default:
      {
        break;
      }
      }

      return alignment;
    }

    IfcCurve<2> IfcGeometryLoader::GetAlignmentCurve(uint32_t expressID) const
    {
      auto lineID = _loader.ExpressIDToLineID(expressID);
      auto &line = _loader.GetLine(lineID);

      IfcCurve<2> alignmentCurve;

      switch (line.ifcType)
      {
      case schema::IFCALIGNMENTSEGMENT:
      {

        _loader.MoveToArgumentOffset(line, 5);
        uint32_t localPlacement = 0;
        if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          localPlacement = _loader.GetRefArgument();
        }

        glm::dmat4 transform_t = glm::dmat4(1);
        if (localPlacement != 0 && _loader.IsValidExpressID(localPlacement))
        {
          transform_t = GetPlacement<3>(localPlacement);
        }

        _loader.MoveToArgumentOffset(line, 7);
        uint32_t curveID = 0;
        if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          curveID = _loader.GetRefArgument();
        }
        if (curveID != 0 && _loader.IsValidExpressID(curveID))
        {
          alignmentCurve = GetAlignmentCurve(curveID);
        }

        for (size_t i = 0; i < alignmentCurve.size(); i++)
        {
          alignmentCurve[i] = glm::dvec4(alignmentCurve[i].x,alignmentCurve[i].y, 0, 1) * transform_t;
        }

        break;
      }
      case schema::IFCALIGNMENTHORIZONTALSEGMENT:
      {

        _loader.MoveToArgumentOffset(line, 8);
        std::string type = _loader.GetStringArgument();

        _loader.MoveToArgumentOffset(line, 2);
        uint32_t ifcStartPoint = _loader.GetRefArgument();
        glm::dvec2 StartPoint = GetCartesianPoint<2>(ifcStartPoint);

        _loader.MoveToArgumentOffset(line, 3);
        double ifcStartDirection = _loader.GetDoubleArgument();

        _loader.MoveToArgumentOffset(line, 4);
        double StartRadiusOfCurvature = _loader.GetDoubleArgument();

        _loader.MoveToArgumentOffset(line, 5);
        double EndRadiusOfCurvature = _loader.GetDoubleArgument();

        _loader.MoveToArgumentOffset(line, 6);
        double SegmentLength = _loader.GetDoubleArgument();

        _loader.MoveToArgumentOffset(line, 7);
        double GravityCenterLineHeight = _loader.GetDoubleArgument();

        switch (Horizontal_alignment_type.at(type))
        {
        default:
        {
          break;
        }
        case 1: // LINE
        {

          IfcCurve<2> curve;
          glm::dvec2 Direction(
            glm::cos(ifcStartDirection),
            glm::sin(ifcStartDirection));
          glm::dvec2 EndPoint = StartPoint + Direction * SegmentLength;

          curve.push_back(StartPoint);
          curve.push_back(EndPoint);

          alignmentCurve = curve;

          break;
        }
        case 2: // ARC
        {

          IfcCurve<2> curve;
          double span = (SegmentLength / StartRadiusOfCurvature);
          ifcStartDirection = ifcStartDirection - (CONST_PI / 2);

          bool sw = true;
          auto curve2D = GetEllipseCurve<2>(StartRadiusOfCurvature, StartRadiusOfCurvature, 20, glm::dmat3(1), ifcStartDirection, ifcStartDirection + span, sw);
          glm::dvec2 desp = glm::dvec2(StartPoint.x - curve2D[0].x, StartPoint.y - curve2D[0].y);

          for (size_t i=0; i < curve2D.size();i++)
          {
            curve.push_back(curve2D[i] + desp);
          }


          alignmentCurve = curve;

          break;
        }
        case 3: // CLOTHOID
        {
          bool inverse = false;
          if (abs(StartRadiusOfCurvature) > abs(EndRadiusOfCurvature))
          {
            inverse = true;
          }
          IfcCurve<2> curve;
          double A = sqrt(abs(EndRadiusOfCurvature - StartRadiusOfCurvature) * SegmentLength);
          double Api = A * sqrt(CONST_PI);
          double uMax = SegmentLength / Api;

          double s = A * uMax * sqrt(CONST_PI);
          double radFin = (A * A * A) / (A * s);

          double vSin = 0;
          double vCos = 0;

          glm::dvec2 DirectionX(
            glm::cos(ifcStartDirection),
            glm::sin(ifcStartDirection));
          glm::dvec2 DirectionY(
            -glm::sin(ifcStartDirection),
            glm::cos(ifcStartDirection));

          if (EndRadiusOfCurvature < 0 || StartRadiusOfCurvature < 0)
          {
            DirectionY.x = -DirectionY.x;
            DirectionY.y = -DirectionY.y;
          }

          if (inverse)
          {
            DirectionX.x = -DirectionX.x;
            DirectionX.y = -DirectionX.y;
          }

          double def = 1000;
          double dif = def / 10;
          double count = 0;
          double tram = uMax / def;
          glm::dvec2 end(0, 0);
          glm::dvec2 prev(0, 0);
          glm::dvec2 endDir;
          for (double c = 1; c < def + 1; c++)
          {
            prev = end;
            end = StartPoint + Api * (DirectionX * vCos + DirectionY * vSin);
            if (c == def || c == 1 || count >= dif)
            {
              curve.push_back(end);
              count = 0;
            }
            if (c == def)
            {
              endDir = prev - end;
            }
            double val = c * tram;
            vSin += sin(CONST_PI * ((A * val * val) / (2 * abs(A)))) * tram;
            vCos += cos(CONST_PI * ((A * val * val) / (2 * abs(A)))) * tram;
            count++;
          }

          if (inverse)
          {
            DirectionX.x = -DirectionX.x;
            DirectionX.y = -DirectionX.y;

            glm::dvec2 newDirectionX(
              endDir.x,
              endDir.y);
            glm::dvec2 newDirectionY(
              -endDir.y,
              endDir.x);

            if (EndRadiusOfCurvature < 0 || StartRadiusOfCurvature < 0)
            {
              newDirectionY.x = -newDirectionY.x;
              newDirectionY.y = -newDirectionY.y;
            }

            newDirectionX = glm::normalize(newDirectionX);
            newDirectionY = glm::normalize(newDirectionY);

            for (uint32_t i = 0; i < curve.size(); i++)
            {
              double xx = curve[i].x - end.x;
              double yy = curve[i].y - end.y;
              double dx = xx * newDirectionX.x + yy * newDirectionX.y;
              double dy = xx * newDirectionY.x + yy * newDirectionY.y;
              double newDx = StartPoint.x + DirectionX.x * dx + DirectionY.x * dy;
              double newDy = StartPoint.y + DirectionX.y * dx + DirectionY.y * dy;
              curve[i].x = newDx;
              curve[i].y = newDy;
            }
          }

          alignmentCurve = curve;

          break;
        }
        case 4: // CUBIC
        {
          break;
        }
        case 5:
        {
          break;
        }
        case 6:
        {
          break;
        }
        case 7:
        {
          break;
        }
        case 8:
        {
          break;
        }
        case 9:
        {
          break;
        }
        }

        break;
      }
      case schema::IFCALIGNMENTVERTICALSEGMENT:
      {
        _loader.MoveToArgumentOffset(line, 2);
        double StartDistAlong = _loader.GetDoubleArgument();

        _loader.MoveToArgumentOffset(line, 3);
        double HorizontalLength = _loader.GetDoubleArgument();

        _loader.MoveToArgumentOffset(line, 4);
        double StartHeight = _loader.GetDoubleArgument();

        _loader.MoveToArgumentOffset(line, 5);
        double StartGradient = _loader.GetDoubleArgument();

        _loader.MoveToArgumentOffset(line, 6);
        double EndGradient = _loader.GetDoubleArgument();

        _loader.MoveToArgumentOffset(line, 7);
        double RadiusOfCurvature = _loader.GetDoubleArgument();

        _loader.MoveToArgumentOffset(line, 8);
        std::string type = _loader.GetStringArgument();

        IfcProfile<2> profile;

        switch (Vertical_alignment_type.at(type))
        {
        default:
        {
          break;
        }
        case 1: // CONSTANTGRADIENT
        {
          IfcCurve<2> curve;

          glm::dvec3 iPoint = glm::dvec3(StartDistAlong, StartHeight, 1);
          glm::dvec3 jPoint = glm::dvec3(StartDistAlong + HorizontalLength, StartHeight + HorizontalLength * StartGradient, 1);

          curve.push_back(iPoint);
          curve.push_back(jPoint);

          alignmentCurve = curve;

          break;
        }
        case 2: // ARC
        {
          IfcCurve<2> curve;

          double ifcStartDirection = atan(StartGradient);
          double ifcEndDirection = atan(EndGradient);

          ifcStartDirection = ifcStartDirection - (CONST_PI / 2);
          ifcEndDirection = ifcEndDirection - (CONST_PI / 2);

          glm::dvec2 StartPoint(StartDistAlong, StartHeight);

          bool sw = true;
          if (ifcStartDirection > ifcEndDirection)
          {
            ifcStartDirection = ifcStartDirection + CONST_PI;
            ifcEndDirection = ifcEndDirection + CONST_PI;
          }
          auto curve2D = GetEllipseCurve<2>(RadiusOfCurvature, RadiusOfCurvature, _circleSegments, glm::dmat3(1), ifcStartDirection, ifcEndDirection, sw);
          glm::dvec2 desp = glm::dvec2(StartPoint.x - curve2D[0].x, StartPoint.y - curve2D[0].y);

          for (size_t i=0; i < curve2D.size();i++)
          {
            
            curve.push_back(curve2D[i] + desp);
          }

          alignmentCurve = curve;

          break;
        }
        case 3: // PARABOLIC
        {
          IfcCurve<2> curve;

          glm::dvec2 StartPoint(StartDistAlong, StartHeight);

          double R = HorizontalLength / (EndGradient - StartGradient);

          std::vector<glm::dvec2> points;

          for (double i = 0; i <= _circleSegments; i++)
          {
            double pr = i / _circleSegments;
            double grad = ((HorizontalLength * pr) / R) + StartGradient;
            double alt = (HorizontalLength * pr * (grad + StartGradient) * 0.5) + StartHeight;
            points.push_back(glm::dvec2(HorizontalLength * pr, alt));
          }

          glm::dvec2 desp = glm::dvec2(StartPoint.x - points[0].x, StartPoint.y - points[0].y);

          for (auto &pt2D : points)
          {
            curve.push_back(pt2D + desp);
          }

          alignmentCurve = curve;

          break;
        }
        }
        break;
      }
      }

      return alignmentCurve;
    }

  std::optional<glm::dvec4> IfcGeometryLoader::GetColor(uint32_t expressID) const
    {
      auto lineID = _loader.ExpressIDToLineID(expressID);
      auto &line = _loader.GetLine(lineID);
      switch (line.ifcType)
      {
      case schema::IFCPRESENTATIONSTYLEASSIGNMENT:
      {
        _loader.MoveToArgumentOffset(line, 0);
        auto ifcPresentationStyleSelects = _loader.GetSetArgument();

        for (auto &styleSelect : ifcPresentationStyleSelects)
        {
          uint32_t styleSelectID = _loader.GetRefArgument(styleSelect);
          auto foundColor = GetColor(styleSelectID);
          if (foundColor) return foundColor;
        }

        return {};
      }
      case schema::IFCDRAUGHTINGPREDEFINEDCOLOUR:
      {
        _loader.MoveToArgumentOffset(line, 0);
         std::string color = _loader.GetStringArgument();
         if (color == "black") return glm::dvec4(0.0,0.0,0.0,1.0);
         else if (color == "red") return glm::dvec4(1.0,0,0,1.0);
         else if (color == "green") return glm::dvec4(0,1.0,0,1.0);
         else if (color == "blue") return glm::dvec4(0,0,1.0,1.0);
         else if (color == "yellow") return glm::dvec4(1.0,1.0,0,1.0);
         else if (color == "magenta") return glm::dvec4(1.0,0,1.0,1.0);
         else if (color == "cyan")  return glm::dvec4(0,1.0,1.0,1.0);
         else if (color == "white") return glm::dvec4(1.0,1.0,1.0,1.0);
         return {};
      }
      case schema::IFCCURVESTYLE:
      {
         _loader.MoveToArgumentOffset(line, 3);
         auto foundColor = GetColor(_loader.GetRefArgument());
         if (foundColor) return foundColor;
         return {};
      }
      case schema::IFCFILLAREASTYLEHATCHING:
      {
          //we cannot properly support this but for now use its colour as solid
         _loader.MoveToArgumentOffset(line, 0);
         auto foundColor = GetColor(_loader.GetRefArgument());
         if (foundColor) return foundColor;
         return {};
      }
      case schema::IFCSURFACESTYLE:
      {
        _loader.MoveToArgumentOffset(line, 2);
        auto ifcSurfaceStyleElementSelects = _loader.GetSetArgument();

        for (auto &styleElementSelect : ifcSurfaceStyleElementSelects)
        {
          uint32_t styleElementSelectID = _loader.GetRefArgument(styleElementSelect);
          auto foundColor = GetColor(styleElementSelectID);
          if (foundColor) return foundColor;
        }

        return {};
      }
      case schema::IFCSURFACESTYLERENDERING:
      {
        _loader.MoveToArgumentOffset(line, 0);
        auto outputColor = GetColor(_loader.GetRefArgument());
        _loader.MoveToArgumentOffset(line, 1);

        if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
        {
          _loader.StepBack();
          outputColor.value().a = 1 - _loader.GetDoubleArgument();
        }

        return outputColor;
      }
      case schema::IFCSURFACESTYLESHADING:
      {
        _loader.MoveToArgumentOffset(line, 0);
        return GetColor(_loader.GetRefArgument());
      }
      case schema::IFCSTYLEDREPRESENTATION:
      {
        _loader.MoveToArgumentOffset(line, 3);
        auto repItems = _loader.GetSetArgument();

        for (auto &repItem : repItems)
        {
          uint32_t repItemID = _loader.GetRefArgument(repItem);
          auto foundColor = GetColor(repItemID);
          if (foundColor) return foundColor;
        }

        return {};
      }
      case schema::IFCSTYLEDITEM:
      {
        _loader.MoveToArgumentOffset(line, 1);
        auto styledItems = _loader.GetSetArgument();

        for (auto &styledItem : styledItems)
        {
          uint32_t styledItemID = _loader.GetRefArgument(styledItem);
          auto foundColor = GetColor(styledItemID);
          if (foundColor)  return foundColor;
        }

        return {};
      }
      case schema::IFCCOLOURRGB:
      {
        _loader.MoveToArgumentOffset(line, 1);
        glm::dvec4 outputColor;
        outputColor.r = _loader.GetDoubleArgument();
        outputColor.g = _loader.GetDoubleArgument();
        outputColor.b = _loader.GetDoubleArgument();
        outputColor.a = 1;

        return outputColor;
      }
      case schema::IFCMATERIALLAYERSETUSAGE:
      {
        _loader.MoveToArgumentOffset(line, 0);
        uint32_t layerSetID = _loader.GetRefArgument();
        return GetColor(layerSetID);
      }
      case schema::IFCMATERIALLAYERSET:
      {
        _loader.MoveToArgumentOffset(line, 0);
        auto layers = _loader.GetSetArgument();

        for (auto &layer : layers)
        {
          uint32_t layerID = _loader.GetRefArgument(layer);
          auto foundColor = GetColor(layerID);
          if (foundColor)  return foundColor;
        }

        return {};
      }
      case schema::IFCMATERIALLAYER:
      {
        _loader.MoveToArgumentOffset(line, 0);
        uint32_t matRepID = _loader.GetRefArgument();
        return GetColor(matRepID);
      }
      case schema::IFCMATERIAL:
      {
        if (GetMaterialDefinitions().count(line.expressID) != 0)
        {
          auto &defs = GetMaterialDefinitions().at(line.expressID);
          for (auto def : defs)
          {
            auto success = GetColor(def.second);
            if (success) return success;
          }

          return {};
        }
        return {};
      }
      case schema::IFCFILLAREASTYLE:
      {
        _loader.MoveToArgumentOffset(line, 1);
        auto ifcFillStyleSelects = _loader.GetSetArgument();

        for (auto &styleSelect : ifcFillStyleSelects)
        {
          uint32_t styleSelectID = _loader.GetRefArgument(styleSelect);
          auto foundColor = GetColor(styleSelectID);
          if (foundColor) return foundColor;
        }

        return {};
      }
      case schema::IFCMATERIALLIST:
      {
        _loader.MoveToArgumentOffset(line, 0);
        auto materials = _loader.GetSetArgument();

        for (auto &material : materials)
        {
          uint32_t materialID = _loader.GetRefArgument(material);
          auto foundColor = GetColor(materialID);
          if (foundColor) return foundColor;
        }
        return {};
      }
      case schema::IFCMATERIALCONSTITUENTSET:
      {
        _loader.MoveToArgumentOffset(line, 2);
        auto materialContituents = _loader.GetSetArgument();

        for (auto &materialContituent : materialContituents)
        {
          uint32_t materialContituentID = _loader.GetRefArgument(materialContituent);
          auto foundColor = GetColor(materialContituentID);
          if (foundColor) return foundColor;
        }
        return {};
      }
      case schema::IFCMATERIALCONSTITUENT:
      {
        _loader.MoveToArgumentOffset(line, 2);
        auto material = _loader.GetRefArgument();
        auto foundColor = GetColor(material);
        if (foundColor) return foundColor;
        return {};
      }
      case schema::IFCMATERIALPROFILESETUSAGE:
      {
        _loader.MoveToArgumentOffset(line, 0);
        auto profileSet = _loader.GetRefArgument();
        auto foundColor = GetColor(profileSet);
        if (foundColor) return foundColor;
        return {};
      }
      case schema::IFCMATERIALPROFILE:
      {
        _loader.MoveToArgumentOffset(line, 2);
        auto profileSet = _loader.GetRefArgument();
        auto foundColor = GetColor(profileSet);
        if (foundColor) return foundColor;
        return {};
      }
      case schema::IFCMATERIALPROFILESET:
      {
        _loader.MoveToArgumentOffset(line, 2);
        auto materialProfiles = _loader.GetSetArgument();

        for (auto &materialProfile : materialProfiles)
        {
          uint32_t materialProfileID = _loader.GetRefArgument(materialProfile);
          auto foundColor = GetColor(materialProfileID);
          if (foundColor) return foundColor;
        }
        return {};
      }
      default:
        _errorHandler.ReportError(utility::LoaderErrorType::UNSUPPORTED_TYPE, "unexpected style type", line.expressID, line.ifcType);
        break;
      }

      return {};
    }

    template<size_t N> 
    IfcBound<N>  IfcGeometryLoader::GetBound(const uint32_t expressID) const
    {
      auto lineID = _loader.ExpressIDToLineID(expressID);
      auto &line = _loader.GetLine(lineID);

      switch (line.ifcType)
      {
      case schema::IFCFACEOUTERBOUND:
      {
        _loader.MoveToArgumentOffset(line, 0);
        uint32_t loop = _loader.GetRefArgument();
        _loader.MoveToArgumentOffset(line, 1);
        std::string orientValue = _loader.GetStringArgument();
        bool orient = orientValue == "T";

        IfcBound<N> bound;
        bound.curve = GetLoop<N>(loop);
        bound.orientation = orient;
        bound.type = IfcBoundType::OUTERBOUND;

        if (!orient)
        {
          std::reverse(bound.curve.begin(), bound.curve.end());
        }

        return bound;
      }
      case schema::IFCFACEBOUND:
      {
        _loader.MoveToArgumentOffset(line, 0);
        uint32_t loop = _loader.GetRefArgument();
        _loader.MoveToArgumentOffset(line, 1);
        std::string orientValue = _loader.GetStringArgument();
        bool orient = orientValue == "T";

        IfcBound<N> bound;
        bound.curve = GetLoop<N>(loop);
        bound.orientation = orient;
        bound.type = IfcBoundType::BOUND;

        if (!orient)
        {
          std::reverse(bound.curve.begin(), bound.curve.end());
        }

        return bound;
      }
      default:
        _errorHandler.ReportError(utility::LoaderErrorType::UNSUPPORTED_TYPE, "unexpected bound type", line.expressID, line.ifcType);
        break;
      }

      return {};
    }

    template <size_t N>
    IfcLoop<N> IfcGeometryLoader::GetLoop(uint32_t expressID) const
    {
      auto lineID = _loader.ExpressIDToLineID(expressID);
      auto &line = _loader.GetLine(lineID);

      switch (line.ifcType)
      {
      case schema::IFCPOLYLOOP:
      {
        IfcCurve<N> curve;

        _loader.MoveToArgumentOffset(line, 0);
        auto points = _loader.GetSetArgument();

        curve.reserve(points.size());

        uint32_t prevID = 0;
        for (auto &token : points)
        {
          uint32_t pointId = _loader.GetRefArgument(token);

          // trim out consecutive equal points
          if (pointId != prevID)
          {
            curve.push_back(GetCartesianPoint<N>(pointId));
          }

          prevID = pointId;
        }

        return curve;
      }
      case schema::IFCEDGELOOP:
      {
        std::vector<IfcCurve<N>> curves;

        _loader.MoveToArgumentOffset(line, 0);
        auto edges = _loader.GetSetArgument();
        int id = 0;

        for (auto &token : edges)
        {
          uint32_t edgeId = _loader.GetRefArgument(token);
          IfcCurve<N> edgeCurve = GetOrientedEdge<N>(edgeId);

          // Important not to repeat the last point otherwise triangulation fails
          // if the list has zero points this is initial, no repetition is possible, otherwise we must check
          for (auto &curve : curves)
          {
            for (auto &pt : curve)
            {
                auto found = std::find(edgeCurve.begin(), edgeCurve.end(), pt);
                if (found != edgeCurve.end()) edgeCurve.erase(found);
            }
          }
          curves.push_back(edgeCurve);
        }
        return curves;
      }
      default:
        _errorHandler.ReportError(utility::LoaderErrorType::UNSUPPORTED_TYPE, "unexpected loop type", line.expressID, line.ifcType);
        break;
      }
      return {};
    }

    template <size_t N>
    IfcCurve<N> IfcGeometryLoader::GetOrientedEdge(uint32_t expressID) const
    {
      auto lineID = _loader.ExpressIDToLineID(expressID);
      auto &line = _loader.GetLine(lineID);

      _loader.MoveToArgumentOffset(line, 3);
      std::string orientValue = _loader.GetStringArgument();
      bool orient = orientValue == "T";
      _loader.MoveToArgumentOffset(line, 2);
      uint32_t edgeCurveRef = _loader.GetRefArgument();
      IfcCurve<N> curveEdge = GetEdge<N>(edgeCurveRef);

      // Read edgeCurve

      if (orient)
      {
        std::reverse(curveEdge.begin(), curveEdge.end());
      }

      return curveEdge;
    }

    glm::dvec3 IfcGeometryLoader::GetVertexPoint(uint32_t expressID) const 
    {
        auto &vertex = _loader.GetLine(_loader.ExpressIDToLineID(expressID));
        _loader.MoveToArgumentOffset(vertex, 0);
        uint32_t pointRef = _loader.GetRefArgument();
        auto &point = _loader.GetLine(_loader.ExpressIDToLineID(pointRef));
        if (point.ifcType == schema::IFCCARTESIANPOINT)
        {
          return GetCartesianPoint<3>(pointRef);
        }
        else
        {
          _errorHandler.ReportError(utility::LoaderErrorType::UNSUPPORTED_TYPE, "unexpected vertxpoint type", point.expressID, point.ifcType);
        }
        
    }

    template <size_t N>
    IfcCurve<N> IfcGeometryLoader::GetEdge(uint32_t expressID) const
    {
      auto edgeID = _loader.ExpressIDToLineID(expressID);
      auto &line = _loader.GetLine(edgeID);

      switch (line.ifcType)
      {
      case schema::IFCEDGECURVE:
      {

        _loader.MoveToArgumentOffset(line, 0);
        IfcTrimmingSelect<3> startTrim;
        startTrim = GetVertexPoint(_loader.GetRefArgument());
        _loader.MoveToArgumentOffset(line, 1);
        IfcTrimmingSelect<3> endTrim;
        endTrim =  GetVertexPoint(_loader.GetRefArgument());

        _loader.MoveToArgumentOffset(line, 2);
        uint32_t CurveRef = _loader.GetRefArgument();

        return GetCurve(CurveRef, {}, 3, true, -1, -1, startTrim, endTrim);

      }
      default:
        _errorHandler.ReportError(utility::LoaderErrorType::UNSUPPORTED_TYPE, "unexpected edgecurve type", line.expressID, line.ifcType);
        break;
      }
      return {};
    }

  template <size_t N>
  IfcTrimmingSelect<N> IfcGeometryLoader::GetTrimSelect(std::vector<uint32_t> &tapeOffsets) const
  {
    for (size_t i = 0; i < tapeOffsets.size(); i++)
    {
      auto tokenType = _loader.GetTokenType(tapeOffsets[i]);

      _loader.StepBack();
      if (tokenType == parsing::IfcTokenType::REF)
      {
        // caresian point
        uint32_t cartesianPointRef = _loader.GetRefArgument();
        IfcTrimmingSelect<N> trim = GetCartesianPoint<N>(cartesianPointRef);
        return trim;
      }
      else if (tokenType == parsing::IfcTokenType::LABEL)
      {
        // parametervalue
        std::string type = _loader.GetStringArgument();

        if (type == "IFCPARAMETERVALUE")
        {
          IfcTrimmingSelect<N> trim = _loader.GetDoubleArgument(tapeOffsets[i]);
          return trim;
        }
      }
    }
    return {};
  }

  template<size_t N>
  glm::vec<N,double> IfcGeometryLoader::GetCartesianPoint(const uint32_t expressID) const
  {
    uint32_t lineID = _loader.ExpressIDToLineID(expressID);
    auto &line = _loader.GetLine(lineID);
    _loader.MoveToArgumentOffset(line, 0);
    _loader.GetTokenType();
    // because these calls cannot be reordered we have to use intermediate variables
    double x = _loader.GetDoubleArgument();
    double y = _loader.GetDoubleArgument();
    if (N == 3) 
    {
        double z = _loader.GetDoubleArgument();
        return glm::vec<3,double>(x,y,z);
    } else
    {
      return glm::vec<2,double>(x,y);
    }
  }

  
  template<size_t N>
  std::vector<glm::vec<N,double>> IfcGeometryLoader::GetIfcCartesianPointList(uint32_t expressID) const
  {
    auto lineID = _loader.ExpressIDToLineID(expressID);
    auto &line = _loader.GetLine(lineID);

    _loader.MoveToArgumentOffset(line, 0);

    std::vector<glm::vec<N,double>> result;

    _loader.GetTokenType();

    // while we have point set begin
    while (_loader.GetTokenType() == parsing::IfcTokenType::SET_BEGIN)
    {
      // because these calls cannot be reordered we have to use intermediate variables
      double x = _loader.GetDoubleArgument();
      double y = _loader.GetDoubleArgument();
      if (N==3)
      {
        double z = _loader.GetDoubleArgument();
        result.emplace_back(x, y, z);
      } else {
        result.emplace_back(x,y);
      }

      // read point set end
      _loader.GetTokenType();
    }

    return result;
  }

  template<size_t N> 
  IfcCurve<N>& IfcGeometryLoader::GetCurve(uint32_t expressID, IfcCurve<N> curve, bool edge, int sameSense, int trimSense, std::optional<IfcTrimmingSelect<N>> startTrim, std::optional<IfcTrimmingSelect<N>> endTrim) const
  {
    uint32_t lineID = _loader.ExpressIDToLineID(expressID);
    auto &line = _loader.GetLine(lineID);
    switch (line.ifcType)
    {
    case schema::IFCPOLYLINE:
    {
      _loader.MoveToArgumentOffset(line, 0);
      auto points = _loader.GetSetArgument();

      for (auto &token : points)
      {
        uint32_t pointId = _loader.GetRefArgument(token);
        curve.Add(GetCartesianPoint<N>(pointId));    
      }

      if (edge)
      {
        if (sameSense == 1 || sameSense == -1)  std::reverse(curve.begin(), curve.end());
      }

      return curve;
    }
    case schema::IFCCOMPOSITECURVE:
    {
      _loader.MoveToArgumentOffset(line, 0);
      auto segments = _loader.GetSetArgument();
      auto selfIntersects = _loader.GetStringArgument();

      if (selfIntersects == "T")
      {
        // TODO: this is probably bad news
        _errorHandler.ReportError(utility::LoaderErrorType::UNSPECIFIED, "Self intersecting composite curve", line.expressID);
      }

      for (auto &token : segments)
      {
        #ifdef DEBUG_DUMP_SVG
            io::DumpSVGCurve(curve, "partial_curve.html");
        #endif

        uint32_t segmentId = _loader.GetRefArgument(token);

        curve = GetCurve(segmentId, curve, edge, sameSense, trimSense);
      }

      return curve;
    }
    case schema::IFCCOMPOSITECURVESEGMENT:
    {
      _loader.MoveToArgumentOffset(line, 0);
      auto transition = _loader.GetStringArgument();
      auto sameSenseS = _loader.GetStringArgument();
      auto parentID = _loader.GetRefArgument();

      bool sameSense = sameSenseS == "T";

      curve = GetCurve(parentID, curve, edge, sameSense, trimSense);

      return curve;
    }
    case schema::IFCLINE:
    {
      bool condition = sameSense == 1 || sameSense == -1;
      if (edge)
      {
        condition = !condition;
      }
        
      if (startTrim && startTrim.value().GetType() == typeid(glm::vec<3,double>) && endTrim && endTrim.value().GetType() == typeid(glm::vec<3,double>))
      {

        if (condition)
        {
          curve.push_back(startTrim.value());
          curve.push_back(endTrim.value());
        }
        else
        {
          curve.push_back(endTrim.value());
          curve.push_back(startTrim.value());
        }
      }
      else if (startTrim && startTrim.value().GetType() == typeid(double) && endTrim && endTrim.value().GetType() == typeid(double))
      {
        _loader.MoveToArgumentOffset(line, 0);
        auto positionID = _loader.GetRefArgument();
        auto vectorID = _loader.GetRefArgument();
        glm::dvec3 placement = glm::dvec3(GetCartesianPoint<N>(positionID), 0);
        glm::dvec3 vector;
        vector = GetVector(vectorID);

        if (condition)
        {
          glm::dvec3 p1 = placement + vector * startTrim;
          glm::dvec3 p2 = placement + vector * endTrim;
          curve.push_back(p1);
          curve.push_back(p2);
        }
        else
        {
          glm::dvec3 p2 = placement + vector * startTrim;
          glm::dvec3 p1 = placement + vector * endTrim;
          curve.push_back(p1);
          curve.push_back(p2);
        }
      }
      else
      {
        _errorHandler.ReportError(utility::LoaderErrorType::UNSUPPORTED_TYPE, "Unsupported trimmingselect IFCLINE", line.expressID, line.ifcType);
      }
   
      return curve;
    }
    case schema::IFCTRIMMEDCURVE:
    {
      _loader.MoveToArgumentOffset(line, 0);
      auto basisCurveID = _loader.GetRefArgument();
      auto trim1Set = _loader.GetSetArgument();
      auto trim2Set = _loader.GetSetArgument();

      auto senseAgreementS = _loader.GetStringArgument();
      auto trimmingPreference = _loader.GetStringArgument();

      auto trim1 = GetTrimSelect<N>(trim1Set);
      auto trim2 = GetTrimSelect<N>(trim2Set);

      bool senseAgreement = senseAgreementS == "T";

      if (trimSense == 0)
      {
        senseAgreement = !senseAgreement;
      }

      return GetCurve(basisCurveID, curve, edge, sameSense, senseAgreement, trim1,trim2);

    }
    case schema::IFCINDEXEDPOLYCURVE:
    {
      _loader.MoveToArgumentOffset(line, 0);
      auto pts2DRef = _loader.GetRefArgument();

      _loader.MoveToArgumentOffset(line, 2);

      if (_loader.GetTokenType() != parsing::IfcTokenType::EMPTY)
      {
        _loader.StepBack();
        auto selfIntersects = _loader.GetStringArgument();

        if (selfIntersects == "T")
        {
          // TODO: this is probably bad news
          _errorHandler.ReportError(utility::LoaderErrorType::UNSPECIFIED, "Self intersecting ifcindexedpolycurve", line.expressID);
        }
      }

      _loader.MoveToArgumentOffset(line, 1);
      if (_loader.GetTokenType() != parsing::IfcTokenType::EMPTY)
      {
        auto pnSegment = ReadCurveIndices();
        for (auto &sg : pnSegment)
        {
          if (sg.type == "IFCLINEINDEX")
          {
            auto pts = GetIfcCartesianPointList<N>(pts2DRef);
            for (auto &pt : sg.indexs)
            {
              curve.push_back(pts[pt - 1]);
            }
          }
          if (sg.type == "IFCARCINDEX")
          {
            auto pts = GetIfcCartesianPointList<N>(pts2DRef);
            IfcCurve<N> arc = BuildArc3Pt(pts[sg.indexs[0] - 1], pts[sg.indexs[1] - 1], pts[sg.indexs[2] - 1],_circleSegments);
            for (auto &pt : arc)
            {
              curve.push_back(pt);
            }
          }
        }
      }
      else
      {
        auto pts = GetIfcCartesianPointList<N>(pts2DRef);
        for (auto &pt : pts)
        {
          curve.push_back(pt);
        }
      }
    
      return curve;
    }
    case schema::IFCCIRCLE:
    {
      _loader.MoveToArgumentOffset(line, 0);
      auto positionID = _loader.GetRefArgument();
      double radius = _loader.GetDoubleArgument();

      double startDegrees = 0;
      double endDegrees = 360;

      if (startTrim && endTrim)
      {
        if (startTrim.GetType() == typeid(double) && endTrim.GetType() == typeid(double))
        {
          startDegrees = startTrim;
          endDegrees = endTrim;
        }
        else if (startTrim.GetType() == typeid(glm::vec<N,double>) && endTrim.GetType() == typeid(glm::vec<N,double>))
        {
          
          auto placement = GetPlacement<N>(positionID);
          if (N == 2)
          { 
            double xx = placement[2].x - startTrim.x;
            double yy = placement[2].y - startTrim.y;
            startDegrees = VectorToAngle(xx, yy);
            xx = placement[2].x - endTrim.x;
            yy = placement[2].y - endTrim.y;
            endDegrees = VectorToAngle(xx, yy);
          }
          else if (N == 3)
          {
            glm::dvec4 vecX = placement[0];
            glm::dvec4 vecY = placement[1];
            glm::dvec4 vecZ = placement[2];

            glm::dvec3 v1 = glm::dvec3(
              startTrim.x - placement[3].x,
              startTrim.y - placement[3].y,
              startTrim.z - placement[3].z);
            glm::dvec3 v2 = glm::dvec3(
              endTrim.x - placement[3].x,
              endTrim.y - placement[3].y,
              endTrim.z - placement[3].z);

            double dxS = vecX.x * v1.x + vecX.y * v1.y + vecX.z * v1.z;
            double dyS = vecY.x * v1.x + vecY.y * v1.y + vecY.z * v1.z;
            // double dzS = vecZ.x * v1.x + vecZ.y * v1.y + vecZ.z * v1.z;

            double dxE = vecX.x * v2.x + vecX.y * v2.y + vecX.z * v2.z;
            double dyE = vecY.x * v2.x + vecY.y * v2.y + vecY.z * v2.z;
            // double dzE = vecZ.x * v2.x + vecZ.y * v2.y + vecZ.z * v2.z;

            endDegrees = VectorToAngle(dxS, dyS) - 90;
            startDegrees = VectorToAngle(dxE, dyE) - 90;
          }
        }
      }

      double startRad = startDegrees / 180 * CONST_PI;
      double endRad = endDegrees / 180 * CONST_PI;
      double lengthDegrees = endDegrees - startDegrees;

      // unset or true
      if (trimSense == 1 || trimSense == -1)
      {
        if (lengthDegrees < 0)
        {
          lengthDegrees += 360;
        }
      }
      else
      {
        if (lengthDegrees > 0)
        {
          lengthDegrees -= 360;
        }
      }

      double lengthRad = lengthDegrees / 180 * CONST_PI;

      size_t startIndex = curve.size();

      for (int i = 0; i < _circleSegments; i++)
      {
        double ratio = static_cast<double>(i) / (_circleSegments - 1);
        double angle = startRad + ratio * lengthRad;

        if (sameSense == 0)
        {
          angle = endRad - ratio * lengthRad;
        }

        glm::vec<N,double> vec(0);
        vec[0] = radius * std::cos(angle);
        vec[1] = -radius * std::sin(angle); // negative or not???
        glm::vec<N,double> pos = GetLocalPlacement(positionID) * glm::dvec4(glm::dvec3(vec), 1);
        curve.push_back(pos);
      }

      // without a trim, we close the circle
      if (startTrim || endTrim)
      {
        curve.Add(curve[startIndex]);
      }

      break;
    }
    case schema::IFCELLIPSE:
    {
      _loader.MoveToArgumentOffset(line, 0);
      auto positionID = _loader.GetRefArgument();
      double radius1 = _loader.GetDoubleArgument();
      double radius2 = _loader.GetDoubleArgument();

      double startDegrees = 0;
      double endDegrees = 360;

      if (startTrim && endTrim)
      {
        if (startTrim.GetType() == typeid(double) && endTrim.GetType() == typeid(double))
        {
          startDegrees = startTrim;
          endDegrees = endTrim;
        }
        else if (startTrim.GetType() == typeid(glm::vec<N,double>) && endTrim.GetType() == typeid(glm::vec<N,double>))
        {
         
          auto placement = GetPlacement<N>(positionID);
          if (N == 2)
          {
            double xx = placement[2].x - startTrim.x;
            double yy = placement[2].y - startTrim.y;
            startDegrees = VectorToAngle(xx, yy);
            xx = placement[2].x - endTrim.x;
            yy = placement[2].y - endTrim.y;
            endDegrees = VectorToAngle(xx, yy);
          }
          else if (N == 3)
          {
            glm::dvec4 vecX = placement[0];
            glm::dvec4 vecY = placement[1];
            glm::dvec4 vecZ = placement[2];

            glm::dvec3 v1 = glm::dvec3(
              startTrim.x - placement[3].x,
              startTrim.y - placement[3].y,
              startTrim.z - placement[3].z);
            glm::dvec3 v2 = glm::dvec3(
              endTrim.x - placement[3].x,
              endTrim.y - placement[3].y,
              endTrim.z - placement[3].z);

            double dxS = vecX.x * v1.x + vecX.y * v1.y + vecX.z * v1.z;
            double dyS = vecY.x * v1.x + vecY.y * v1.y + vecY.z * v1.z;
            // double dzS = vecZ.x * v1.x + vecZ.y * v1.y + vecZ.z * v1.z;

            double dxE = vecX.x * v2.x + vecX.y * v2.y + vecX.z * v2.z;
            double dyE = vecY.x * v2.x + vecY.y * v2.y + vecY.z * v2.z;
            // double dzE = vecZ.x * v2.x + vecZ.y * v2.y + vecZ.z * v2.z;

            endDegrees = VectorToAngle(dxS, dyS) - 90;
            startDegrees = VectorToAngle(dxE, dyE) - 90;
          }
        }
      }

      double startRad = startDegrees / 180 * CONST_PI;
      double endRad = endDegrees / 180 * CONST_PI;

      // TODO: Because this is an ellipse you need to correct the angles

      // startRad = atan((radius1 / radius2) * tan(startDegrees));
      // endRad = atan((radius1 / radius2) * tan(endDegrees));

      double lengthDegrees = endDegrees - startDegrees;

      // unset or true
      if (trimSense == 1 || trimSense == -1)
      {
        if (lengthDegrees < 0)
        {
          lengthDegrees += 360;
        }
      }
      else
      {
        if (lengthDegrees > 0)
        {
          lengthDegrees -= 360;
        }
      }

      double lengthRad = lengthDegrees / 180 * CONST_PI;

      size_t startIndex = curve.size();

      for (int i = 0; i < _circleSegments; i++)
      {
        double ratio = static_cast<double>(i) / (_circleSegments - 1);
        double angle = startRad + ratio * lengthRad;
        if (sameSense == 0)
        {
          angle = endRad - ratio * lengthRad;
        }

        if ( N == 2)
        {
          glm::dvec2 vec(0);
          vec[0] = radius1 * std::cos(angle);
          vec[1] = -radius2 * std::sin(angle);
          curve.push_back(GetPlacement<N>(positionID) * glm::dvec3(vec, 1));
        }
        else
        {
          glm::dvec3 vec(0);
          vec[0] = radius1 * std::cos(angle);
          vec[1] = -radius2 * std::sin(angle); // negative or not???
          glm::dvec3 pos = GetPlacement<N>(positionID) * glm::dvec4(glm::dvec3(vec), 1);
          curve.push_back(pos);
        }
      }

      // without a trim, we close the circle
      if (!startTrim || !endTrim)
      {
        curve.push_back(curve[startIndex]);
      }

      break;
    }
    case schema::IFCBSPLINECURVE:
      {
        bool condition = sameSense == 0;
        if (edge)
        {
          condition = !condition;
        }
        std::vector<glm::f64> knotMultiplicities;
        std::vector<glm::f64> distinctKnots;
        std::vector<glm::f64> knots;
        std::vector<glm::f64> indexes;
        std::vector<glm::f64> weights;

        _loader.MoveToArgumentOffset(line, 0);
        double degree = _loader.GetDoubleArgument();
        auto points = _loader.GetSetArgument();
        auto curveType = _loader.GetStringArgument();
        auto closed = _loader.GetStringArgument();
        auto selfIntersect = _loader.GetStringArgument();


        // build default knots
        for (int k = 0; k < points.size() + degree + 1; k++)
        {
          knots.push_back(k);
        }

        if (knots.size() != points.size() + degree + 1)
        {
          std::cout << "Error: Knots and control points do not match" << std::endl;
        }

        // build default weights vector
        for (size_t i = 0; i < points.size(); i++)
        {
          weights.push_back(1);
        }
        
        std::vector<glm::vec<N,double>> ctrolPts;
        for (auto &token : points)
        {
          uint32_t pointId = _loader.GetRefArgument(token);
          ctrolPts.push_back(GetCartesianPoint<N>(pointId));
        }
        
        std::vector<glm::vec<N,double>> tempPoints = GetRationalBSplineCurveWithKnots(degree, ctrolPts, knots, weights);
        for (size_t i = 0; i < tempPoints.size(); i++) curve.Add(tempPoints[i]);
               

        if (condition)
        {
          std::reverse(curve.begin(), curve.end());
        }

        break;
      }
    case schema::IFCBSPLINECURVEWITHKNOTS:
      {
        bool condition = sameSense == 0;
        if (edge)
        {
          condition = !condition;
        }

        // The IfcBSplineCurveWithKnots is a spline curve parameterized by spline functions for which the knot values are explicitly given.
        // This is the type of b-spline curve for which the knot values are explicitly given. This subtype shall be used to represent non-uniform B-spline curves and may be used for other knot types.
        // Let L denote the number of distinct values amongst the d+k+2 knots in the knot list; L will be referred to as the ‘upper index on knots’. Let mj denote the multiplicity (i.e., number of repetitions) of the _j_th distinct knot.

        // All knot multiplicities except the first and the last shall be in the range 1,...,d; the first and last may have a maximum value of d + 1. In evaluating the basis functions, a knot u of, e.g., multiplicity 3 is interpreted as a sequence u, u, u,; in the knot array.
        std::vector<glm::f64> knotMultiplicities;
        std::vector<glm::f64> distinctKnots;
        std::vector<glm::f64> knots;
        std::vector<glm::f64> indexes;
        std::vector<glm::f64> weights;

        _loader.MoveToArgumentOffset(line, 0);
        double degree = _loader.GetDoubleArgument();
        auto points = _loader.GetSetArgument();
        auto curveType = _loader.GetStringArgument();
        auto closed = _loader.GetStringArgument();
        auto selfIntersect = _loader.GetStringArgument();
        auto knotMultiplicitiesSet = _loader.GetSetArgument(); // The multiplicities of the knots. This list defines the number of times each knot in the knots list is to be repeated in constructing the knot array.
        auto knotSet = _loader.GetSetArgument();         // The list of distinct knots used to define the B-spline basis functions.



        for (auto &token : knotMultiplicitiesSet)
        {
          knotMultiplicities.push_back(_loader.GetDoubleArgument(token));
        }

        for (auto &token : knotSet)
        {
          distinctKnots.push_back(_loader.GetDoubleArgument(token));
        }

        for (size_t k = 0; k < distinctKnots.size(); k++)
        {
          double knot = distinctKnots[k];
          for (int i = 0; i < knotMultiplicities[k]; i++)
          {
            knots.push_back(knot);
          }
        }

        if (knots.size() != points.size() + degree + 1)
        {
          std::cout << "Error: Knots and control points do not match" << std::endl;
        }

        // build default weights vector
        for (size_t i = 0; i < points.size(); i++)
        {
          weights.push_back(1);
        }

        std::vector<glm::vec<N,double>> ctrolPts;
        for (auto &token : points)
        {
          uint32_t pointId = _loader.GetRefArgument(token);
          ctrolPts.push_back(GetCartesianPoint<N>(pointId));
        }
        std::vector<glm::vec<N,double>> tempPoints = GetRationalBSplineCurveWithKnots(degree, ctrolPts, knots, weights);
        for (size_t i = 0; i < tempPoints.size(); i++) curve.Add(tempPoints[i]);
        
    if (condition)
    {
      std::reverse(curve.begin(), curve.end());
    }

    break;
  }
case schema::IFCRATIONALBSPLINECURVEWITHKNOTS:
  {

    bool condition = sameSense == 0;
    if (edge)
    {
      condition = !condition;
    }

    std::vector<glm::f64> distinctKnots;
    std::vector<glm::u32> knotMultiplicities;
    std::vector<glm::f64> knots;
    std::vector<glm::f64> weights;
    _loader.MoveToArgumentOffset(line, 0);
    double degree = _loader.GetDoubleArgument();
    auto points = _loader.GetSetArgument();
    auto curveType = _loader.GetStringArgument();
    auto closed = _loader.GetStringArgument();
    auto selfIntersect = _loader.GetStringArgument();
    auto knotMultiplicitiesSet = _loader.GetSetArgument(); // The multiplicities of the knots. This list defines the number of times each knot in the knots list is to be repeated in constructing the knot array.
    auto knotSet = _loader.GetSetArgument();
    auto knotSpec = _loader.GetStringArgument(); // The description of the knot type. This is for information only.
    auto weightsSet = _loader.GetSetArgument();

    for (auto &token : knotMultiplicitiesSet)
    {
      knotMultiplicities.push_back(_loader.GetDoubleArgument(token));
    }

    for (auto &token : knotSet)
    {
      distinctKnots.push_back(_loader.GetDoubleArgument(token));
    }

    for (auto &token : weightsSet)
    {
      weights.push_back(_loader.GetDoubleArgument(token));
    }

    for (size_t k = 0; k < distinctKnots.size(); k++)
    {
      double knot = distinctKnots[k];
      for (size_t i = 0; i < knotMultiplicities[k]; i++)
      {
        knots.push_back(knot);
      }
    }

    if (knots.size() != points.size() + degree + 1)
    {
      std::cout << "Error: Knots and control points do not match" << std::endl;
    }

    std::vector<glm::vec<N,double>> ctrolPts;
    for (auto &token : points)
    {
      uint32_t pointId = _loader.GetRefArgument(token);
      ctrolPts.push_back(GetCartesianPoint<N>(pointId));
    }

    std::vector<glm::dvec3> tempPoints = GetRationalBSplineCurveWithKnots(degree, ctrolPts, knots, weights);
    for (size_t i = 0; i < tempPoints.size(); i++) curve.Add(tempPoints[i]);
        
    if (condition)
    {
      std::reverse(curve.begin(), curve.end());
    }

    break;
  }
default:
  _errorHandler.ReportError(utility::LoaderErrorType::UNSUPPORTED_TYPE, "Unsupported curve type", line.expressID, line.ifcType);
  break;
}
  
  #ifdef DEBUG_DUMP_SVG
      io::DumpSVGCurve(curve, "partial_curve.html");
  #endif

}


IfcProfile IfcGeometryLoader::GetProfile(uint32_t expressID) const
{
  auto profile = GetProfileByLine(_loader.ExpressIDToLineID(expressID));

  if (!profile.isComposite)
  {
    if (!profile.curve.IsCCW())
    {
      profile.curve.Invert();
    }
    for (auto &hole : profile.holes)
    {
      if (hole.IsCCW())
      {
        hole.Invert();
      }
    }
  }
  else
  {
    for (uint32_t i = 0; i < profile.profiles.size(); i++)
    {
      if (!profile.profiles[i].curve.IsCCW())
      {
        profile.profiles[i].curve.Invert();
      }
      for (auto &hole : profile.profiles[i].holes)
      {
        if (hole.IsCCW())
        {
          hole.Invert();
        }
      }
    }
  }

  return profile;
}

IfcProfile IfcGeometryLoader::GetProfileByLine(uint32_t lineID) const
{
  auto &line = _loader.GetLine(lineID);
  switch (line.ifcType)
  {
  case schema::IFCARBITRARYOPENPROFILEDEF:
  case schema::IFCARBITRARYCLOSEDPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      _loader.MoveToArgumentOffset(line, 2);
      profile.curve = GetCurve(_loader.GetRefArgument(),2);
      profile.isConvex = IsCurveConvex(profile.curve);

      return profile;
    }
  case schema::IFCARBITRARYPROFILEDEFWITHVOIDS:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      _loader.MoveToArgumentOffset(line, 2);
      profile.curve = GetCurve(_loader.GetRefArgument(),2);
      profile.isConvex = IsCurveConvex(profile.curve);

      _loader.MoveToArgumentOffset(line, 3);
      auto holes = _loader.GetSetArgument();

      for (auto &hole : holes)
      {
        IfcCurve holeCurve = GetCurve(_loader.GetRefArgument(hole),2);
        profile.holes.push_back(holeCurve);
      }

      return profile;
    }
  case schema::IFCRECTANGLEPROFILEDEF:
  case schema::IFCROUNDEDRECTANGLEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(line, 2);
      uint32_t placementID = _loader.GetRefArgument();
      double xdim = _loader.GetDoubleArgument();
      double ydim = _loader.GetDoubleArgument();

      if (placementID != 0)
      {
        glm::dmat3 placement = GetAxis2Placement2D(placementID);
        profile.curve = GetRectangleCurve(xdim, ydim, placement);
      }
      else
      {
        glm::dmat3 placement = glm::dmat3(
          glm::dvec3(1, 0, 0),
          glm::dvec3(0, 1, 0),
          glm::dvec3(0, 0, 1));
        profile.curve = GetRectangleCurve(xdim, ydim, placement);
      }
      return profile;
    }
  case schema::IFCRECTANGLEHOLLOWPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(line, 2);
      uint32_t placementID = _loader.GetRefArgument();
      double xdim = _loader.GetDoubleArgument();
      double ydim = _loader.GetDoubleArgument();
      double thickness = _loader.GetDoubleArgument();

        // fillets not implemented yet

      glm::dmat3 placement = GetAxis2Placement2D(placementID);

      profile.curve = GetRectangleCurve(xdim, ydim, placement);
      profile.holes.push_back(GetRectangleCurve(xdim - thickness, ydim - thickness, placement));
      std::reverse(profile.holes[0].begin(), profile.holes[0].end());

      return profile;
    }
  case schema::IFCCIRCLEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(line, 2);
      uint32_t placementID = _loader.GetOptionalRefArgument();
      double radius = _loader.GetDoubleArgument();

      glm::dmat3 placement(1);

      if (placementID)
      {
        placement = GetAxis2Placement2D(placementID);
      }

      profile.curve = GetCircleCurve(radius, _circleSegments, placement);

      return profile;
    }
  case schema::IFCELLIPSEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(line, 2);
      uint32_t placementID = _loader.GetRefArgument();
      double radiusX = _loader.GetDoubleArgument();
      double radiusY = _loader.GetDoubleArgument();

      glm::dmat3 placement = GetAxis2Placement2D(placementID);

      profile.curve = GetEllipseCurve(radiusX, radiusY, _circleSegments, placement);

      return profile;
    }
  case schema::IFCCIRCLEHOLLOWPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(line, 2);
      uint32_t placementID = _loader.GetRefArgument();
      double radius = _loader.GetDoubleArgument();
      double thickness = _loader.GetDoubleArgument();

      glm::dmat3 placement = GetAxis2Placement2D(placementID);

      profile.curve = GetCircleCurve(radius, _circleSegments, placement);
      profile.holes.push_back(GetCircleCurve(radius - thickness, _circleSegments, placement));
      std::reverse(profile.holes[0].begin(), profile.holes[0].end());

      return profile;
    }
  case schema::IFCISHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(line, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        placement = GetAxis2Placement2D(placementID);
      }

      _loader.MoveToArgumentOffset(line, 3);

      double width = _loader.GetDoubleArgument();
      double depth = _loader.GetDoubleArgument();
      double webThickness = _loader.GetDoubleArgument();
      double flangeThickness = _loader.GetDoubleArgument();

        // optional fillet
      bool hasFillet = false;
      double filletRadius = 0;
      if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
      {
        _loader.StepBack();

        hasFillet = true;
        filletRadius = _loader.GetDoubleArgument();
      }

      profile.curve = GetIShapedCurve(width, depth, webThickness, flangeThickness, hasFillet, filletRadius, placement);

      return profile;
    }
  case schema::IFCLSHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = false;

      _loader.MoveToArgumentOffset(line, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        placement = GetAxis2Placement2D(placementID);
      }

      _loader.MoveToArgumentOffset(line, 3);
      double filletRadius = 0;
      double depth = _loader.GetDoubleArgument();
      double width = _loader.GetDoubleArgument();
      double thickness = _loader.GetDoubleArgument();
      filletRadius = _loader.GetDoubleArgument();
      double edgeRadius = _loader.GetDoubleArgument();
      double legSlope = _loader.GetDoubleArgument();
        // double centreOfGravityInX =
      _loader.GetDoubleArgument();
        // double centreOfGravityInY =
      _loader.GetDoubleArgument();

        // optional fillet
      bool hasFillet = false;

      if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
      {
        _loader.StepBack();

        hasFillet = true;
        filletRadius = _loader.GetDoubleArgument();
      }

      profile.curve = GetLShapedCurve(width, depth, thickness, hasFillet, filletRadius, edgeRadius, legSlope, placement);

      return profile;
    }
  case schema::IFCTSHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = false;

      _loader.MoveToArgumentOffset(line, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        placement = GetAxis2Placement2D(placementID);
      }

      _loader.MoveToArgumentOffset(line, 3);
      double depth = _loader.GetDoubleArgument();
      double width = _loader.GetDoubleArgument();
      double webThickness = _loader.GetDoubleArgument();
        // double flangeThickness =
      _loader.GetDoubleArgument();
      double filletRadius = _loader.GetDoubleArgument();
      double flangeEdgeRadius = _loader.GetDoubleArgument();
        // double webEdgeRadius =
      _loader.GetDoubleArgument();
        // double webSlope =
      _loader.GetDoubleArgument();
      double flangeSlope = _loader.GetDoubleArgument();

        // optional fillet
      bool hasFillet = false;

      if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
      {
        _loader.StepBack();

        hasFillet = true;
        filletRadius = _loader.GetDoubleArgument();
      }

      profile.curve = GetTShapedCurve(width, depth, webThickness, hasFillet, filletRadius, flangeEdgeRadius, flangeSlope, placement);

      return profile;
    }
  case schema::IFCUSHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(line, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        placement = GetAxis2Placement2D(placementID);
      }

      _loader.MoveToArgumentOffset(line, 3);

      double depth = _loader.GetDoubleArgument();
      double flangeWidth = _loader.GetDoubleArgument();
      double webThickness = _loader.GetDoubleArgument();
      double flangeThickness = _loader.GetDoubleArgument();

        // optional parameters
        // double filletRadius = GetOptionalDoubleParam();
        // double edgeRadius = GetOptionalDoubleParam();
        // double flangeSlope = GetOptionalDoubleParam();
      double filletRadius = 0;
      double edgeRadius = 0;
      double flangeSlope = 0;

      profile.curve = GetUShapedCurve(depth, flangeWidth, webThickness, flangeThickness, filletRadius, edgeRadius, flangeSlope, placement);

      return profile;
    }
  case schema::IFCCSHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(line, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        placement = GetAxis2Placement2D(placementID);
      }

      bool hasFillet = false;

      _loader.MoveToArgumentOffset(line, 3);

      double depth = _loader.GetDoubleArgument();
      double Width = _loader.GetDoubleArgument();
      double Thickness = _loader.GetDoubleArgument();
      double girth = _loader.GetDoubleArgument();
      double filletRadius = _loader.GetDoubleArgument();

      profile.curve = GetCShapedCurve(Width, depth, girth, Thickness, hasFillet, filletRadius, placement);

      return profile;
    }
  case schema::IFCZSHAPEPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      profile.isConvex = true;

      _loader.MoveToArgumentOffset(line, 2);

      glm::dmat3 placement(1);

      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();

        uint32_t placementID = _loader.GetRefArgument();
        glm::dmat3 placement = GetAxis2Placement2D(placementID);
      }

      bool hasFillet = false;

      _loader.MoveToArgumentOffset(line, 3);

      double depth = _loader.GetDoubleArgument();
      double flangeWidth = _loader.GetDoubleArgument();
      double webThickness = _loader.GetDoubleArgument();
      double flangeThickness = _loader.GetDoubleArgument();
      double filletRadius = _loader.GetDoubleArgument();
      double edgeRadius = _loader.GetDoubleArgument();

      profile.curve = GetZShapedCurve(depth, flangeWidth, webThickness, flangeThickness, filletRadius, edgeRadius, placement);

      return profile;
    }
  case schema::IFCDERIVEDPROFILEDEF:
    {
      _loader.MoveToArgumentOffset(line, 2);
      uint32_t profileID = _loader.GetRefArgument();
      IfcProfile profile = GetProfileByLine(_loader.ExpressIDToLineID(profileID));

      _loader.MoveToArgumentOffset(line, 3);
      uint32_t transformID = _loader.GetRefArgument();
      glm::dmat3 transformation = GetAxis2Placement2D(transformID);

      if (!profile.isComposite)
      {
        for (uint32_t i = 0; i < profile.curve.size(); i++)
        {
          profile.curve[i] = transformation * glm::dvec3(profile.curve[i].x,profile.curve[i].y, 1);
        }
      }
      else
      {
        for (uint32_t j = 0; j < profile.profiles.size(); j++)
        {
          for (uint32_t i = 0; i < profile.profiles[j].curve.size(); i++)
          {
            profile.profiles[j].curve[i] = transformation * glm::dvec3(profile.profiles[j].curve[i].x,profile.profiles[j].curve[i].y, 1);
          }
        }
      }

      return profile;
    }
  case schema::IFCCOMPOSITEPROFILEDEF:
    {
      IfcProfile profile = IfcProfile();

      std::vector<uint32_t> lst;

      _loader.MoveToArgumentOffset(line, 2);
      parsing::IfcTokenType t = _loader.GetTokenType();
      if (t == parsing::IfcTokenType::SET_BEGIN)
      {
        while (_loader.GetTokenType() == parsing::IfcTokenType::REF)
        {
          _loader.StepBack();
          uint32_t profileID = _loader.ExpressIDToLineID(_loader.GetRefArgument());
          lst.push_back(profileID);
        }
      }

      profile.isComposite = true;

      for (uint32_t i = 0; i < lst.size(); i++)
      {
        IfcProfile profile_t = GetProfileByLine(lst[i]);
        profile.profiles.push_back(profile_t);
      }

      return profile;
    }
  default:
    _errorHandler.ReportError(utility::LoaderErrorType::UNSUPPORTED_TYPE, "unexpected profile type", line.expressID, line.ifcType);
    break;
  }

  return IfcProfile();
}

IfcProfile IfcGeometryLoader::GetProfile3D(uint32_t lineID) const
{
  auto &line = _loader.GetLine(lineID);
  switch (line.ifcType)
  {
  case schema::IFCARBITRARYOPENPROFILEDEF:
    {
      IfcProfile profile;

      _loader.MoveToArgumentOffset(line, 0);
      profile.type = _loader.GetStringArgument();
      _loader.MoveToArgumentOffset(line, 2);
      profile.curve = GetCurve(_loader.GetRefArgument(),3);

      return profile;
    }
  default:
    _errorHandler.ReportError(utility::LoaderErrorType::UNSUPPORTED_TYPE, "unexpected 3D profile type", line.expressID, line.ifcType);
    break;
  }

  return IfcProfile();
}

glm::dvec3 IfcGeometryLoader::GetVector(uint32_t expressID) const
{
  uint32_t lineID = _loader.ExpressIDToLineID(expressID);
  auto &line = _loader.GetLine(lineID);

  _loader.MoveToArgumentOffset(line, 0);
  auto positionID = _loader.GetRefArgument();
  double length = _loader.GetDoubleArgument();

  glm::dvec3 direction = GetCartesianPoint3D(positionID);
  direction.x = direction.x * length;
  direction.y = direction.y * length;
  direction.z = direction.z * length;

  return direction;
}

template<size_t N>
glm::mat<N+1,N+1,double> IfcGeometryLoader::GetPlacement<N>(uint32_t expressID) const
{
  uint32_t lineID = _loader.ExpressIDToLineID(expressID);
  auto &line = _loader.GetLine(lineID);
  switch (line.ifcType)
  {
    case N == 2 && schema::IFCAXIS2PLACEMENT2D:
    {
      uint32_t lineID = _loader.ExpressIDToLineID(expressID);
      auto &line = _loader.GetLine(lineID);

      _loader.MoveToArgumentOffset(line, 0);
      uint32_t locationID = _loader.GetRefArgument();
      parsing::IfcTokenType dirToken = _loader.GetTokenType();

      glm::dvec2 xAxis = glm::dvec2(1, 0);
      if (dirToken == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        xAxis = glm::normalize(GetCartesianPoint<N>(_loader.GetRefArgument()));
      }

      auto pos = GetCartesianPoint<n>(locationID);

      glm::dvec2 yAxis = glm::normalize(glm::dvec2(xAxis.y, -xAxis.x));

      return glm::dmat3(
        glm::dvec3(xAxis, 0),
        glm::dvec3(yAxis, 0),
        glm::dvec3(pos, 1));
    }
  case N==2 && schema::IFCCARTESIANTRANSFORMATIONOPERATOR2D:
  case N==2 && schema::IFCCARTESIANTRANSFORMATIONOPERATOR2DNONUNIFORM:
    {
      double scale1 = 1.0;
      double scale2 = 1.0;

      glm::dvec2 Axis1(1, 0);
      glm::dvec2 Axis2(0, 1);

      _loader.MoveToArgumentOffset(line, 0);
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        Axis1 = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
      }
      _loader.MoveToArgumentOffset(line, 1);
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        Axis2 = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
      }

      _loader.MoveToArgumentOffset(line, 2);
      uint32_t posID = _loader.GetRefArgument();
      glm::dvec2 pos = GetCartesianPoint2D(posID);

      _loader.MoveToArgumentOffset(line, 3);
      if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
      {
        _loader.StepBack();
        scale1 = _loader.GetDoubleArgument();
      }

      if (line.ifcType == schema::IFCCARTESIANTRANSFORMATIONOPERATOR2DNONUNIFORM)
      {
        _loader.MoveToArgumentOffset(line, 4);
        if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
        {
          _loader.StepBack();
          scale2 = _loader.GetDoubleArgument();
        }
      }

      if (line.ifcType == schema::IFCCARTESIANTRANSFORMATIONOPERATOR2D)
      {
        scale2 = scale1;
      }

      return glm::dmat3(
        glm::dvec3(Axis1 * scale1, 0),
        glm::dvec3(Axis2 * scale2, 0),
        glm::dvec3(pos, 1));
    }
    case N==3 && schema::IFCAXIS1PLACEMENT:
    {
      glm::dvec3 zAxis(0, 0, 1);
      glm::dvec3 xAxis(1, 0, 0);
      _loader.MoveToArgumentOffset(line, 0);
      uint32_t posID = _loader.GetRefArgument();
      parsing::IfcTokenType zID = _loader.GetTokenType();
      if (zID == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        zAxis = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
      }
      glm::dvec3 pos = GetCartesianPoint3D(posID);
      if (std::abs(glm::dot(xAxis, zAxis)) > 0.9)
      {
        xAxis = glm::dvec3(0, 1, 0);
      }
      glm::dvec3 yAxis = glm::normalize(glm::cross(zAxis, xAxis));
      xAxis = glm::normalize(glm::cross(zAxis, yAxis));

      glm::dmat4 result = glm::dmat4(
        glm::dvec4(xAxis, 0),
        glm::dvec4(yAxis, 0),
        glm::dvec4(zAxis, 0),
        glm::dvec4(pos, 1));

      return result;
    }
  case N==3 && schema::IFCAXIS2PLACEMENT3D:
    {
      glm::dvec3 zAxis(0, 0, 1);
      glm::dvec3 xAxis(1, 0, 0);

      _loader.MoveToArgumentOffset(line, 0);
      uint32_t posID = _loader.GetRefArgument();
      parsing::IfcTokenType zID = _loader.GetTokenType();
      if (zID == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        zAxis = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
      }

      _loader.MoveToArgumentOffset(line, 2);
      parsing::IfcTokenType xID = _loader.GetTokenType();
      if (xID == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        xAxis = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
      }

      glm::dvec3 pos = GetCartesianPoint3D(posID);

      glm::dvec3 yAxis = glm::normalize(glm::cross(zAxis, xAxis));
      xAxis = glm::normalize(glm::cross(yAxis, zAxis));

      return glm::dmat4(
        glm::dvec4(xAxis, 0),
        glm::dvec4(yAxis, 0),
        glm::dvec4(zAxis, 0),
        glm::dvec4(pos, 1));
    }
  case n==3 && schema::IFCLOCALPLACEMENT:
    {
      glm::dmat4 relPlacement(1);

      _loader.MoveToArgumentOffset(line, 0);
      parsing::IfcTokenType relPlacementToken = _loader.GetTokenType();
      if (relPlacementToken == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        relPlacement = GetLocalPlacement(_loader.GetRefArgument());
      }

      _loader.MoveToArgumentOffset(line, 1);
      uint32_t axis2PlacementID = _loader.GetRefArgument();

      glm::dmat4 axis2Placement = GetLocalPlacement(axis2PlacementID);

      auto result = relPlacement * axis2Placement;
      return result;
    }
  case N==3 && schema::IFCCARTESIANTRANSFORMATIONOPERATOR3D:
  case N==3 && schema::IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM:
    {
      double scale1 = 1.0;
      double scale2 = 1.0;
      double scale3 = 1.0;

      glm::dvec3 Axis1(1, 0, 0);
      glm::dvec3 Axis2(0, 1, 0);
      glm::dvec3 Axis3(0, 0, 1);

      _loader.MoveToArgumentOffset(line, 0);
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        Axis1 = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
      }
      _loader.MoveToArgumentOffset(line, 1);
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        Axis2 = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
      }

      _loader.MoveToArgumentOffset(line, 2);
      uint32_t posID = _loader.GetRefArgument();
      glm::dvec3 pos = GetCartesianPoint3D(posID);

      _loader.MoveToArgumentOffset(line, 3);
      if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
      {
        _loader.StepBack();
        scale1 = _loader.GetDoubleArgument();
      }

      _loader.MoveToArgumentOffset(line, 4);
      if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
      {
        _loader.StepBack();
        Axis3 = glm::normalize(GetCartesianPoint3D(_loader.GetRefArgument()));
      }

      if (line.ifcType == schema::IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM)
      {
        _loader.MoveToArgumentOffset(line, 5);
        if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
        {
          _loader.StepBack();
          scale2 = _loader.GetDoubleArgument();
        }

        _loader.MoveToArgumentOffset(line, 6);
        if (_loader.GetTokenType() == parsing::IfcTokenType::REAL)
        {
          _loader.StepBack();
          scale3 = _loader.GetDoubleArgument();
        }
      }

      if (line.ifcType == schema::IFCCARTESIANTRANSFORMATIONOPERATOR3D)
      {
        scale2 = scale1;
        scale3 = scale1;
      }

      return glm::dmat4(
        glm::dvec4(Axis1 * scale1, 0),
        glm::dvec4(Axis2 * scale2, 0),
        glm::dvec4(Axis3 * scale3, 0),
        glm::dvec4(pos, 1));
    }
  default:
    _errorHandler.ReportError(utility::LoaderErrorType::UNSUPPORTED_TYPE, "unexpected placement type", line.expressID, line.ifcType);
    break;
  }
  return {};
}


std::unordered_map<uint32_t, std::vector<uint32_t>>  IfcGeometryLoader::PopulateRelVoidsMap()
{
  std::unordered_map<uint32_t, std::vector<uint32_t>> resultVector;
  auto relVoids = _loader.GetExpressIDsWithType(schema::IFCRELVOIDSELEMENT);

  for (uint32_t relVoidID : relVoids)
  {
    uint32_t lineID = _loader.ExpressIDToLineID(relVoidID);
    auto &line = _loader.GetLine(lineID);

    _loader.MoveToArgumentOffset(line, 4);

    uint32_t relatingBuildingElement = _loader.GetRefArgument();
    uint32_t relatedOpeningElement = _loader.GetRefArgument();

    resultVector[relatingBuildingElement].push_back(relatedOpeningElement);
  }
  return resultVector;
}

std::unordered_map<uint32_t, std::vector<uint32_t>>  IfcGeometryLoader::PopulateRelVoidsRelMap()
{
  std::unordered_map<uint32_t, std::vector<uint32_t>> resultVector;
  auto relVoids = _loader.GetExpressIDsWithType(schema::IFCRELVOIDSELEMENT);

  for (uint32_t relVoidID : relVoids)
  {
    uint32_t lineID = _loader.ExpressIDToLineID(relVoidID);
    auto &line = _loader.GetLine(lineID);

    _loader.MoveToArgumentOffset(line, 4);

    uint32_t relatingBuildingElement = _loader.GetRefArgument();

    resultVector[relatingBuildingElement].push_back(relVoidID);
  }
  return resultVector;
}


std::unordered_map<uint32_t, std::vector<uint32_t>>  IfcGeometryLoader::PopulateRelAggregatesMap()
{
  std::unordered_map<uint32_t, std::vector<uint32_t>>  resultVector;
  auto relVoids = _loader.GetExpressIDsWithType(schema::IFCRELAGGREGATES);

  for (uint32_t relVoidID : relVoids)
  {
    uint32_t lineID = _loader.ExpressIDToLineID(relVoidID);
    auto &line = _loader.GetLine(lineID);

    _loader.MoveToArgumentOffset(line, 4);

    uint32_t relatingBuildingElement = _loader.GetRefArgument();
    auto aggregates = _loader.GetSetArgument();

    for (auto &aggregate : aggregates)
    {
      uint32_t aggregateID = _loader.GetRefArgument(aggregate);
      resultVector[relatingBuildingElement].push_back(aggregateID);
    }
  }
  return resultVector;
}

std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> IfcGeometryLoader::PopulateStyledItemMap()
{
   std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> returnVector;
  auto styledItems = _loader.GetExpressIDsWithType(schema::IFCSTYLEDITEM);

  for (uint32_t styledItemID : styledItems)
  {
    uint32_t lineID = _loader.ExpressIDToLineID(styledItemID);
    auto &line = _loader.GetLine(lineID);

    _loader.MoveToArgumentOffset(line, 0);

    if (_loader.GetTokenType() == parsing::IfcTokenType::REF)
    {
     _loader.StepBack();
     uint32_t representationItem = _loader.GetRefArgument();

     auto styleAssignments = _loader.GetSetArgument();

     for (auto &styleAssignment : styleAssignments)
     {
      uint32_t styleAssignmentID = _loader.GetRefArgument(styleAssignment);
      returnVector[representationItem].emplace_back(styledItemID, styleAssignmentID);
    }
  }
}
return returnVector;
}

std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> IfcGeometryLoader::PopulateRelMaterialsMap()
{
  std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> resultVector;
  auto styledItems = _loader.GetExpressIDsWithType(schema::IFCRELASSOCIATESMATERIAL);

  for (uint32_t styledItemID : styledItems)
  {
    uint32_t lineID = _loader.ExpressIDToLineID(styledItemID);
    auto &line = _loader.GetLine(lineID);

    _loader.MoveToArgumentOffset(line, 5);

    uint32_t materialSelect = _loader.GetRefArgument();

    _loader.MoveToArgumentOffset(line, 4);

    auto RelatedObjects = _loader.GetSetArgument();

    for (auto &ifcRoot : RelatedObjects)
    {
     uint32_t ifcRootID = _loader.GetRefArgument(ifcRoot);
     resultVector[ifcRootID].emplace_back(styledItemID, materialSelect);
   }

 }
   return resultVector;
}

std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> IfcGeometryLoader::PopulateMaterialDefinitionsMap()
{
  std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> resultVector;
 auto matDefs = _loader.GetExpressIDsWithType(schema::IFCMATERIALDEFINITIONREPRESENTATION);

 for (uint32_t styledItemID : matDefs)
 {
  uint32_t lineID = _loader.ExpressIDToLineID(styledItemID);
  auto &line = _loader.GetLine(lineID);

  _loader.MoveToArgumentOffset(line, 2);

  auto representations = _loader.GetSetArgument();

  _loader.MoveToArgumentOffset(line, 3);

  uint32_t material = _loader.GetRefArgument();

  for (auto &representation : representations)
  {
   uint32_t representationID = _loader.GetRefArgument(representation);
   resultVector[material].emplace_back(styledItemID, representationID);
 }
}
return resultVector;
}

void IfcGeometryLoader::ReadLinearScalingFactor()
{
  auto projects = _loader.GetExpressIDsWithType(schema::IFCPROJECT);

  if (projects.size() != 1)
  {
    std::cout << "SIZE:"<<projects.size()<<std::endl;
    _errorHandler.ReportError(utility::LoaderErrorType::PARSING, "unexpected empty ifc project");
    return;
  }

  auto projectEID = projects[0];

  auto &line = _loader.GetLine(_loader.ExpressIDToLineID(projectEID));
  _loader.MoveToArgumentOffset(line, 8);

  auto unitsID = _loader.GetRefArgument();

  auto &unitsLine =_loader. GetLine(_loader.ExpressIDToLineID(unitsID));
  _loader.MoveToArgumentOffset(unitsLine, 0);

  auto unitIds = _loader.GetSetArgument();

  for (auto &unitID : unitIds)
  {
    auto unitRef = _loader.GetRefArgument(unitID);

    auto &line = _loader.GetLine(_loader.ExpressIDToLineID(unitRef));

    if (line.ifcType == schema::IFCSIUNIT)
    {
     _loader.MoveToArgumentOffset(line, 1);
     std::string unitType = _loader.GetStringArgument();

     std::string unitPrefix;

     _loader.MoveToArgumentOffset(line, 2);
     if (_loader.GetTokenType() == parsing::IfcTokenType::ENUM)
     {
       _loader.StepBack();
       unitPrefix = _loader.GetStringArgument();
     }

     _loader.MoveToArgumentOffset(line, 3);
     std::string unitName = _loader.GetStringArgument();

     if (unitType == "LENGTHUNIT" && unitName == "METRE")
     {
       double prefix = ConvertPrefix(unitPrefix);
       _linearScalingFactor *= prefix;
     }
   }
   if(line.ifcType == schema::IFCCONVERSIONBASEDUNIT)
   {
     _loader.MoveToArgumentOffset(line, 1);
     std::string unitType = _loader.GetStringArgument();
     _loader.MoveToArgumentOffset(line, 3);
     auto unitRefLine = _loader.GetRefArgument();
     auto &unitLine = _loader.GetLine(_loader.ExpressIDToLineID(unitRefLine));

     _loader.MoveToArgumentOffset(unitLine, 1);
     auto ratios = _loader.GetSetArgument();

     ///Scale Correction

     _loader.MoveToArgumentOffset(unitLine, 2);
     auto scaleRefLine = _loader.GetRefArgument();

     auto &scaleLine = _loader.GetLine(_loader.ExpressIDToLineID(scaleRefLine));

     _loader.MoveToArgumentOffset(scaleLine, 1);
     std::string unitTypeScale = _loader.GetStringArgument();

     std::string unitPrefix;

     _loader.MoveToArgumentOffset(scaleLine, 2);
     if (_loader.GetTokenType() == parsing::IfcTokenType::ENUM)
     {
       _loader.StepBack();
       unitPrefix =_loader.GetStringArgument();
     }

     _loader.MoveToArgumentOffset(scaleLine, 3);
     std::string unitName = _loader.GetStringArgument();

     if (unitTypeScale == "LENGTHUNIT" && unitName == "METRE")
     {
       double prefix = ConvertPrefix(unitPrefix);
       _linearScalingFactor *= prefix;
     }

     double ratio = _loader.GetDoubleArgument(ratios[0]);
     if(unitType == "LENGTHUNIT")
     {
       _linearScalingFactor *= ratio;
     }
     else if (unitType == "AREAUNIT")
     {
       _squaredScalingFactor *= ratio;
     }
     else if (unitType == "VOLUMEUNIT")
     {
       _cubicScalingFactor *= ratio;
     }
     else if (unitType == "PLANEANGLEUNIT")
     {
       _angularScalingFactor *= ratio;
     }
   }        
 }
}

double IfcGeometryLoader::ConvertPrefix(const std::string &prefix)
{
  if (prefix == "") return 1;
  else if (prefix == "EXA") return 1e18;
  else if (prefix == "PETA") return 1e15;
  else if (prefix == "TERA") return 1e12;
  else if (prefix == "GIGA") return 1e9;
  else if (prefix == "MEGA") return 1e6;
  else if (prefix == "KILO") return 1e3;
  else if (prefix == "HECTO") return 1e2;
  else if (prefix == "DECA") return 10;
  else if (prefix == "DECI") return 1e-1;
  else if (prefix == "CENTI") return 1e-2;
  else if (prefix == "MILLI") return 1e-3;
  else if (prefix == "MICRO") return 1e-6;
  else if (prefix == "NANO") return 1e-9;
  else if (prefix == "PICO") return 1e-12;
  else if (prefix == "FEMTO") return 1e-15;
  else if (prefix == "ATTO") return 1e-18;
  else return 1;
}

const std::unordered_map<uint32_t, std::vector<uint32_t>> &IfcGeometryLoader::GetRelVoids() const
{ 
  return _relVoids;
}

const std::unordered_map<uint32_t, std::vector<uint32_t>> &IfcGeometryLoader::GetRelVoidRels() const
{ 
  return _relVoidRel;
}

const std::unordered_map<uint32_t, std::vector<uint32_t>> &IfcGeometryLoader::GetRelAggregates() const
{ ;
  return _relAggregates;
}

const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> &IfcGeometryLoader::GetStyledItems() const
{ 
  return _styledItems;
}

const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> &IfcGeometryLoader::GetRelMaterials() const
{ 
  return _relMaterials;
}

const std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> &IfcGeometryLoader::GetMaterialDefinitions() const
{ 
  return _materialDefinitions;
}

double IfcGeometryLoader::GetLinearScalingFactor() const
{ 
  return _linearScalingFactor;
}

std::vector<IfcSegmentIndexSelect> IfcGeometryLoader::ReadCurveIndices() const
{
  std::vector<IfcSegmentIndexSelect> result;

  parsing::IfcTokenType t = _loader.GetTokenType();
      // If you receive a reference then go to the reference
  if (t == parsing::IfcTokenType::REF)
  {
    _loader.StepBack();
    uint32_t lineID = _loader.ExpressIDToLineID(_loader.GetRefArgument());
    auto &line = _loader.GetLine(lineID);
    _loader.MoveToArgumentOffset(line, 0);
  }

  _loader.StepBack();
  while (_loader.GetTokenType() != parsing::IfcTokenType::SET_END)
  {
    _loader.StepBack();
    if (_loader.GetTokenType() == parsing::IfcTokenType::LABEL)
    {
      IfcSegmentIndexSelect segment;
      _loader.StepBack();
      segment.type = _loader.GetStringArgument();
      while (_loader.GetTokenType() != parsing::IfcTokenType::SET_END)
      {
        _loader.StepBack();
        while (_loader.GetTokenType() != parsing::IfcTokenType::SET_END)
        {
          _loader.StepBack();
          t = _loader.GetTokenType();
              // If you receive a real then add the real to the list
          if (t == parsing::IfcTokenType::REAL)
          {
            _loader.StepBack();
            segment.indexs.push_back(static_cast<uint32_t>(_loader.GetDoubleArgument()));
          }
        }
      }
      result.push_back(segment);
    }
  }

  return result;
}


}