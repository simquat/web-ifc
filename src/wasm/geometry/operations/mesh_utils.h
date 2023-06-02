/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.  */

#pragma once

#include <vector>
#include <array>
#include <unordered_map>
#include <optional>
#include <tinynurbs/tinynurbs.h>
#include "geometryutils.h"
#include <glm/glm.hpp>

#define CONST_PI 3.141592653589793238462643383279502884L

namespace webifc::geometry
{

	// TODO: review and simplify
	inline void TriangulateRevolution(IfcGeometry &geometry, std::vector<IfcBound<3>> &bounds, IfcSurface &surface)
	{
			// First we get the revolution data

		glm::dvec3 cent = surface.RevolutionSurface.Direction[3];
		glm::dvec3 vecX = glm::normalize(surface.RevolutionSurface.Direction[0]);
		glm::dvec3 vecY = glm::normalize(surface.RevolutionSurface.Direction[1]);
		glm::dvec3 vecZ = glm::normalize(surface.RevolutionSurface.Direction[2]);

		std::vector<std::vector<glm::dvec3>> newPoints;

		double numRots = 10;

		for (int r = 0; r < numRots; r++)
		{
			std::vector<glm::dvec3> newList;
			newPoints.push_back(newList);
		}

		std::vector<glm::dvec3> bounding;
		std::vector<double> angleVec;
		std::vector<double> angleDsp;

			// Now we construct the bounding box of the boundary ...
			// ... by adding the middle point of all curves

		for (size_t i = 0; i < bounds.size(); i++)
		{
			
			for (size_t j = 0; j < bounds[i].loop.size(); j++)
			{
				double xx = 0;
				double yy = 0;
				double zz = 0;
				for (size_t k = 0; k < bounds[i].loop[j].size(); j++) {
						xx += bounds[i].loop[j][k].x;
						yy += bounds[i].loop[j][k].y;
						zz += bounds[i].loop[j][k].z;
				}

				xx /= bounds[i].loop[j].size();
				yy /= bounds[i].loop[j].size();
				zz /= bounds[i].loop[j].size();
				bounding.push_back(glm::dvec3(xx, yy, zz));
			}
		}

			// There is a problem when points in the revolution are around 0 degrees
			// Numerical instabilities can make these points to jump from 0 to 360
			// It causes lots of trouble when drawing the boundaries in the revolution

			// The method presented here finds the angle of each point, measures the ...
			//  ... angular difference and then, if the difference is bigger than 180 ...
			//  ... corrects it to a lesser value. Finally it gets the first angle and ...
			//  ... adds the angular differences again, reconstructing a corrected boundary.

			// Now we find the angle of each point in the reference plane of the cylinder
		for (size_t j = 0; j < bounding.size(); j++)
		{
			double xx = bounding[j].x - cent.x;
			double yy = bounding[j].y - cent.y;
			double zz = bounding[j].z - cent.z;
			double dx = vecX.x * xx + vecX.y * yy + vecX.z * zz;
			double dy = vecY.x * xx + vecY.y * yy + vecY.z * zz;
				//				double dz = vecZ.x * xx + vecZ.y * yy + vecZ.z * zz;
			double temp = VectorToAngle(dx, dy);
			while (temp < 0)
			{
				temp += 360;
			}
			while (temp > 360)
			{
				temp -= 360;
			}
			angleVec.push_back(temp);
		}

		for (size_t i = 0; i < angleVec.size() - 1; i++)
		{
			if (angleVec[i] - angleVec[i + 1] > 180)
			{
				angleDsp.push_back(360 - (angleVec[i] - angleVec[i + 1]));
			}
			else if (angleVec[i] - angleVec[i + 1] < -180)
			{
				angleDsp.push_back(-(angleVec[i] - angleVec[i + 1] + 360));
			}
			else
			{
				angleDsp.push_back(angleVec[i + 1] - angleVec[i]);
			}
		}

		double startDegrees = angleVec[0];
		double endDegrees = angleVec[0];

			// Add angular differences starting from the first angle. We also correct the start and end angles

		double temp = angleVec[0];
		for (size_t i = 0; i < angleDsp.size(); i++)
		{
			temp += angleDsp[i];
			if (endDegrees < temp)
			{
				endDegrees = temp;
			}
			if (startDegrees > temp)
			{
				startDegrees = temp;
			}
		}

			// Then we use the start and end angles as bounding boxes of the boundary ...
			//  ... we will represent this bounding box.

		double startRad = startDegrees / 180 * CONST_PI;
		double endRad = endDegrees / 180 * CONST_PI;
		double radSpan = endRad - startRad;
		double radStep = radSpan / (numRots - 1);

		if (surface.ExtrusionSurface.Profile.isType(typeid(IfcProfile<3>)))
		{
			IfcProfile<3> profile =  surface.RevolutionSurface.Profile;

			for (size_t i = 0; i < profile.curve.size(); i++)
			{
				double xx = profile.curve[i].x - cent.x;
				double yy = profile.curve[i].y - cent.y;
				double zz = profile.curve[i].z - cent.z;

				double dx = vecX.x * xx + vecX.y * yy + vecX.z * zz;
				double dy = vecY.x * xx + vecY.y * yy + vecY.z * zz;
				double dz = vecZ.x * xx + vecZ.y * yy + vecZ.z * zz;
				double dd = sqrt(dx * dx + dy * dy);
				for (int r = 0; r < numRots; r++)
				{
					double angle = startRad + r * radStep;
					double dtempX = sin(angle) * dd;
					double dtempY = cos(angle) * dd;
					double newPx = dtempX * vecX.x + dtempY * vecY.x + dz * vecZ.x + cent.x;
					double newPy = dtempX * vecX.y + dtempY * vecY.y + dz * vecZ.y + cent.y;
					double newPz = dtempX * vecX.z + dtempY * vecY.z + dz * vecZ.z + cent.z;
					glm::dvec3 newPt = glm::dvec3(
						newPx,
						newPy,
						newPz);
					newPoints[r].push_back(newPt);
				}
			}
		}
		for (int r = 0; r < numRots - 1; r++)
		{
			int r1 = r + 1;
			for (size_t s = 0; s < newPoints[r].size() - 1; s++)
			{
				geometry.AddFace(newPoints[r][s], newPoints[r][s + 1], newPoints[r1][s]);
				geometry.AddFace(newPoints[r1][s], newPoints[r][s + 1], newPoints[r1][s + 1]);
			}
		}
	}

		// TODO: review and simplify
	inline void TriangulateCylindricalSurface(IfcGeometry &geometry, std::vector<IfcBound<3>> &bounds, IfcSurface &surface)
	{
			// First we get the cylinder data

		double radius = surface.CylinderSurface.Radius;
		glm::dvec3 cent = surface.transformation[3];
		glm::dvec3 vecX = glm::normalize(surface.transformation[0]);
		glm::dvec3 vecY = glm::normalize(surface.transformation[1]);
		glm::dvec3 vecZ = glm::normalize(surface.transformation[2]);

		std::vector<std::vector<glm::dvec3>> newPoints;

		double numRots = 10;
		double minZ = 1e+10;
		double maxZ = -1e+10;

			// Find the relative coordinates of each curve point in the cylinder reference plane
			// Only retain the max and min relative Z
		for (size_t i = 0; i < bounds.size(); i++)
		{
			for ( auto c : bounds[i].loop) {
				for (size_t j = 0; j < c.size(); j++)
				{
					glm::dvec3 vv = c[j] - cent;
						//					double dx = glm::dot(vecX, vv);
						//					double dy = glm::dot(vecY, vv);
					double dz = glm::dot(vecZ, vv);
					if (maxZ < dz)
					{
						maxZ = dz;
					}
					if (minZ > dz)
					{
						minZ = dz;
					}
				}
			}
		}

		for (int r = 0; r < numRots; r++)
		{
			std::vector<glm::dvec3> newList;
			newPoints.push_back(newList);
		}

		std::vector<glm::dvec3> bounding;
		std::vector<double> angleVec;
		std::vector<double> angleDsp;

		int maxTeam = 0;
		for (size_t i = 0; i < bounds.size(); i++)
		{
			if (bounds[i].loop.size() > maxTeam) maxTeam=bounds[i].loop.size();
		}

		std::vector<std::vector<glm::dvec3>> boundingGroups;

		// We group each point with their boundary

		for (int r = 0; r < maxTeam; r++)
		{
			std::vector<glm::dvec3> boundingTemp = std::vector<glm::dvec3>();
			for (size_t i = 0; i < bounds.size(); i++)
			{
				if ( r >= bounds[i].loop.size()) continue;
				for (size_t j = 0; j < bounds[i].loop[r].size(); j++)
				{
						boundingTemp.push_back(bounds[i].loop[r][j]);
				}
			}
		}

		int repeats = 0;
		bool start = false;
		bool end = false;
		size_t id = 0;

			// In the case of boundary lines having only 2 endings...
			//... we omit these lines and add solely curves having > 2 points...
			//... starting from a 2 point line, by doing it this way we don't have repeated points
		while (!end && repeats < maxTeam * 3)
		{
			if (id >= boundingGroups.size())
			{
				id = 0;
			}
			if (boundingGroups[id].size() < 3)
			{
				if (!start)
				{
					start = true;
				}
				else
				{
					break;
				}
			}
			if (boundingGroups[id].size() > 2 && start)
			{
				for (size_t i = 0; i < boundingGroups[id].size(); i++)
				{
					bounding.push_back(boundingGroups[id][i]);
				}
			}
			id++;
			repeats++;
		}

			// If the previous method finds nothing, then we don't have straight lines ...
			//... then we add all boundary points directly
		if (bounding.size() == 0)
		{
			for (size_t j = 0; j < boundingGroups.size(); j++)
			{
				for (size_t i = 0; i < boundingGroups[j].size(); i++)
				{
					bounding.push_back(boundingGroups[j][i]);
				}
			}
		}

		double startDegrees = 0;
		double endDegrees = 360;

			// Now we project the points in the cylinder surface
			// There is a problem when points in the cylinder are around 0 degrees
			// Numerical instabilities can make these points to jump from 0 to 360
			// It causes lots of trouble when drawing the boundaries in the cylinder

			// The method presented here finds the angle of each point, measures the ...
			//  ... angular difference and then, if the difference is bigger than 180 ...
			//  ... corrects it to a lesser value. Finally it gets the first angle and ...
			//  ... adds the angular differences again, reconstructing a corrected boundary.

			// Now we find the angle of each point in the reference plane of the cylinder
		for (size_t j = 0; j < bounding.size(); j++)
		{
			glm::dvec3 vv = bounding[j] - cent;
			double dx = glm::dot(vecX, vv);
			double dy = glm::dot(vecY, vv);
				// double dz = glm::dot(vecZ, vv);
			double temp = VectorToAngle(dx, dy);
			while (temp < 0)
			{
				temp += 360;
			}
			while (temp > 360)
			{
				temp -= 360;
			}
			angleVec.push_back(temp);
		}

			// Then we find the angular difference
		for (size_t i = 0; i < angleVec.size() - 1; i++)
		{
			if (angleVec[i] - angleVec[i + 1] > 180)
			{
				angleDsp.push_back(360 - (angleVec[i] - angleVec[i + 1]));
			}
			else if (angleVec[i] - angleVec[i + 1] < -180)
			{
				angleDsp.push_back(-(angleVec[i] - angleVec[i + 1] + 360));
			}
			else
			{
				angleDsp.push_back(angleVec[i + 1] - angleVec[i]);
			}
		}

		startDegrees = angleVec[0];
		endDegrees = angleVec[0];

			// Add angular differences starting from the first angle. We also correct the start and end angles

		double temp = angleVec[0];
		for (size_t i = 0; i < angleDsp.size(); i++)
		{
			temp += angleDsp[i];
			if (endDegrees < temp)
			{
				endDegrees = temp;
			}
			if (startDegrees > temp)
			{
				startDegrees = temp;
			}
		}

			// Then we use the start and end angles as bounding boxes of the boundary ...
			//  ... we will represent this bounding box.

		while (startDegrees < -360)
		{
			startDegrees += 360;
		}
		double startRad = startDegrees / 180 * CONST_PI;
		double endRad = endDegrees / 180 * CONST_PI;
		double radSpan = endRad - startRad;
		double radStep = radSpan / (numRots - 1);

		for (int r = 0; r < numRots; r++)
		{
			double angle = startRad + r * radStep;
			double dtempX = sin(angle) * radius;
			double dtempY = cos(angle) * radius;
			double newPx = dtempX * vecX.x + dtempY * vecY.x + minZ * vecZ.x + cent.x;
			double newPy = dtempX * vecX.y + dtempY * vecY.y + minZ * vecZ.y + cent.y;
			double newPz = dtempX * vecX.z + dtempY * vecY.z + minZ * vecZ.z + cent.z;
			glm::dvec3 newPt = glm::dvec3(
				newPx,
				newPy,
				newPz);
			newPoints[r].push_back(newPt);
		}
		for (int r = 0; r < numRots; r++)
		{
			double angle = startRad + r * radStep;
			double dtempX = sin(angle) * radius;
			double dtempY = cos(angle) * radius;
			double newPx = dtempX * vecX.x + dtempY * vecY.x + maxZ * vecZ.x + cent.x;
			double newPy = dtempX * vecX.y + dtempY * vecY.y + maxZ * vecZ.y + cent.y;
			double newPz = dtempX * vecX.z + dtempY * vecY.z + maxZ * vecZ.z + cent.z;
			glm::dvec3 newPt = glm::dvec3(
				newPx,
				newPy,
				newPz);
			newPoints[r].push_back(newPt);
		}

		for (int r = 0; r < numRots - 1; r++)
		{
			int r1 = r + 1;
			for (size_t s = 0; s < newPoints[r].size() - 1; s++)
			{
				geometry.AddFace(newPoints[r][s], newPoints[r][s + 1], newPoints[r1][s]);
				geometry.AddFace(newPoints[r1][s], newPoints[r][s + 1], newPoints[r1][s + 1]);
			}
		}
	}

		// TODO: review and simplify
	inline void TriangulateExtrusion(IfcGeometry &geometry, std::vector<IfcBound<3>> &bounds, IfcSurface &surface)
	{
			// NO EXAMPLE FILES ABOUT THIS CASE

			// THIS IS A SIMPLE EXTRUSION, NOT TRIMMED

		double len = surface.ExtrusionSurface.Length;
		glm::dvec3 dir = surface.ExtrusionSurface.Direction;

		if (surface.ExtrusionSurface.Profile.isType(typeid(IfcProfile<2>)))
		{
			IfcProfile<2> profile = surface.ExtrusionSurface.Profile;
			for (size_t j = 0; j < profile.curve.size() - 1; j++)
			{
				int j2 = j + 1;

				double npx = profile.curve[j].x + dir.x * len;
				double npy = profile.curve[j].y + dir.y * len;
				double npz = dir.z * len;
				glm::dvec3 nptj1 = glm::dvec3(
					npx,
					npy,
					npz);
				npx = profile.curve[j2].x + dir.x * len;
				npy = profile.curve[j2].y + dir.y * len;
				npz = dir.z * len;
				glm::dvec3 nptj2 = glm::dvec3(
					npx,
					npy,
					npz);
				geometry.AddFace(
					glm::dvec3(profile.curve[j].x,profile.curve[j].y, 0),
					glm::dvec3(profile.curve[j2].x,profile.curve[j2].y, 0),
					nptj1);
				geometry.AddFace(
					glm::dvec3(profile.curve[j2].x,profile.curve[j2].y, 0),
					nptj2,
					nptj1);
			}
		}
		else
		{
			std::vector<IfcProfile<2>> profiles = surface.ExtrusionSurface.Profile;
			for (auto profile : profiles)
			{
				for (size_t j = 0; j < profile.curve.size() - 1; j++)
				{
					int j2 = j + 1;

					double npx = profile.curve[j].x + dir.x * len;
					double npy = profile.curve[j].y + dir.y * len;
					double npz = dir.z * len;
					glm::dvec3 nptj1 = glm::dvec3(
						npx,
						npy,
						npz);
					npx = profile.curve[j2].x + dir.x * len;
					npy = profile.curve[j2].y + dir.y * len;
					npz = dir.z * len;
					glm::dvec3 nptj2 = glm::dvec3(
						npx,
						npy,
						npz);
					geometry.AddFace(
						glm::dvec3(profile.curve[j].x,profile.curve[j].y, 0),
						glm::dvec3(profile.curve[j2].x,profile.curve[j2].y, 0),
						nptj1);
					geometry.AddFace(
						glm::dvec3(profile.curve[j2].x,profile.curve[j2].y, 0),
						nptj2,
						nptj1);
				}
			}
		}
	}

	inline double InverseMethod(glm::dvec3 pt, tinynurbs::RationalSurface3d srf, double pr, double rotations, double minError, double maxError,
		double& fU, double& fV, double& divisor, double maxDistance, double& fUs, double& fVs)
	{
		while (maxDistance > maxError && divisor < 10000)
		{
			for (double r = 1; r < 5; r++)
			{
				int round = 0;
				while (maxDistance > minError && round < 3)
				{
					for (double i = 0; i < rotations; i++)
					{
						double rads = (i / rotations) * CONST_PI * 2;
						double incU = glm::sin(rads) / (r * r * divisor);
						double incV = glm::cos(rads) / (r * r * divisor);
						if (pr > 1)
						{
							incV *= pr;
						}
						else
						{
							incU /= pr;
						}
						bool repeat = true;
						while (repeat)
						{
							double ffU = fU + incU;
							double ffV = fV + incV;
							glm::highp_dvec3 pt00 = tinynurbs::surfacePoint(srf, ffU, ffV);
							double di = glm::distance(pt00, pt);
							if (di < maxDistance)
							{
								maxDistance = di;
								fU = ffU;
								fV = ffV;
							}
							else
							{
								repeat = false;
							}
						}
					}
					round++;
				}
			}
			divisor *= 3;
		}
		return maxDistance;
	}

	inline glm::dvec2 BSplineInverseEvaluation(glm::dvec3 pt, tinynurbs::RationalSurface3d srf)
	{
		glm::highp_dvec3 ptc = tinynurbs::surfacePoint(srf, 0.0, 0.0);
		glm::highp_dvec3 pth = tinynurbs::surfacePoint(srf, 1.0, 0.0);
		glm::highp_dvec3 ptv = tinynurbs::surfacePoint(srf, 0.0, 1.0);

		double dh = glm::distance(ptc, pth);
		double dv = glm::distance(ptc, ptv);
		double pr = (dh + 1) / (dv + 1);

		double minError = 0.0001;
		double maxError = 0.01;
		double rotations = 6;

		double fU = 0.5;
		double fV = 0.5;
		double divisor = 100.0;
		double maxDistance = 1e+100;
		double fUs = fU;
		double fVs = fV;

		maxDistance = InverseMethod(pt, srf, pr, rotations, minError, maxError, fU, fV, divisor, maxDistance, fUs, fVs);
		return glm::dvec2(fUs, fVs);
	}

		// TODO: review and simplify
	inline void TriangulateBspline(IfcGeometry &geometry, std::vector<IfcBound<3>> &bounds, IfcSurface &surface)
	{
			//			double limit = 1e-4;

			// First: We define the Nurbs surface

		tinynurbs::RationalSurface3d srf;
		srf.degree_u = surface.BSplineSurface.UDegree;
		srf.degree_v = surface.BSplineSurface.VDegree;
		size_t num_u = surface.BSplineSurface.ControlPoints.size();
		size_t num_v = surface.BSplineSurface.ControlPoints[0].size();

		std::vector<glm::dvec3> controlPoints;
		for (std::vector<glm::dvec3> row : surface.BSplineSurface.ControlPoints)
		{
			for (glm::dvec3 point : row)
			{
				controlPoints.push_back({point.x, point.y, point.z});
			}
		}
		srf.control_points = tinynurbs::array2(num_u, num_v, controlPoints);

		std::vector<double> weights;
		for (std::vector<double> row : surface.BSplineSurface.Weights)
		{
			for (double weight : row)
			{
				weights.push_back(weight);
			}
		}
		if (weights.size() != num_u * num_v)
		{
			for (size_t i = 0; i < num_u * num_v; i++)
			{
				weights.push_back(1.0);
			}
		}
		srf.weights = tinynurbs::array2(num_u, num_v, weights);

		for (size_t i = 0; i < surface.BSplineSurface.UMultiplicity.size(); i++)
		{
			for (int r = 0; r < surface.BSplineSurface.UMultiplicity[i]; r++)
			{
				srf.knots_u.push_back(surface.BSplineSurface.UKnots[i]);
			}
		}

		for (size_t i = 0; i < surface.BSplineSurface.VMultiplicity.size(); i++)
		{
			for (int r = 0; r < surface.BSplineSurface.VMultiplicity[i]; r++)
			{
				srf.knots_v.push_back(surface.BSplineSurface.VKnots[i]);
			}
		}

			// If the NURBS surface is valid we continue

		if (tinynurbs::surfaceIsValid(srf))
		{

				// Find projected boundary using NURBS inverse evaluation

			using Point = std::array<double, 2>;
			std::vector<std::vector<Point>> uvBoundaryValues;

			std::vector<Point> points;
			for (auto c : bounds[0].loop) {
				for (size_t j = 0; j < c.size(); j++)
				{
					glm::dvec3 pt = c[j];
					glm::dvec2 pInv = BSplineInverseEvaluation(pt, srf);
					points.push_back({pInv.x, pInv.y});
				}
			}
			uvBoundaryValues.push_back(points);

				// Triangulate projected boundary
				// Subdivide resulting triangles to increase definition
				// r indicates the level of subdivision, currently 3 you can increase it to 5

			std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(uvBoundaryValues);

			for (size_t r = 0; r < 3; r++)
			{
				std::vector<uint32_t> newIndices;
				std::vector<Point> newUVPoints;

				for (size_t i = 0; i < indices.size(); i += 3)
				{
					Point p0 = uvBoundaryValues[0][indices[i + 0]];
					Point p1 = uvBoundaryValues[0][indices[i + 1]];
					Point p2 = uvBoundaryValues[0][indices[i + 2]];

					newUVPoints.push_back(p0);
					newUVPoints.push_back(p1);
					newUVPoints.push_back(p2);

					Point p3 = {(p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2};
					Point p4 = {(p0[0] + p2[0]) / 2, (p0[1] + p2[1]) / 2};
					Point p5 = {(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2};

					newUVPoints.push_back(p3);
					newUVPoints.push_back(p4);
					newUVPoints.push_back(p5);

					int offset = newUVPoints.size() - 6;

					newIndices.push_back(offset + 0);
					newIndices.push_back(offset + 3);
					newIndices.push_back(offset + 4);

					newIndices.push_back(offset + 3);
					newIndices.push_back(offset + 5);
					newIndices.push_back(offset + 4);

					newIndices.push_back(offset + 3);
					newIndices.push_back(offset + 1);
					newIndices.push_back(offset + 5);

					newIndices.push_back(offset + 4);
					newIndices.push_back(offset + 5);
					newIndices.push_back(offset + 2);
				}

				uvBoundaryValues[0] = newUVPoints;
				indices = newIndices;
			}

			for (size_t i = 0; i < indices.size(); i += 3)
			{
				Point p0 = uvBoundaryValues[0][indices[i + 0]];
				Point p1 = uvBoundaryValues[0][indices[i + 1]];
				Point p2 = uvBoundaryValues[0][indices[i + 2]];
				glm::dvec3 pt00 = tinynurbs::surfacePoint(srf, p0[0], p0[1]);
				glm::dvec3 pt01 = tinynurbs::surfacePoint(srf, p1[0], p1[1]);
				glm::dvec3 pt10 = tinynurbs::surfacePoint(srf, p2[0], p2[1]);
				geometry.AddFace(pt00, pt01, pt10);
			}
		}
	}
}