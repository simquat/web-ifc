/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.  */

#pragma once

#include "../representation/constants.h"

namespace webifc::geometry {

	inline bool isConvexOrColinear(glm::dvec2 a, glm::dvec2 b, glm::dvec2 c)
	{
		return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) >= 0;
	}

 	template<size_t N>
	inline IfcCurve<N> BuildArc3Pt(const glm::dvec2 &p1, const glm::dvec2 &p2, const glm::dvec2 &p3,uint16_t circleSegments)
	{
		double f1 = (p1.x * p1.x - p2.x * p2.x + p1.y * p1.y - p2.y * p2.y);
		double f2 = (p1.x * p1.x - p3.x * p3.x + p1.y * p1.y - p3.y * p3.y);
		double v = 2 * (p1.x - p2.x) * (p1.y - p3.y) - 2 * (p1.x - p3.x) * (p1.y - p2.y);

		double cenX = ((p1.y - p3.y) * f1 - (p1.y - p2.y) * f2) / v;
		double cenYa = (f2 - 2 * cenX * (p1.x - p3.x)) / (2 * (p1.y - p3.y));
		double cenYb = (f1 - 2 * cenX * (p1.x - p2.x)) / (2 * (p1.y - p2.y));
		double cenY = cenYa;
		if (std::isnan(cenY))
		{
			cenY = cenYb;
		}

		glm::dvec2 pCen;
		pCen.x = cenX;
		pCen.y = cenY;

		double radius = sqrt(pow(cenX - p1.x, 2) + pow(cenY - p1.y, 2));

			// Using geometrical subdivision to avoid complex calculus with angles

		std::vector<glm::dvec2> pointList;
		pointList.push_back(p1);
		pointList.push_back(p2);
		pointList.push_back(p3);

		while (pointList.size() < (size_t)circleSegments)
		{
			std::vector<glm::dvec2> tempPointList;
			for (uint32_t j = 0; j < pointList.size() - 1; j++)
			{
				glm::dvec2 pt = (pointList[j] + pointList[j + 1]);
				pt.x /= 2;
				pt.y /= 2;
				glm::dvec2 vc = glm::normalize(pt - pCen);
				pt = pCen + vc * radius;
				tempPointList.push_back(pointList[j]);
				tempPointList.push_back(pt);
			}
			tempPointList.push_back(pointList[pointList.size() - 1]);
			pointList = tempPointList;
		}
		IfcCurve<N> curve;
		for (uint32_t j = 0; j < pointList.size(); j++) curve.push_back(pointList.at(j));
			return curve;
	}


	
	inline	glm::dvec3 InterpolateRationalBSplineCurveWithKnots(double t, int degree, std::vector<glm::dvec3> points, std::vector<double> knots, std::vector<double> weights)
	{
		glm::dvec3 point;

		int domainLow = degree;
		int domainHigh = knots.size() - 1 - degree;

		double low = knots[domainLow];
		double high = knots[domainHigh];

		double tPrime = t * (high - low) + low;
		if (tPrime < low || tPrime > high)
		{
			printf("BSpline tPrime out of bounds\n");
			return glm::dvec3(0, 0, 0);
		}

		// find s (the spline segment) for the [t] value provided
		int s = 0;
		for (int i = domainLow; i < domainHigh; i++)
		{
			if (knots[i] <= tPrime && tPrime < knots[i + 1])
			{
				s = i;
				break;
			}
		}

		// TODO: this should be done before calling the function, instead of calling it for each t
		// convert points to homogeneous coordinates
		std::vector<glm::dvec4> homogeneousPoints;
		for (size_t i = 0; i < points.size(); i++)
		{
			glm::dvec3 p = points[i];
			glm::dvec4 h = glm::dvec4(p.x * weights[i], p.y * weights[i], p.z * weights[i], weights[i]);
			homogeneousPoints.push_back(h);
		}

		// l (level) goes from 1 to the curve degree + 1
		double alpha;
		for (int l = 1; l <= degree + 1; l++)
		{
			// build level l of the pyramid
			for (int i = s; i > s - degree - 1 + l; i--)
			{
				alpha = (tPrime - knots[i]) / (knots[i + degree + 1 - l] - knots[i]);

				// interpolate each component

				double x = (1 - alpha) * homogeneousPoints[i - 1].x + alpha * homogeneousPoints[i].x;
				double y = (1 - alpha) * homogeneousPoints[i - 1].y + alpha * homogeneousPoints[i].y;
				double z = (1 - alpha) * homogeneousPoints[i - 1].z + alpha * homogeneousPoints[i].z;
				double w = (1 - alpha) * homogeneousPoints[i - 1].w + alpha * homogeneousPoints[i].w;

				homogeneousPoints[i] = glm::dvec4(x, y, z, w);
			}
		}

		// convert back to cartesian and return
		point = glm::dvec3(homogeneousPoints[s].x / homogeneousPoints[s].w, homogeneousPoints[s].y / homogeneousPoints[s].w, homogeneousPoints[s].z / homogeneousPoints[s].w);
		return point;
	}

	inline	glm::dvec2 InterpolateRationalBSplineCurveWithKnots(double t, int degree, std::vector<glm::dvec2> points, std::vector<double> knots, std::vector<double> weights)
	{

		glm::dvec2 point;

		int domainLow = degree;
		int domainHigh = knots.size() - 1 - degree;

		double low = knots[domainLow];
		double high = knots[domainHigh];

		double tPrime = t * (high - low) + low;
		if (tPrime < low || tPrime > high)
		{
			printf("BSpline tPrime out of bounds\n");
			return glm::dvec2(0, 0);
		}

		// find s (the spline segment) for the [t] value provided
		int s = 0;
		for (int i = domainLow; i < domainHigh; i++)
		{
			if (knots[i] <= tPrime && tPrime < knots[i + 1])
			{
				s = i;
				break;
			}
		}

		// TODO: this should be done before calling the function, instead of calling it for each t
		// convert points to homogeneous coordinates
		std::vector<glm::dvec3> homogeneousPoints;
		for (size_t i = 0; i < points.size(); i++)
		{
			glm::dvec2 p = points[i];
			glm::dvec3 h = glm::dvec3(p.x * weights[i], p.y * weights[i], weights[i]);
			homogeneousPoints.push_back(h);
		}

		// l (level) goes from 1 to the curve degree + 1
		double alpha;
		for (int l = 1; l <= degree + 1; l++)
		{
			// build level l of the pyramid
			for (int i = s; i > s - degree - 1 + l; i--)
			{
				alpha = (tPrime - knots[i]) / (knots[i + degree + 1 - l] - knots[i]);

				// interpolate each component

				double x = (1 - alpha) * homogeneousPoints[i - 1].x + alpha * homogeneousPoints[i].x;
				double y = (1 - alpha) * homogeneousPoints[i - 1].y + alpha * homogeneousPoints[i].y;
				double w = (1 - alpha) * homogeneousPoints[i - 1].z + alpha * homogeneousPoints[i].z;
				glm::dvec3 p = glm::dvec3(x, y, w);

				homogeneousPoints[i] = p;
			}
		}

		// convert back to cartesian and return
		point = glm::dvec2(homogeneousPoints[s].x / homogeneousPoints[s].z, homogeneousPoints[s].y / homogeneousPoints[s].z);
		return point;
	}



	inline	std::vector<glm::dvec3> GetRationalBSplineCurveWithKnots(int degree, std::vector<glm::dvec3> points, std::vector<double> knots, std::vector<double> weights)
	{

		std::vector<glm::dvec3> c;

		for (double i = 0; i < 1; i += 0.05)
		{
			glm::dvec3 point = InterpolateRationalBSplineCurveWithKnots(i, degree, points, knots, weights);
			c.push_back(point);
		}

		// TODO: flip triangles?
		/*
				if (MatrixFlipsTriangles(placement))
				{
					c.Invert();
				}
		*/

		return c;
	}

	inline	std::vector<glm::dvec2> GetRationalBSplineCurveWithKnots(int degree, std::vector<glm::dvec2> points, std::vector<double> knots, std::vector<double> weights)
	{
		std::vector<glm::dvec2> c;

		for (double i = 0; i < 1; i += 0.05)
		{
			glm::dvec2 point = InterpolateRationalBSplineCurveWithKnots(i, degree, points, knots, weights);
			c.push_back(point);
		}
		// TODO: flip triangles?
		/*
				if (MatrixFlipsTriangles(placement))
				{
					c.Invert();
				}
		*/
		return c;
	}

	template<size_t N>
	inline bool IsCurveConvex(IfcCurve<N> &curve)
	{
		for (size_t i = 2; i < curve.size(); i++)
		{
			glm::dvec2 a = curve[i - 2];
			glm::dvec2 b = curve[i - 1];
			glm::dvec2 c = curve[i - 0];

			if (!isConvexOrColinear(a, b, c))
			{
				return false;
			}
		}

		return true;
	}

	inline bool MatrixFlipsTriangles(const glm::dmat3 &mat)
	{
		return glm::determinant(mat) < 0;
	}

	template<size_t N>
	inline IfcCurve<N> GetEllipseCurve(float radiusX, float radiusY, int numSegments, glm::dmat3 placement = glm::dmat3(1), double startRad = 0, double endRad = CONST_PI * 2, bool swap = true)
	{
		IfcCurve<N> c;

		for (int i = 0; i < numSegments; i++)
		{
			double ratio = static_cast<double>(i) / (numSegments - 1);
			double angle = startRad + ratio * (endRad - startRad);

			glm::dvec2 circleCoordinate;
			if (swap)
			{
				circleCoordinate = glm::dvec2(
					radiusX * std::cos(angle),
					radiusY * std::sin(angle));
			}
			else
			{
				circleCoordinate = glm::dvec2(
					radiusX * std::sin(angle),
					radiusY * std::cos(angle));
			}
			glm::dvec2 pos = placement * glm::dvec3(circleCoordinate, 1);
			c.push_back(glm::dvec3(pos,0));
		}

		// check for a closed curve
		if (endRad == CONST_PI * 2 && startRad == 0)
		{
			c.push_back(c[0]);

			if (MatrixFlipsTriangles(placement)) std::reverse(c.begin(), c.end());
			
		}

		return c;
	}

 	template<size_t N>
	inline IfcCurve<N> GetCircleCurve(float radius, int numSegments, glm::dmat3 placement = glm::dmat3(1))
	{
		return GetEllipseCurve<N>(radius, radius, numSegments, placement);
	}

	template<size_t N>
	inline IfcCurve<N> GetRectangleCurve(double xdim, double ydim, glm::dmat3 placement = glm::dmat3(1))
	{
		double halfX = xdim / 2;
		double halfY = ydim / 2;

		glm::dvec2 bl = placement * glm::dvec3(-halfX, -halfY, 1);
		glm::dvec2 br = placement * glm::dvec3(halfX, -halfY, 1);

		glm::dvec2 tl = placement * glm::dvec3(-halfX, halfY, 1);
		glm::dvec2 tr = placement * glm::dvec3(halfX, halfY, 1);

		IfcCurve<N> c;
		c.push_back(bl);
		c.push_back(br);
		c.push_back(tr);
		c.push_back(tl);
		c.push_back(bl);

		if (MatrixFlipsTriangles(placement)) std::reverse(c.begin(), c.end());
		
		return c;
	}

  	template<size_t N>
	inline IfcCurve<N> GetIShapedCurve(double width, double depth, double webThickness, double flangeThickness, bool hasFillet, double filletRadius, glm::dmat3 placement = glm::dmat3(1))
	{
		IfcCurve<N> c;

		double hw = width / 2;
		double hd = depth / 2;
		double hweb = webThickness / 2;

		c.push_back(placement * glm::dvec3(-hw, +hd, 1));				   // TL
		c.push_back(placement * glm::dvec3(+hw, +hd, 1));				   // TR
		c.push_back(placement * glm::dvec3(+hw, +hd - flangeThickness, 1)); // TR knee

		if (hasFillet)
		{
			// TODO: interpolate
			c.push_back(placement * glm::dvec3(+hweb + filletRadius, +hd - flangeThickness, 1)); // TR elbow start
			c.push_back(placement * glm::dvec3(+hweb, +hd - flangeThickness - filletRadius, 1)); // TR elbow end

			c.push_back(placement * glm::dvec3(+hweb, -hd + flangeThickness + filletRadius, 1)); // BR elbow start
			c.push_back(placement * glm::dvec3(+hweb + filletRadius, -hd + flangeThickness, 1)); // BR elbow end
		}
		else
		{
			c.push_back(placement * glm::dvec3(+hweb, +hd - flangeThickness, 1)); // TR elbow
			c.push_back(placement * glm::dvec3(+hweb, -hd + flangeThickness, 1)); // BR elbow
		}

		c.push_back(placement * glm::dvec3(+hw, -hd + flangeThickness, 1)); // BR knee
		c.push_back(placement * glm::dvec3(+hw, -hd, 1));				   // BR

		c.push_back(placement * glm::dvec3(-hw, -hd, 1));				   // BL
		c.push_back(placement * glm::dvec3(-hw, -hd + flangeThickness, 1)); // BL knee

		if (hasFillet)
		{
			// TODO: interpolate
			c.push_back(placement * glm::dvec3(-hweb - filletRadius, -hd + flangeThickness, 1)); // BL elbow start
			c.push_back(placement * glm::dvec3(-hweb, -hd + flangeThickness + filletRadius, 1)); // BL elbow end

			c.push_back(placement * glm::dvec3(-hweb, +hd - flangeThickness - filletRadius, 1)); // TL elbow start
			c.push_back(placement * glm::dvec3(-hweb - filletRadius, +hd - flangeThickness, 1)); // TL elbow end
		}
		else
		{
			c.push_back(placement * glm::dvec3(-hweb, -hd + flangeThickness, 1)); // BL elbow
			c.push_back(placement * glm::dvec3(-hweb, +hd - flangeThickness, 1)); // TL elbow
		}

		c.push_back(placement * glm::dvec3(-hw, +hd - flangeThickness, 1)); // TL knee
		c.push_back(placement * glm::dvec3(-hw, +hd, 1));				   // TL

		if (MatrixFlipsTriangles(placement))
		{
			c.Invert();
		}

		return c;
	}

 	template<size_t N>
	inline IfcCurve<N> GetUShapedCurve(double depth, double flangeWidth, double webThickness, double flangeThickness, double filletRadius, double edgeRadius, double flangeSlope, glm::dmat3 placement = glm::dmat3(1))
	{
		IfcCurve<N> c;

		double hd = depth / 2;
		double hw = flangeWidth / 2;
		//		double hweb = webThickness / 2;
		double slopeOffsetRight = flangeSlope * hw;
		double slopeOffsetLeft = flangeSlope * (hw - webThickness);
		// double flangeReferencePointY = hd - flangeThickness;

		// TODO: implement the radius

		c.push_back(placement * glm::dvec3(-hw, +hd, 1));
		c.push_back(placement * glm::dvec3(+hw, +hd, 1));

		c.push_back(placement * glm::dvec3(+hw, +hd - flangeThickness + slopeOffsetRight, 1));

		c.push_back(placement * glm::dvec3(-hw + webThickness, +hd - flangeThickness, 1 - slopeOffsetLeft));
		c.push_back(placement * glm::dvec3(-hw + webThickness, -hd + flangeThickness, 1 + slopeOffsetLeft));

		c.push_back(placement * glm::dvec3(+hw, -hd + flangeThickness - slopeOffsetRight, 1));

		c.push_back(placement * glm::dvec3(+hw, -hd, 1));
		c.push_back(placement * glm::dvec3(-hw, -hd, 1));

		c.push_back(placement * glm::dvec3(-hw, +hd, 1));

		if (MatrixFlipsTriangles(placement))
		{
			c.Invert();
		}

		return c;
	}

 	template<size_t N>
	inline IfcCurve<N> GetLShapedCurve(double width, double depth, double thickness, bool hasFillet, double filletRadius, double edgeRadius, double legSlope, glm::dmat3 placement = glm::dmat3(1))
	{
		IfcCurve<N> c;

		double hw = width / 2;
		double hd = depth / 2;
		double hweb = thickness / 2;

		c.push_back(placement * glm::dvec3(+hw, +hd, 1));
		c.push_back(placement * glm::dvec3(+hw, -hd, 1));
		c.push_back(placement * glm::dvec3(-hw, -hd, 1));

		if (hasFillet)
		{
			// TODO: Create interpolation and sloped lines
		}
		else
		{
			c.push_back(placement * glm::dvec3(-hw, -hd + thickness, 1));
			c.push_back(placement * glm::dvec3(+hw - thickness, -hd + thickness, 1));
			c.push_back(placement * glm::dvec3(+hw - thickness, +hd, 1));
		}

		c.push_back(placement * glm::dvec3(+hw, +hd, 1));

		if (MatrixFlipsTriangles(placement))
		{
			c.Invert();
		}

		return c;
	}

	template<size_t N>
	inline IfcCurve<N> GetTShapedCurve(double width, double depth, double thickness, bool hasFillet, double filletRadius, double edgeRadius, double legSlope, glm::dmat3 placement = glm::dmat3(1))
	{
		IfcCurve<N> c;

		double hw = width / 2;
		double hd = depth / 2;
		double hweb = thickness / 2;

		c.push_back(placement * glm::dvec3(hw, hd, 1));

		if (hasFillet)
		{
			// TODO: Create interpolation and sloped lines
		}
		else
		{
			c.push_back(placement * glm::dvec3(hw, hd - thickness, 1));
			c.push_back(placement * glm::dvec3(hweb, hd - thickness, 1));
			c.push_back(placement * glm::dvec3(hweb, -hd, 1));
			c.push_back(placement * glm::dvec3(-hweb, -hd, 1));
			c.push_back(placement * glm::dvec3(-hweb, hd - thickness, 1));
			c.push_back(placement * glm::dvec3(-hw, hd - thickness, 1));
			c.push_back(placement * glm::dvec3(-hw, hd, 1));
		}

		c.push_back(placement * glm::dvec3(hw, hd, 1));

		if (MatrixFlipsTriangles(placement))
		{
			c.Invert();
		}

		return c;
	}

 	template<size_t N>
	inline IfcCurve<N> GetCShapedCurve(double width, double depth, double girth, double thickness, bool hasFillet, double filletRadius, glm::dmat3 placement = glm::dmat3(1))
	{
		IfcCurve<N> c;

		double hw = width / 2;
		double hd = depth / 2;
		// double hweb = thickness / 2;

		c.push_back(placement * glm::dvec3(-hw, hd, 1));
		c.push_back(placement * glm::dvec3(hw, hd, 1));

		if (hasFillet)
		{
			// TODO: Create interpolation and sloped lines
		}
		else
		{
			c.push_back(placement * glm::dvec3(hw, hd - girth, 1));
			c.push_back(placement * glm::dvec3(hw - thickness, hd - girth, 1));
			c.push_back(placement * glm::dvec3(hw - thickness, hd - thickness, 1));
			c.push_back(placement * glm::dvec3(-hw + thickness, hd - thickness, 1));
			c.push_back(placement * glm::dvec3(-hw + thickness, -hd + thickness, 1));
			c.push_back(placement * glm::dvec3(hw - thickness, -hd + thickness, 1));
			c.push_back(placement * glm::dvec3(hw - thickness, -hd + girth, 1));
			c.push_back(placement * glm::dvec3(hw, -hd + girth, 1));
			c.push_back(placement * glm::dvec3(hw, -hd, 1));
			c.push_back(placement * glm::dvec3(-hw, -hd, 1));
		}

		c.push_back(placement * glm::dvec3(-hw, hd, 1));

		if (MatrixFlipsTriangles(placement))
		{
			c.Invert();
		}

		return c;
	}

	template<size_t N>
	inline IfcCurve<N> GetZShapedCurve(double depth, double flangeWidth, double webThickness, double flangeThickness, double filletRadius, double edgeRadius, glm::dmat3 placement = glm::dmat3(1))
	{
		IfcCurve<N> c;
		double hw = flangeWidth / 2;
		double hd = depth / 2;
		double hweb = webThickness / 2;
		// double hfla = flangeThickness / 2;

		c.push_back(placement * glm::dvec3(-hw, hd, 1));
		c.push_back(placement * glm::dvec3(hweb, hd, 1));
		c.push_back(placement * glm::dvec3(hweb, -hd + flangeThickness, 1));
		c.push_back(placement * glm::dvec3(hw, -hd + flangeThickness, 1));
		c.push_back(placement * glm::dvec3(hw, -hd, 1));
		c.push_back(placement * glm::dvec3(-hweb, -hd, 1));
		c.push_back(placement * glm::dvec3(-hweb, hd - flangeThickness, 1));
		c.push_back(placement * glm::dvec3(-hw, hd - flangeThickness, 1));
		c.push_back(placement * glm::dvec3(-hw, hd, 1));

		if (MatrixFlipsTriangles(placement))
		{
			c.Invert();
		}

		return c;
	}


	template<size_t N>
	inline IfcCurve<N> BuildArc(const glm::dvec3 &pos, const glm::dvec3 &axis, double angleRad,uint16_t _circleSegments)
	{
		IfcCurve<N> curve;

		// project pos onto axis

		double pdota = glm::dot(axis, pos);
		glm::dvec3 pproja = pdota * axis;

		glm::dvec3 right = -(pos - pproja);
		glm::dvec3 up = glm::cross(axis, right);

		auto curve2D = GetEllipseCurve<2>(1, 1, _circleSegments, glm::dmat3(1), 0, angleRad, true);

		for (auto &pt2D : curve2D)
		{
			glm::dvec3 pt3D = pos + pt2D.x * right + pt2D.y * up;
			curve.push_back(pt3D);
		}

		return curve;
	}

}