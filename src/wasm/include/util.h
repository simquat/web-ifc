/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */
 
#pragma once

#include <sstream>
#include <fstream>
#include <vector>
#include <array>
#include <unordered_map>
#include <optional>

#include "../deps/glm/glm/glm.hpp"

constexpr bool CSG_DEBUG_OUTPUT = false;

#define CONST_PI 3.141592653589793238462643383279502884L

namespace webifc
{
	constexpr double EPS_NONZERO = 1e-20; // ???
	constexpr double EPS_MINISCULE = 1e-12; // what?
	constexpr double EPS_TINY = 1e-9;
	constexpr double EPS_SMALL = 1e-6;
	constexpr double EPS_BIG = 1e-4;


	static double areaOfTriangle(glm::dvec3 a, glm::dvec3 b, glm::dvec3 c)
	{
		glm::dvec3 ab = b - a;
		glm::dvec3 ac = c - a;

		glm::dvec3 norm = glm::cross(ab, ac);
		return glm::length(norm) / 2;
	}

	static bool MatrixFlipsTriangles(const glm::dmat4& mat)
	{
		return glm::determinant(mat) < 0;
	}

	static bool MatrixFlipsTriangles(const glm::dmat3& mat)
	{
		return glm::determinant(mat) < 0;
	}

	static void writeFile(std::wstring filename, std::string data)
    {
    #ifdef _MSC_VER
		std::ofstream out(L"debug_output/" + filename);
		out << data;
    #endif
    }

	// for some reason std::string_view is not compiling...
	struct StringView
	{
		StringView(char* d, uint32_t l) :
			data(d),
			len(l)
		{

		}

		char* data;
		uint32_t len;
	};

	struct Face
	{
		int i0;
		int i1;
		int i2;
	};

	struct Loop
	{
		bool hasOne;
		glm::dvec2 v1;
		glm::dvec2 v2;
	};

	constexpr int VERTEX_FORMAT_SIZE_FLOATS = 6;

	static glm::dvec3 computeNormal(const glm::dvec3 v1, const glm::dvec3 v2, const glm::dvec3 v3)
	{
		glm::dvec3 v12(v2 - v1);
		glm::dvec3 v13(v3 - v1);

		v12 = glm::normalize(v12);
		v13 = glm::normalize(v13);

		glm::dvec3 norm = glm::cross(v12, v13);

		return glm::normalize(norm);
	}

	// just follow the ifc spec, damn
	static bool computeSafeNormal(const glm::dvec3 v1, const glm::dvec3 v2, const glm::dvec3 v3, glm::dvec3& normal, double eps = 0)
	{
		glm::dvec3 v12(v2 - v1);
		glm::dvec3 v13(v3 - v1);

		v12 = glm::normalize(v12);
		v13 = glm::normalize(v13);

		glm::dvec3 norm = glm::cross(v12, v13);

		double len = glm::length(norm);

		if (std::isnan(len) || len <= eps)
		{
			return false;
		}

		normal = norm / len;

		return true;
	}

	static bool IsInsideCenterExtents(const glm::dvec3& pt, const glm::dvec3& center, const glm::dvec3& extents)
	{
		glm::dvec3 delta = pt - center;
		delta = glm::abs(delta);
		glm::dvec3 offset = delta - extents;
		
		return offset.x < EPS_SMALL && offset.y < EPS_SMALL && offset.z < EPS_SMALL;
	}

	struct AABB
	{
		uint32_t index;
		glm::dvec3 min = glm::dvec3(DBL_MAX, DBL_MAX, DBL_MAX);
		glm::dvec3 max = glm::dvec3(-DBL_MAX, -DBL_MAX, -DBL_MAX);
		glm::dvec3 center = glm::dvec3();

		bool intersects(const AABB& other) const
		{
			constexpr double eps = EPS_BIG;
			return (max.x + eps >= other.min.x && other.max.x + eps >= min.x &&
					max.y + eps >= other.min.y && other.max.y + eps >= min.y &&
					max.z + eps >= other.min.z && other.max.z + eps >= min.z);
		}

		void merge(const AABB& other)
		{
			min = glm::min(min, other.min);
			max = glm::max(max, other.max);
		}

		// https://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
		bool Intersect(const glm::dvec3& origin, const glm::dvec3& dir) const
		{// r.dir is unit direction vector of ray
			glm::dvec3 dirfrac;
			dirfrac.x = 1.0f / dir.x;
			dirfrac.y = 1.0f / dir.y;
			dirfrac.z = 1.0f / dir.z;
			// lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
			// r.org is origin of ray
			double t1 = (min.x - origin.x) * dirfrac.x;
			double t2 = (max.x - origin.x) * dirfrac.x;
			double t3 = (min.y - origin.y) * dirfrac.y;
			double t4 = (max.y - origin.y) * dirfrac.y;
			double t5 = (min.z - origin.z) * dirfrac.z;
			double t6 = (max.z - origin.z) * dirfrac.z;

			double tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
			double tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

			// if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
			if (tmax < -EPS_BIG)
			{
				//t = tmax;
				return false;
			}

			// if tmin > tmax, ray doesn't intersect AABB
			if (tmin > tmax + EPS_BIG)
			{
				//t = tmax;
				return false;
			}

			//t = tmin;
			return true;
		}
	};

	struct IfcGeometry
	{
		std::vector<float> fvertexData;
		std::vector<double> vertexData;
		std::vector<uint32_t> indexData;

		uint32_t numPoints = 0;
		uint32_t numFaces = 0;

		inline void AddPoint(glm::dvec4& pt, glm::dvec3& n)
		{
			glm::dvec3 p = pt;
			AddPoint(p, n);
		}

		inline void AddPoint(glm::dvec3& pt, glm::dvec3& n)
		{
			//vertexData.reserve((numPoints + 1) * VERTEX_FORMAT_SIZE_FLOATS);
			//vertexData[numPoints * VERTEX_FORMAT_SIZE_FLOATS + 0] = pt.x;
			//vertexData[numPoints * VERTEX_FORMAT_SIZE_FLOATS + 1] = pt.y;
			//vertexData[numPoints * VERTEX_FORMAT_SIZE_FLOATS + 2] = pt.z;
			vertexData.push_back(pt.x);
			vertexData.push_back(pt.y);
			vertexData.push_back(pt.z);

			vertexData.push_back(n.x);
			vertexData.push_back(n.y);
			vertexData.push_back(n.z);

			if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
			{
				printf("NaN in geom!\n");
			}

			if (std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z))
			{
				printf("NaN in geom!\n");
			}

			//vertexData[numPoints * VERTEX_FORMAT_SIZE_FLOATS + 3] = n.x;
			//vertexData[numPoints * VERTEX_FORMAT_SIZE_FLOATS + 4] = n.y;
			//vertexData[numPoints * VERTEX_FORMAT_SIZE_FLOATS + 5] = n.z;

			numPoints += 1;
		}

		inline void AddFace(glm::dvec3 a, glm::dvec3 b, glm::dvec3 c)
		{
			glm::dvec3 normal;

			double area = areaOfTriangle(a, b, c);

			if (!computeSafeNormal(a, b, c, normal))
			{
				// bail out, zero area triangle
				printf("zero tri");
				return;
			}

			AddPoint(a, normal);
			AddPoint(b, normal);
			AddPoint(c, normal);

			AddFace(numPoints - 3, numPoints - 2, numPoints - 1);
		}

		inline void AddFace(uint32_t a, uint32_t b, uint32_t c)
		{
			//indexData.reserve((numFaces + 1) * 3);
			//indexData[numFaces * 3 + 0] = a;
			//indexData[numFaces * 3 + 1] = b;
			//indexData[numFaces * 3 + 2] = c;
			indexData.push_back(a);
			indexData.push_back(b);
			indexData.push_back(c);

			double area = areaOfTriangle(GetPoint(a), GetPoint(b), GetPoint(c));

			glm::dvec3 normal;
			if (!computeSafeNormal(GetPoint(a), GetPoint(b), GetPoint(c), normal))
			{
				// bail out, zero area triangle
				printf("zero tri");
			}

			numFaces++;
		}

		inline Face GetFace(uint32_t index) const
		{
			Face f;
			f.i0 = indexData[index * 3 + 0];
			f.i1 = indexData[index * 3 + 1];
			f.i2 = indexData[index * 3 + 2];
			return f;
		}

		inline AABB GetFaceBox(uint32_t index) const
		{
			AABB aabb;
			aabb.index = index;

			glm::dvec3 a = GetPoint(indexData[index * 3 + 0]);
			glm::dvec3 b = GetPoint(indexData[index * 3 + 1]);
			glm::dvec3 c = GetPoint(indexData[index * 3 + 2]);

			aabb.min = glm::min(a, aabb.min);
			aabb.min = glm::min(b, aabb.min);
			aabb.min = glm::min(c, aabb.min);

			aabb.max = glm::max(a, aabb.max);
			aabb.max = glm::max(b, aabb.max);
			aabb.max = glm::max(c, aabb.max);

			aabb.center = (aabb.max + aabb.min) / 2.0;

			return aabb;
		}

		inline glm::dvec3 GetPoint(uint32_t index) const
		{
			return glm::dvec3(
				vertexData[index * VERTEX_FORMAT_SIZE_FLOATS + 0],
				vertexData[index * VERTEX_FORMAT_SIZE_FLOATS + 1],
				vertexData[index * VERTEX_FORMAT_SIZE_FLOATS + 2]
			);
		}

		void GetCenterExtents(glm::dvec3& center, glm::dvec3& extents) const
		{
			glm::dvec3 min(DBL_MAX, DBL_MAX, DBL_MAX);
			glm::dvec3 max(DBL_MIN, DBL_MIN, DBL_MIN);

			for (size_t i = 0; i < numPoints; i ++)
			{
				auto pt = GetPoint(i);
				min = glm::min(min, pt);
				max = glm::max(max, pt);
			}

			extents = (max - min);
			center = min + extents / 2.0;
		}

		IfcGeometry Normalize(glm::dvec3 center, glm::dvec3 extents) const
		{
			IfcGeometry newGeom;

			double scale = std::max(extents.x, std::max(extents.y, extents.z)) / 10.0;

			for (size_t i = 0; i < numFaces; i++)
			{
				auto face = GetFace(i);
				auto pa = GetPoint(face.i0);
				auto pb = GetPoint(face.i1);
				auto pc = GetPoint(face.i2);

				auto a = (pa - center) / scale;
				auto b = (pb - center) / scale;
				auto c = (pc - center) / scale;

				// std::cout << areaOfTriangle(pa, pb, pc) << std::endl;

				newGeom.AddFace(a, b, c);
			}


			return newGeom;
		}

		IfcGeometry DeNormalize(glm::dvec3 center, glm::dvec3 extents) const
		{
			IfcGeometry newGeom;

			double scale = std::max(extents.x, std::max(extents.y, extents.z)) / 10.0;

			for (size_t i = 0; i < numFaces; i++)
			{
				auto face = GetFace(i);
				auto pa = GetPoint(face.i0);
				auto pb = GetPoint(face.i1);
				auto pc = GetPoint(face.i2);

				// std::cout << areaOfTriangle(pa, pb, pc) << std::endl;

				auto a = pa * scale + center;
				auto b = pb * scale + center;
				auto c = pc * scale + center;

				newGeom.AddFace(a, b, c);
			}


			return newGeom;
		}

		uint32_t GetVertexData()
		{
			// unfortunately webgl can't do doubles
			if (fvertexData.size() != vertexData.size())
			{
				fvertexData.resize(vertexData.size());
				for (size_t i = 0; i < vertexData.size(); i += 6)
				{
					fvertexData[i + 0] = vertexData[i + 0];
					fvertexData[i + 1] = vertexData[i + 1];
					fvertexData[i + 2] = vertexData[i + 2];
				
					fvertexData[i + 3] = vertexData[i + 3];
					fvertexData[i + 4] = vertexData[i + 4];
					fvertexData[i + 5] = vertexData[i + 5];
				}

				// cleanup
				// vertexData = {};
			}

			if (fvertexData.empty())
			{
				return 0;
			}

			return (uint32_t)&fvertexData[0];
		}

		uint32_t GetVertexDataSize()
		{
			return (uint32_t)fvertexData.size();
		}

		uint32_t GetIndexData()
		{
			return (uint32_t)&indexData[0];
		}

		uint32_t GetIndexDataSize()
		{
			return (uint32_t)indexData.size();
		}

		bool IsEmpty()
		{
			return vertexData.empty();
		}
	};

	static AABB GetAABB(const IfcGeometry& mesh)
	{
		AABB aabb;

		for (uint32_t i = 0; i < mesh.numPoints; i++)
		{
			aabb.min = glm::min(aabb.min, mesh.GetPoint(i));
			aabb.max = glm::max(aabb.max, mesh.GetPoint(i));
		}

		return aabb;
	}

	static bool equals2d(glm::dvec2 A, glm::dvec2 B, double eps = 0)
	{
		return std::fabs(A.x - B.x) <= eps && std::fabs(A.y - B.y) <= eps;
	}

	static bool equals(glm::dvec3 A, glm::dvec3 B, double eps = 0)
	{
		return std::fabs(A.x - B.x) <= eps && std::fabs(A.y - B.y) <= eps && std::fabs(A.z - B.z) <= eps;
	}

	static bool equals(double A, double B, double eps = 0)
	{
		return std::fabs(A - B) <= eps;
	}

	static double sign2D(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b)
	{
		return (p.x - b.x) * (a.y - b.y) - (a.x - b.x) * (p.y - b.y);
	}

	// https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
	static double signOneZero(double x)
	{
		return (x > 0) - (x < 0);
	}

	static double ComparableAngle(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b)
	{
		double upDown = sign2D(p, a, b) >= 0 ? 1 : -1;
		double dot = (1 + glm::dot(glm::normalize(a - b), glm::normalize(p - b))) / 2.0;
		return upDown * dot;
	}

	// https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
	static double DistancePointToLineSegment2D(const glm::dvec2& v, const glm::dvec2& w, const glm::dvec2& p)
	{
		// Return minimum distance between line segment vw and point p
		const double l2 = glm::length(v - w);  // i.e. |w-v|^2 -  avoid a sqrt
		if (l2 == 0.0) return glm::distance(p, v);   // v == w case
		// Consider the line extending the segment, parameterized as v + t (w - v).
		// We find projection of point p onto the line. 
		// It falls where t = [(p-v) . (w-v)] / |w-v|^2
		// We clamp t from [0,1] to handle points outside the segment vw.
		const double t = std::max(0.0, std::min(1.0, dot(p - v, w - v) / (l2 * l2)));
		const glm::dvec2 projection = v + t * (w - v);  // Projection falls on the segment
		return glm::distance(p, projection);
	}

	static bool PointOnLineSegment2D(const glm::dvec2& v, const glm::dvec2& w, const glm::dvec2& p, double EPS)
	{
		double dist = DistancePointToLineSegment2D(v, w, p);
		return dist <= EPS;
	}

	static bool onEdge2D(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b, double EPS)
	{
		double dist = std::fabs(sign2D(p, a, b));
		return dist <= EPS;
	}

	static bool IsVectorCCW(const std::vector<glm::dvec2>& points)
	{
		double sum = 0;

		for (int i = 0; i < points.size(); i++)
		{
			glm::dvec2 pt1 = points[i];
			glm::dvec2 pt2 = points[(i + 1) % points.size()];

			sum += (pt2.x - pt1.x) * (pt2.y + pt1.y);
		}

		return sum < 0;
	}

	template<uint32_t DIM>
	struct IfcCurve
	{
		std::vector<glm::vec<DIM, glm::f64>> points;
		inline void Add(const glm::vec<DIM, glm::f64>& pt)
		{
			if (points.empty())
			{
				points.push_back(pt);
			}
			else
			{
				bool add = false;
				if constexpr (DIM == 2)
				{
					add = !equals2d(pt, points.back(), EPS_TINY);
				}
				else
				{
					add = !equals(pt, points.back(), EPS_TINY);
				}

				if (add)
				{
					points.push_back(pt);
				}
				else
				{
					// TODO: we are now discarding these points, but they probably should not even be generated
				}
			}
		}

		// the insert routine above does not deduplicate the looped points
		void DeduplicateLast()
		{
			if (points.empty())
			{
				return;
			}

			bool removeLast = false;
			if constexpr (DIM == 2)
			{
				removeLast = equals2d(points.front(), points.back(), EPS_TINY);
			}
			else
			{
				removeLast = equals(points.front(), points.back(), EPS_TINY);
			}

			if (removeLast)
			{
				points.pop_back();
			}
		}

		void Invert()
		{
			std::reverse(points.begin(), points.end());
		}

		bool IsCCW()
		{
			return IsVectorCCW(points);
		}
	};

	struct IfcSurface
	{
		glm::dmat4 transformation;
	};

	struct IfcTrimmingSelect
	{
		bool hasParam = false;
		bool hasPos = false;
		double param;
		glm::dvec2 pos;
	};

	struct IfcTrimmingArguments
	{
		bool exist = false;
		IfcTrimmingSelect start;
		IfcTrimmingSelect end;
	};

	struct IfcCurve3D
	{
		std::vector<glm::dvec3> points;
	};

	static IfcCurve<2> GetEllipseCurve(float radiusX, float radiusY, int numSegments, glm::dmat3 placement = glm::dmat3(1), double startRad = 0, double endRad = CONST_PI * 2, bool swap = true)
	{
		IfcCurve<2> c;

		for (int i = 0; i < numSegments; i++)
		{
			double ratio = static_cast<double>(i) / (numSegments - 1);
			double angle = startRad + ratio * (endRad - startRad);

			glm::dvec2 circleCoordinate;
			if (swap)
			{
				circleCoordinate = glm::dvec2(
					radiusX * std::cos(angle),
					radiusY * std::sin(angle)
				);
			}
			else
			{
				circleCoordinate = glm::dvec2(
					radiusX * std::sin(angle),
					radiusY * std::cos(angle)
				);
			}
			glm::dvec2 pos = placement * glm::dvec3(circleCoordinate, 1);
			c.points.push_back(pos);
		}

		// check for a closed curve
		if (endRad == CONST_PI * 2 && startRad == 0)
		{
			if (MatrixFlipsTriangles(placement))
			{
				c.Invert();
			}
		}

		return c;
	}

	static IfcCurve<2> GetCircleCurve(float radius, int numSegments, glm::dmat3 placement = glm::dmat3(1))
	{
		return GetEllipseCurve(radius, radius, numSegments, placement);
	}

	static bool GetWindingOfTriangle(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c)
	{
		auto norm = computeNormal(a, b, c);
		return glm::dot(norm, glm::dvec3(0, 0, 1)) > 0.0;
	}

	static bool TriangleIsCW(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c)
	{
		auto norm = computeNormal(glm::dvec3(a, 0), glm::dvec3(b, 0), glm::dvec3(c, 0));
		return glm::dot(norm, glm::dvec3(0, 0, 1)) > 0.0;
	}

	static IfcCurve<2> GetRectangleCurve(double xdim, double ydim, glm::dmat3 placement = glm::dmat3(1))
	{
		double halfX = xdim / 2;
		double halfY = ydim / 2;

		glm::dvec2 bl = placement * glm::dvec3(-halfX, -halfY, 1);
		glm::dvec2 br = placement * glm::dvec3(halfX, -halfY, 1);

		glm::dvec2 tl = placement * glm::dvec3(-halfX, halfY, 1);
		glm::dvec2 tr = placement * glm::dvec3(halfX, halfY, 1);

		IfcCurve<2> c;
		c.points.push_back(bl);
		c.points.push_back(br);
		c.points.push_back(tr);
		c.points.push_back(tl);

		if (MatrixFlipsTriangles(placement))
		{
			c.Invert();
		}

		return c;
	}

	static IfcCurve<2> GetIShapedCurve(double width, double depth, double webThickness, double flangeThickness, bool hasFillet, double filletRadius, glm::dmat3 placement = glm::dmat3(1))
	{
		IfcCurve<2> c;

		double hw = width / 2;
		double hd = depth / 2;
		double hweb = webThickness / 2;

		c.points.push_back(placement * glm::dvec3(-hw, +hd, 1)); // TL
		c.points.push_back(placement * glm::dvec3(+hw, +hd, 1)); // TR
		c.points.push_back(placement * glm::dvec3(+hw, +hd - flangeThickness, 1)); // TR knee

		if (hasFillet)
		{
			// TODO: interpolate
			c.points.push_back(placement * glm::dvec3(+hweb + filletRadius, +hd - flangeThickness, 1));// TR elbow start
			c.points.push_back(placement * glm::dvec3(+hweb, +hd - flangeThickness - filletRadius, 1));// TR elbow end

			c.points.push_back(placement * glm::dvec3(+hweb, -hd + flangeThickness + filletRadius, 1));// BR elbow start
			c.points.push_back(placement * glm::dvec3(+hweb + filletRadius, -hd + flangeThickness, 1));// BR elbow end
		}
		else
		{
			c.points.push_back(placement * glm::dvec3(+hweb, +hd - flangeThickness, 1));// TR elbow 
			c.points.push_back(placement * glm::dvec3(+hweb, -hd + flangeThickness, 1));// BR elbow 
		}

		c.points.push_back(placement * glm::dvec3(+hw, -hd + flangeThickness, 1));// BR knee 
		c.points.push_back(placement * glm::dvec3(+hw, -hd, 1));// BR

		c.points.push_back(placement * glm::dvec3(-hw, -hd, 1));// BL
		c.points.push_back(placement * glm::dvec3(-hw, -hd + flangeThickness, 1));// BL knee

		if (hasFillet)
		{
			// TODO: interpolate
			c.points.push_back(placement * glm::dvec3(-hweb - filletRadius, -hd + flangeThickness, 1));// BL elbow start
			c.points.push_back(placement * glm::dvec3(-hweb, -hd + flangeThickness + filletRadius, 1));// BL elbow end

			c.points.push_back(placement * glm::dvec3(-hweb, +hd - flangeThickness - filletRadius, 1));// TL elbow start
			c.points.push_back(placement * glm::dvec3(-hweb - filletRadius, +hd - flangeThickness, 1));// TL elbow end
		}
		else
		{
			c.points.push_back(placement * glm::dvec3(-hweb, -hd + flangeThickness, 1));// BL elbow
			c.points.push_back(placement * glm::dvec3(-hweb, +hd - flangeThickness, 1));// TL elbow
		}

		c.points.push_back(placement * glm::dvec3(-hw, +hd - flangeThickness, 1));// TL knee

		if (MatrixFlipsTriangles(placement))
		{
			c.Invert();
		}

		return c;
	}

	static double DistanceToLine(const glm::dvec3& pt, const glm::dvec3& lpos, const glm::dvec3& ldir)
	{
		glm::dvec3 x2 = lpos + ldir;
		double l = glm::length(glm::cross(pt - lpos, pt - x2));
		double s = glm::length(ldir);
		return l / s;
	}
	
	static glm::dvec3 projectOntoPlane(const glm::dvec3& origin, const glm::dvec3& normal, const glm::dvec3& point, const glm::dvec3& dir)
	{
		// project {et} onto the plane, following the extrusion normal						
		double ldotn = glm::dot(dir, normal);
		if (ldotn == 0)
		{
			printf("0 direction in extrude\n");
			return glm::dvec3(0);
		}
		else
		{
			glm::dvec3 dpos = origin - glm::dvec3(point);
			double dist = glm::dot(dpos, normal) / ldotn;
			return point + dist * dir;
		}
	}

	static bool GetBasisFromCoplanarPoints(std::vector<glm::dvec3>& points, glm::dvec3& v1, glm::dvec3& v2, glm::dvec3& v3)
	{
		v1 = points[0];

		for (auto& p : points)
		{
			if (v1 != p)
			{
				v2 = p;
				break;
			}
		}

		glm::dvec3 normal;
		for (auto& p : points)
		{
			if (computeSafeNormal(v1, v2, p, normal, EPS_SMALL))
			{
				v3 = p;
				return true;
			}
		}

		return false;
	}

	enum class IfcBoundType
	{
		OUTERBOUND,
		BOUND
	};

	// TODO: IfcBound3D can probably be merged with IfcProfile
	struct IfcBound3D
	{
		IfcBoundType type;
		bool orientation;
		IfcCurve3D curve;
	};

	struct IfcProfile
	{
		std::string type;
		IfcCurve<2> curve;
		std::vector<IfcCurve<2>> holes;
		bool isConvex;
	};

	static std::array<double, 16> FlattenTransformation(const glm::dmat4& transformation)
	{
		std::array<double, 16> flatTransformation;

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				flatTransformation[i * 4 + j] = transformation[i][j];
			}
		}

		return flatTransformation;
	}

	struct IfcPlacedGeometry
	{
		glm::dvec4 color;
		glm::dmat4 transformation;
		std::array<double, 16> flatTransformation;
		uint32_t geometryExpressID;

		void SetFlatTransformation()
		{
			flatTransformation = FlattenTransformation(transformation);
		}
	};

	struct IfcFlatMesh
	{
		std::vector<IfcPlacedGeometry> geometries;
		uint32_t expressID;
	};

	struct IfcComposedMesh
	{
		glm::dvec4 color;
		glm::dmat4 transformation;
		uint32_t expressID;
		bool hasGeometry = false;
		bool hasColor = false;
		std::vector<IfcComposedMesh> children;
	};

	static std::optional<glm::dmat4> GetOrientationRec(IfcComposedMesh& mesh, std::unordered_map<uint32_t, IfcGeometry>& geometryMap, glm::dmat4 mat)
	{
		glm::dmat4 newMat = mat * mesh.transformation;

		bool transformationBreaksWinding = MatrixFlipsTriangles(newMat);

		auto geomIt = geometryMap.find(mesh.expressID);

		if (geomIt != geometryMap.end())
		{
			auto meshGeom = geomIt->second;

			if (meshGeom.numFaces)
			{
				for (uint32_t i = 0; i < meshGeom.numFaces; i++)
				{
					// Face f = meshGeom.GetFace(i);
					// glm::dvec3 a = newMat * glm::dvec4(meshGeom.GetPoint(f.i0), 1);

					return newMat;
				}
			}
		}

		for (auto& c : mesh.children)
		{
			auto v = GetOrientationRec(c, geometryMap, newMat);
			if (v.has_value())
			{
				return v;
			}
		}

		return std::nullopt;
	}

	static glm::dmat4 GetOrientation(IfcComposedMesh& mesh, std::unordered_map<uint32_t, IfcGeometry>& geometryMap)
	{
		auto v = GetOrientationRec(mesh, geometryMap, glm::dmat4(1));

		if (v.has_value())
		{
			return *v;
		}
		else
		{
			return glm::dmat4(1);
		}
	}

	static void flattenRecursive(IfcComposedMesh& mesh, std::unordered_map<uint32_t, IfcGeometry>& geometryMap, IfcGeometry& geom, glm::dmat4 mat)
	{
		glm::dmat4 newMat = mat * mesh.transformation;

		bool transformationBreaksWinding = MatrixFlipsTriangles(newMat);

		auto geomIt = geometryMap.find(mesh.expressID);

		if (geomIt != geometryMap.end())
		{
			auto meshGeom = geomIt->second;

			if (meshGeom.numFaces)
			{
				for (uint32_t i = 0; i < meshGeom.numFaces; i++)
				{
					Face f = meshGeom.GetFace(i);
					glm::dvec3 a = newMat * glm::dvec4(meshGeom.GetPoint(f.i0), 1);
					glm::dvec3 b = newMat * glm::dvec4(meshGeom.GetPoint(f.i1), 1);
					glm::dvec3 c = newMat * glm::dvec4(meshGeom.GetPoint(f.i2), 1);

					if (transformationBreaksWinding)
					{
						geom.AddFace(b, a, c);
					}
					else
					{
						geom.AddFace(a, b, c);
					}
				}
			}
		}

		for (auto& c : mesh.children)
		{
			flattenRecursive(c, geometryMap, geom, newMat);
		}
	}

	static IfcGeometry flatten(IfcComposedMesh& mesh, std::unordered_map<uint32_t, IfcGeometry>& geometryMap, glm::dmat4 mat = glm::dmat4(1))
	{
		IfcGeometry geom;
		flattenRecursive(mesh, geometryMap, geom, mat);
		return geom;
	}

	static std::vector<glm::dvec2> rescale(std::vector<glm::dvec2> input, glm::dvec2 size, glm::dvec2 offset)
	{
		std::vector<glm::dvec2> retval;

		glm::dvec2 min(
			DBL_MAX,
			DBL_MAX
		);

		glm::dvec2 max(
			-DBL_MAX,
			-DBL_MAX
		);

		for (auto& pt : input)
		{
			min = glm::min(min, pt);
			max = glm::max(max, pt);
		}

		double width = max.x - min.x;
		double height = max.y - min.y;

		double maxSize = std::max(width, height);

		if (width == 0 && height == 0)
		{
			printf("asdf\n");
		}

		for (auto& pt : input)
		{
			// here we invert Y, since the canvas +y is down, but makes more sense to think about +y as up
			retval.emplace_back(
				((pt.x - min.x) / (maxSize)) * size.x + offset.x,
				(size.y - ((pt.y - min.y) / (maxSize)) * size.y) + offset.y
			);
		}

		return retval;
	}

	static std::string makeSVGLines(std::vector<glm::dvec2> input, std::vector<uint32_t> indices)
	{
		glm::dvec2 size(512, 512);
		glm::dvec2 offset(5, 5);

		auto rescaled = rescale(input, size, offset);

		std::stringstream svg;

		svg << "<svg width=\"" << size.x + offset.x * 2 << "\" height=\"" << size.y + offset.y * 2 << "\" xmlns=\"http://www.w3.org/2000/svg\" >";

		if (!rescaled.empty())
		{
			for (int i = 1; i < 2; i++)
			{
				auto& start = rescaled[i - 1];
				auto& end = rescaled[i];
				svg << "<line x1=\"" << start.x << "\" y1=\"" << start.y << "\" ";
				svg << "x2=\"" << end.x << "\" y2=\"" << end.y << "\" ";
				svg << "style = \"stroke:rgb(0,255,0);stroke-width:2\" />";
			}

			for (int i = 2; i < rescaled.size() - 1; i++)
			{
				auto& start = rescaled[i - 1];
				auto& end = rescaled[i];
				svg << "<line x1=\"" << start.x << "\" y1=\"" << start.y << "\" ";
				svg << "x2=\"" << end.x << "\" y2=\"" << end.y << "\" ";
				svg << "style = \"stroke:rgb(0,0,0);stroke-width:2\" />";
			}

			for (int i = rescaled.size() - 1; i < rescaled.size(); i++)
			{
				auto& start = rescaled[i - 1];
				auto& end = rescaled[i];
				svg << "<line x1=\"" << start.x << "\" y1=\"" << start.y << "\" ";
				svg << "x2=\"" << end.x << "\" y2=\"" << end.y << "\" ";
				svg << "style = \"stroke:rgb(255,0,0);stroke-width:2\" />";
			}
		}

		for (int i = 0; i < indices.size(); i += 3)
		{
			glm::dvec2 a = rescaled[indices[i + 0]];
			glm::dvec2 b = rescaled[indices[i + 1]];
			glm::dvec2 c = rescaled[indices[i + 2]];

			svg << "<polygon points=\"" << a.x << "," << a.y << " " << b.x << "," << b.y << " " << c.x << "," << c.y << "\" style=\"fill:gray; stroke:none; stroke - width:0\" />`;";
		}

		svg << "</svg>";

		return svg.str();
	}

	static void DumpSVGCurve(std::vector<glm::dvec2> points, std::wstring filename, std::vector<uint32_t> indices = {})
	{
        writeFile(filename, makeSVGLines(points, indices));
	}

	static void DumpSVGCurve(std::vector<glm::dvec3> points, glm::vec3 dir, std::wstring filename, std::vector<uint32_t> indices = {})
	{
		std::vector<glm::dvec2> points2D;
		for (auto& pt : points)
		{
			points2D.emplace_back(pt.x, pt.z);
		}
		DumpSVGCurve(points2D, filename, indices);
	}

    struct Point
    {
        double x;
        double y;
        int32_t id = -1;
		bool isBoundary = false;

		Point()
		{

		}

		Point(double xx, double yy)
		{
			x = xx;
			y = yy;
		}

		Point(glm::dvec2 p)
		{
			x = p.x;
			y = p.y;
		}

        glm::dvec2 operator()() const
        {
            return glm::dvec2(
                x, y
            );
        }
    };

    struct Triangle
    {
        Point a;
        Point b;
        Point c;

        int32_t id = -1;
    };

    struct Edge
    {
        int32_t a = -1;
        int32_t b = -1;
    };

	struct Bounds
	{
		glm::dvec2 min;
		glm::dvec2 max;

		void Merge(const Bounds& other)
		{
			min = glm::min(min, other.min);
			max = glm::max(max, other.max);
		}
	};

	static glm::dvec2 cmin(glm::dvec2 m, Point p)
	{
		return glm::dvec2(
			std::min(m.x, p.x),
			std::min(m.y, p.y)
		);
	}

	static glm::dvec2 cmax(glm::dvec2 m, Point p)
	{
		return glm::dvec2(
			std::max(m.x, p.x),
			std::max(m.y, p.y)
		);
	}

	static Bounds getBounds(std::vector<Triangle> input, glm::dvec2 size, glm::dvec2 offset)
	{
		std::vector<glm::dvec2> retval;

		glm::dvec2 min(
			DBL_MAX,
			DBL_MAX
		);

		glm::dvec2 max(
			-DBL_MAX,
			-DBL_MAX
		);

		for (auto& tri : input)
		{
			min = cmin(min, tri.a);
			max = cmax(max, tri.a);

			min = cmin(min, tri.b);
			max = cmax(max, tri.b);

			min = cmin(min, tri.c);
			max = cmax(max, tri.c);
		}

		double width = max.x - min.x;
		double height = max.y - min.y;

		if (width == 0 && height == 0)
		{
			printf("asdf");
		}

		return {
			min,
			max
		};
	}

	static Bounds getBounds(std::vector<std::vector<glm::dvec2>> input, glm::dvec2 size, glm::dvec2 offset)
	{
		std::vector<glm::dvec2> retval;

		glm::dvec2 min(
			DBL_MAX,
			DBL_MAX
		);

		glm::dvec2 max(
			-DBL_MAX,
			-DBL_MAX
		);

		for (auto& loop : input)
		{
			for (auto& point : loop)
			{
				min = glm::min(min, point);
				max = glm::max(max, point);
			}
		}

		double width = max.x - min.x;
		double height = max.y - min.y;

		if (width == 0 && height == 0)
		{
			printf("asdf");
		}

		return {
			min,
			max
		};
	}

	static glm::dvec2 rescale(Point p, Bounds b, glm::dvec2 size, glm::dvec2 offset)
	{
		return glm::dvec2(
			((p.x - b.min.x) / (b.max.x - b.min.x)) * size.x + offset.x,
			((p.y - b.min.y) / (b.max.y - b.min.y)) * size.y + offset.y
		);
	}

	static void svgMakeLine(glm::dvec2 a, glm::dvec2 b, std::stringstream& svg, std::string col = "rgb(255,0,0)")
	{
		svg << "<line x1=\"" << a.x << "\" y1=\"" << a.y << "\" ";
		svg << "x2=\"" << b.x << "\" y2=\"" << b.y << "\" ";
		svg << "style = \"stroke:" + col + ";stroke-width:1\" />";
	}

	static std::string makeSVGTriangles(std::vector<Triangle> triangles, Point p, Point prev, std::vector<Point> pts = {})
	{
		glm::dvec2 size(512, 512);
		glm::dvec2 offset(5, 5);

		Bounds bounds = getBounds(triangles, size, offset);

		std::stringstream svg;

		svg << "<svg width=\"" << size.x + offset.x * 2 << "\" height=\"" << size.y + offset.y * 2 << "  \" xmlns=\"http://www.w3.org/2000/svg\" >";

		for (auto& t : triangles)
		{
			if (t.id != -1)
			{
				glm::dvec2 a = rescale(t.a, bounds, size, offset);
				glm::dvec2 b = rescale(t.b, bounds, size, offset);
				glm::dvec2 c = rescale(t.c, bounds, size, offset);

				svgMakeLine(a, b, svg);
				svgMakeLine(b, c, svg);
				svgMakeLine(c, a, svg);
			}
		}

		glm::dvec2 rp = rescale(p, bounds, size, offset);
		glm::dvec2 rprev = rescale(prev, bounds, size, offset);

		if (p.id != -1)
		{
			svg << "<circle cx = \"" << rp.x << "\" cy = \"" << rp.y << "\" r = \"3\" style = \"stroke:rgb(0,0,255);stroke-width:2\" />";
		}

		if (prev.id != -1)
		{
			svg << "<circle cx = \"" << rprev.x << "\" cy = \"" << rprev.y << "\" r = \"3\" style = \"stroke:rgb(0,0,100);stroke-width:2\" />";
		}

		for (auto& pt : pts)
		{
			glm::dvec2 p = rescale(pt, bounds, size, offset);
			svg << "<circle cx = \"" << p.x << "\" cy = \"" << p.y << "\" r = \"1\" style = \"stroke:rgb(0,0,100);stroke-width:2\" />";
		}

		svg << "</svg>";

		return svg.str();
	}

	static glm::dvec2 rescale(glm::dvec2 p, Bounds b, glm::dvec2 size, glm::dvec2 offset)
	{
		return glm::dvec2(
			((p.x - b.min.x) / (b.max.x - b.min.x)) * size.x + offset.x,
			((p.y - b.min.y) / (b.max.y - b.min.y)) * size.y + offset.y
		);
	}

	struct SVGLineSet
	{
		std::vector<std::vector<glm::dvec2>> lines;
		std::string color = "rgb(255,0,0)";

		Bounds GetBounds(glm::dvec2 size, glm::dvec2 offset)
		{
			return getBounds(lines, size, offset);;
		}
	};

	struct SVGDrawing
	{
		std::vector<SVGLineSet> sets;

		Bounds GetBounds(glm::dvec2 size, glm::dvec2 offset)
		{
			Bounds b; 
			b.min = glm::dvec2(
				DBL_MAX,
				DBL_MAX
			);

			b.max = glm::dvec2(
				-DBL_MAX,
				-DBL_MAX
			);

			for (auto& set : sets)
			{
				b.Merge(set.GetBounds(size, offset));
			}

			return b;
		}
	};

	static void SVGLinesToString(Bounds bounds, glm::dvec2 size, glm::dvec2 offset, SVGLineSet lineSet, std::stringstream& svg)
	{
		for (auto& line : lineSet.lines)
		{
			if (line.size() > 1)
			{
				for (int i = 1; i < line.size(); i++)
				{
					glm::dvec2 a = rescale(line[i], bounds, size, offset);
					glm::dvec2 b = rescale(line[i - 1], bounds, size, offset);

					svgMakeLine(a, b, svg, lineSet.color);
				}
			}
			else
			{
				glm::dvec2 a = rescale(line[0], bounds, size, offset);
				svg << "<circle cx = \"" << a.x << "\" cy = \"" << a.y << "\" r = \"3\" style = \"stroke:rgb(0,0,255);stroke-width:2\" />";
			}
		}
	}

	static std::string makeSVGLines(SVGDrawing drawing)
	{
		glm::dvec2 size(2048, 2048);
		glm::dvec2 offset(5, 5);

		Bounds bounds = drawing.GetBounds(size, offset);

		std::stringstream svg;

		svg << "<svg width=\"" << size.x + offset.x * 2 << "\" height=\"" << size.y + offset.y * 2 << " \" xmlns=\"http://www.w3.org/2000/svg\">";

		for (auto& set : drawing.sets)
		{
			SVGLinesToString(bounds, size, offset, set, svg);
		}

		svg << "</svg>";

		return svg.str();
	}

	static void DumpSVGTriangles(std::vector<Triangle> triangles, Point p, Point prev, std::wstring filename, std::vector<Point> pts = {})
	{
        writeFile(filename, makeSVGTriangles(triangles, p, prev, pts));
	}

	static void DumpSVGLines(std::vector<std::vector<glm::dvec2>> lines, std::wstring filename)
	{
		SVGLineSet set;
		set.lines = lines;
		SVGDrawing drawing;
		drawing.sets.push_back(set);
        writeFile(filename, makeSVGLines(drawing));
	}

	static bool isConvexOrColinear(glm::dvec2 a, glm::dvec2 b, glm::dvec2 c)
	{
		return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) >= 0;
	}

	constexpr glm::dmat4 NormalizeIFC(
		glm::dvec4(1, 0, 0, 0),
		glm::dvec4(0, 0, -1, 0),
		glm::dvec4(0, 1, 0, 0),
		glm::dvec4(0, 0, 0, 1)
	);

	static double cross2d(const glm::dvec2& point1, const glm::dvec2& point2) {
		return point1.x * point2.y - point1.y * point2.x;
	}

	static double areaOfTriangle(glm::dvec2 a, glm::dvec2 b, glm::dvec2 c)
	{
		glm::dvec2 ab = b - a;
		glm::dvec2 ac = c - a;

		double norm = cross2d(ab, ac) / 2;
		return std::fabs(norm);
	}

	// https://en.wikipedia.org/wiki/Barycentric_coordinate_system
	static glm::dvec3 ToBary(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const glm::dvec3& pt)
	{
		glm::dvec3 E1 = b - a;
		glm::dvec3 E2 = c - a;
		glm::dvec3 ROV0 = pt - a;
		glm::dvec3 N = glm::cross(E1, E2);
		glm::dvec3 dir = -N;
		glm::dvec3 Q = glm::cross(ROV0, dir);
		double d = dot(dir, N);

		if (d == 0)
		{
			printf("bary conversion perp");
		}

		double det = 1.0 / d;
		double u = det * glm::dot(E2, (Q * -1.0));
		double v = det * glm::dot(E1, Q);
		double w = 1 - u - v;

		return glm::dvec3(w, u, v);
	}

	static glm::dvec2 FromBary(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c, const glm::dvec3& pt)
	{
		return pt.x * a + pt.y * b + pt.z * c;
	}

	// assume 0,0 1,0 0,1 triangle
	static glm::dvec3 ToBary2(const glm::dvec2& pt)
	{
		double v = pt.x;
		double w = pt.y;
		double u = 1 - v - w;

		return glm::dvec3(u, v, w);
	}

	static glm::dvec3 FromBary(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const glm::dvec3& pt)
	{
		return pt.x * a + pt.y * b + pt.z * c;
	}

	static void CheckTriangle(glm::dvec3 a, glm::dvec3 b, glm::dvec3 c)
	{
		if (areaOfTriangle(a, b, c) == 0)
		{
			printf("0 triangle\n");
		}
	}

	static void CheckTriangle(Face& f, std::vector<glm::dvec3>& pts)
	{
		if (areaOfTriangle(pts[f.i0], pts[f.i1], pts[f.i2]) == 0)
		{
			printf("0 triangle\n");
		}
	}

	static double RandomDouble(double lo, double hi)
	{
		return lo + static_cast<double>(rand()) / (static_cast<double> (RAND_MAX / (hi - lo)));
	}

	static std::string ToObj(const IfcGeometry& geom, size_t& offset, glm::dmat4 transform = glm::dmat4(1))
	{
		std::stringstream obj;

		double scale = 1.0;

		for (uint32_t i = 0; i < geom.numPoints; i++)
		{
			glm::dvec4 t = transform * glm::dvec4(geom.GetPoint(i), 1);
			obj << "v " << t.x * scale << " " << t.y * scale << " " << t.z * scale << "\n";
		}

		for (uint32_t i = 0; i < geom.numFaces; i++)
		{
			Face f = geom.GetFace(i);
			obj << "f " << (f.i0 + 1 + offset) << "// " << (f.i1 + 1 + offset) << "// " << (f.i2 + 1 + offset) << "//\n";
		}

		offset += geom.numPoints;

		return obj.str();
	}

	static std::string ToObj(IfcComposedMesh& mesh, std::unordered_map<uint32_t, IfcGeometry>& geometryMap, size_t& offset, glm::dmat4 mat = glm::dmat4(1))
	{
		std::string complete;

		glm::dmat4 trans = mat * mesh.transformation;

		auto& geom = geometryMap[mesh.expressID];

		complete += ToObj(geom, offset, trans);

		for (auto c : mesh.children)
		{
			complete += ToObj(c, geometryMap, offset, trans);
		}

		return complete;
	}

	static void DumpIfcGeometry(const IfcGeometry& geom, std::wstring filename)
	{
		size_t offset = 0;
        writeFile(filename, ToObj(geom, offset));
	}

	//! This is essentially a chunked tightly packed dynamic array
	template<uint32_t N>
	class DynamicTape
	{
	public:
		DynamicTape()
		{
			AddChunk();
		}

		inline void AddChunk()
		{
			chunks.emplace_back();
			memset(chunks.back().data(), 0, N);
			sizes.push_back(0);
			writePtr++;
		}

		inline void CheckChunk(unsigned long long size)
		{
			if (sizes[writePtr] + size >= N)
			{
				AddChunk();
			}
		}

		inline void push(char v)
		{
			CheckChunk(1);
			chunks.back().data()[sizes[writePtr]] = v;
			sizes[writePtr] += 1;
		}

		inline void push(void* v, unsigned long long size)
		{
			CheckChunk(size);
			memcpy(chunks.back().data() + sizes[writePtr], v, size);
			sizes[writePtr] += size;
		}

		inline void SetWriteAtEnd()
		{
			writePtr = chunks.size() - 1;
		}

		uint64_t GetTotalSize()
		{
			return (chunks.size() - 1) * N + sizes.back();
		}

		uint64_t GetCapacity()
		{
			return chunks.size() * N;
		}

		void Reset()
		{
			readChunkIndex = 0;
			readPtr = 0;
		}

		inline bool Expect(char expectedType)
		{
			auto receivedType = Read<char>();
			return (receivedType == expectedType);
		}

		template <typename T>
		inline T Read()
		{
			std::array<uint8_t, N>& chunk = chunks[readChunkIndex];
			uint8_t* valuePtr = &chunk[readPtr];

			//T v = *(T*)(valuePtr);
			// make this memory access aligned for emscripten

			T v;

			memcpy(&v, valuePtr, sizeof(T));

			AdvanceRead(sizeof(T));
			return v;
		}

		void* GetReadPtr()
		{
			std::array<uint8_t, N>& chunk = chunks[readChunkIndex];
			uint8_t* valuePtr = &chunk[readPtr];

			return (void*)valuePtr;
		}

        StringView ReadStringView()
        {
			uint8_t length = Read<uint8_t>();
			char* charPtr = (char*)GetReadPtr();
			AdvanceRead(length);

			return StringView { charPtr, length };
        }

		inline void Reverse()
		{
			if (readPtr > 0)
			{
				readPtr--;
			}
			else
			{
				readChunkIndex--;
				readPtr = static_cast<uint32_t>(sizes[readChunkIndex] - 1);
			}
		}

		inline bool AtEnd()
		{
			return readChunkIndex == chunks.size();
		}

		inline uint32_t GetReadOffset()
		{
			return readChunkIndex * N + readPtr;
		}

		inline void MoveTo(uint32_t pos)
		{
			readChunkIndex = pos / N;
			readPtr = pos % N;
		}

		void DumpToDisk()
		{
			std::ofstream file("tape.bin");
			for (int i = 0; i < chunks.size(); i++)
			{
				std::array<uint8_t, N>& ch = chunks[i];
				file.write((char*)ch.data(), sizes[i]);
			}
		}

		uint32_t Copy(uint32_t offset, uint32_t endOffset, uint8_t* dest)
		{
			uint32_t chunkStart = offset / N;
			uint32_t chunkStartPos = offset % N;

			uint32_t chunkEnd = endOffset / N;
			uint32_t chunkEndPos = endOffset % N;

			if (chunkStart == chunkEnd)
			{
				memcpy(dest, &chunks[chunkStart][chunkStartPos], chunkEndPos - chunkStartPos);
				
				return chunkEndPos - chunkStartPos;
			}
			else
			{
				uint32_t startChunkSize = sizes[chunkStart];
				uint32_t partOfStartchunk = startChunkSize - chunkStartPos;

				memcpy(dest, &chunks[chunkStart][chunkStartPos], partOfStartchunk);
				memcpy(dest + partOfStartchunk, &chunks[chunkEnd][0], chunkEndPos);

				return partOfStartchunk + chunkEndPos;
			}
		}

		inline void AdvanceRead(unsigned long long size)
		{
			readPtr += static_cast<uint32_t>(size);
			if (readPtr >= sizes[readChunkIndex])
			{
				readChunkIndex++;
				readPtr = 0;
			}
		}

	private:

		uint32_t readPtr = 0;
		uint32_t readChunkIndex = 0;
		uint32_t writePtr = -1;
		std::vector<std::array<uint8_t, N>> chunks;
		std::vector<size_t> sizes;
	};

}