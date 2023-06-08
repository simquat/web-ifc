/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.  */

//Generic Representations of geometry used in web-ifc

#pragma once

#include <array>
#include <optional>
#include <string>
#include <cstdio>
#include <algorithm>
#include <unordered_map>
#include <glm/glm.hpp>
#include <typeinfo>
#include <typeindex>


namespace webifc::geometry {

	template <size_t N> using IfcCurve = std::vector<glm::vec<N,double>>;

	template <typename T1,typename T2>
	class Select
	{
		public:
			void operator=(T1 t) { option1=t; has1=true; }
			void operator=(T2 t) { option2=t; has2=true; }
			operator T1() { return option1; }
			operator T2() { return option2; }
			bool isType(const std::type_index type) {  if (has1 && std::type_index(typeid(has1)) == type) return true; else if (has2 && std::type_index(typeid(has2)) == type) return true; else return false; }

		private:
			T1 option1;
			T2 option2;
			bool has1 = false;
			bool has2 = false;
	};

	template <size_t N> using IfcTrimmingSelect =  Select<double,glm::vec<N,double>>;
	
	template <size_t N>
	struct IfcProfile
	{
			IfcCurve<N> curve;
			std::vector<IfcCurve<N>> voids;
	};

	template <size_t N> using IfcProfileDef  = Select< std::vector<IfcProfile<N>>, IfcProfile<N> >;
	
	template <size_t N> using IfcLoop = std::vector<IfcCurve<N>>;


	enum class IfcBoundType
	{
		OUTERBOUND,
		BOUND
	};
	
	template <size_t N>
	struct IfcBound
	{
		IfcBoundType type;
		bool orientation;
		IfcLoop<N> loop;
	};

	const static std::unordered_map<std::string, int> Horizontal_alignment_type{
		{"LINE", 1},		
		{"CIRCULARARC", 2},	
		{"CLOTHOID", 3},	
		{"CUBICSPIRAL", 4},				//ToDo
		{"BIQUADRATICPARABOLA", 5},		//ToDo
		{"BLOSSCURVE", 6},				//ToDo
		{"COSINECURVE", 7},				//ToDo
		{"SINECURVE", 8},				//ToDo
		{"VIENNESEBEND", 9}				//ToDo
	};

	const static std::unordered_map<std::string, int> Vertical_alignment_type{
		{"CONSTANTGRADIENT", 1},	
		{"CIRCULARARC", 2},			
		{"PARABOLICARC", 3},		
		{"CLOTHOID", 4} //ToDo
	};				

	template <size_t N> using IfcAlignmentSegment = std::vector<IfcCurve<N>>;

	template <size_t N>
	struct IfcAlignment
	{
		IfcAlignmentSegment<N> Horizontal;
		IfcAlignmentSegment<N> Vertical;
	};
	
	//old geometry definitions below


		const double EXTRUSION_DISTANCE_HALFSPACE_M = 100;


		struct IfcSegmentIndexSelect
		{
			std::string type;
			std::vector<uint32_t> indexs;
		};

		struct Cylinder
		{
			bool Active = false;
			double Radius;
		};

		struct BSpline
		{
			bool Active = false;
			double UDegree;
			double VDegree;
			std::string ClosedU;
			std::string ClosedV;
			std::string CurveType;
			std::vector<std::vector<double>> Weights;
			std::vector<std::vector<glm::dvec3>> ControlPoints;
			std::vector<glm::f64> UMultiplicity;
			std::vector<glm::f64> VMultiplicity;
			std::vector<glm::f64> UKnots;
			std::vector<glm::f64> VKnots;
			std::vector<std::vector<glm::f64>> WeightPoints;
		};

		struct Revolution
		{
			bool Active = false;
			glm::dmat4 Direction;
			IfcProfileDef<3> Profile;
		};

		struct Extrusion
		{
			bool Active = false;
			glm::dvec3 Direction;
			IfcProfileDef<2> Profile;
			double Length;
		};


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

			std::array<double, 16> FlattenTransformation(const glm::dmat4 &transformation)
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

			bool testReverse()
			{
				glm::dvec3 cx = glm::dvec3(transformation[0].x, transformation[0].y, transformation[0].z);
				glm::dvec3 cy = glm::dvec3(transformation[1].x, transformation[1].y, transformation[1].z);
				glm::dvec3 cz = glm::dvec3(transformation[2].x, transformation[2].y, transformation[2].z);

				glm::dvec3 dx = glm::cross(cy, cz);
				glm::dvec3 dy = glm::cross(cx, cz);
				glm::dvec3 dz = glm::cross(cx, cy);

				double fac1 = glm::dot(cx, dx);
				double fac2 = -glm::dot(cy, dy);
				double fac3 = glm::dot(cz, dz);

				return fac1 * fac2 * fac3 < 0;
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

			std::optional<glm::dvec4> GetColor()
			{
				if (hasColor)
				{
					return color;
				}
				else
				{
					for (auto &c : children)
					{
						auto col = c.GetColor();
						if (col.has_value())
						{
							return col;
						}
					}
				}

				return std::nullopt;
			}
		};
		
		inline	glm::dmat4 NormalizeIFC(
			glm::dvec4(1, 0, 0, 0),
			glm::dvec4(0, 0, -1, 0),
			glm::dvec4(0, 1, 0, 0),
			glm::dvec4(0, 0, 0, 1));

		
		struct IfcSurface
		{
			glm::dmat4 transformation;
			BSpline BSplineSurface;
			Cylinder CylinderSurface;
			Revolution RevolutionSurface;
			Extrusion ExtrusionSurface;

			glm::dvec3 normal()
			{
				if (!CylinderSurface.Active && !BSplineSurface.Active && !RevolutionSurface.Active)
				{
					return transformation[2];
				}
				else
				{
					if (BSplineSurface.Active)
					{
						printf("Normal to bspline still not implemented\n");
					}
					if (CylinderSurface.Active)
					{
						printf("Normal to cylinder still not implemented\n");
					}
					if (RevolutionSurface.Active)
					{
						printf("Normal to revolution still not implemented\n");
					}
					return glm::dvec3(0);
				}
			}
		};
	}