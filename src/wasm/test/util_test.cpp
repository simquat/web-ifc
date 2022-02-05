
#include <stack>

#include "../deps/tinycpptest/TinyCppTest.hpp"
#include "../include/util.h"
#include "../include/math/triangulate-with-boundaries.h"
#include "../include/math/intersect-mesh-mesh.h"

#include "bool_test.h"

using namespace webifc;

void AddRandomTri(webifc::IfcGeometry& geom, double sceneSize, double triSize)
{
	glm::dvec3 a(webifc::RandomDouble(0, sceneSize),
			     webifc::RandomDouble(0, sceneSize),
			     webifc::RandomDouble(0, sceneSize));

	glm::dvec3 b(webifc::RandomDouble(triSize / 4, triSize),
				 webifc::RandomDouble(triSize / 4, triSize),
				 webifc::RandomDouble(triSize / 4, triSize));

	glm::dvec3 c(webifc::RandomDouble(triSize / 4, triSize),
				 webifc::RandomDouble(triSize / 4, triSize),
				 webifc::RandomDouble(triSize / 4, triSize));

	geom.AddFace(a, a + b, a + c);
}

webifc::IfcGeometry GetRandomMesh(int size)
{
	webifc::IfcGeometry geom;

	for (int i = 0; i < size; i++)
	{
		AddRandomTri(geom, 10, 0.5);
	}

	return geom;
}

TEST(ClipSegments2D_Angle)
{
	/*
	 _____
	|   _| vs ______
	|__|
	*/

	// clockwise, expect lines (0, 1) (1, 2)
	std::vector<glm::dvec2> points ={
		{ 0, -1 },
		{ 0, 1 },
		{ 2, 1 },
		{ 2, 0 },
		{ 1, 0 },
		{ 1, -1 }
	};

	// line defaults to pos: (0,0) dir: (1, 0)
	auto segs = LineDistancesToClipSegments2D(points);

	// expect three segments
	ASSERT(segs.size() == 4);

	// flip dir to -1
	segs = LineDistancesToClipSegments2D(points, glm::dvec2(2, 0), glm::dvec2(-1, 0));

	// expect three segments
	ASSERT(segs.size() == 4);
}

TEST(ClipSegments2D_Slot)
{
	/*
	 ____   ____
	|	 |_|	| vs ______
	|___________|
	*/

	// clockwise, expect lines (0, 2) (2, 3) (3, 5)
	std::vector<glm::dvec2> points = {
		{ 0, -1 },
		{ 0, 1 },
		{ 2, 1 },
		{ 2, 0 },
		{ 3, 0 },
		{ 3, 1 },
		{ 5, 1 },
		{ 5, -1 }
	};

	// line defaults to pos: (0,0) dir: (1, 0)
	auto segs = LineDistancesToClipSegments2D(points);

	// expect three segments
	ASSERT(segs.size() == 6);

	// flip dir to -1
	segs = LineDistancesToClipSegments2D(points, glm::dvec2(0, 0), glm::dvec2(-1, 0));

	// expect three segments
	ASSERT(segs.size() == 6);
}

TEST(ComparableAngleCheck)
{
	glm::dvec2 a(0,0);
	glm::dvec2 b(1,0);

	glm::dvec2 left(1, -1);
	glm::dvec2 right(1, 1);
	glm::dvec2 sharpRight(0, 1);
	glm::dvec2 sharpLeft(0, -1);

	double leftS = ComparableAngle(left, a, b);
	double rightS = ComparableAngle(right, a, b);
	double sharpRightS = ComparableAngle(sharpRight, a, b);
	double sharpLeftS = ComparableAngle(sharpLeft, a, b);

	ASSERT(leftS < 0);
	ASSERT(rightS > 0);
	ASSERT(sharpRightS > rightS);
	ASSERT(sharpLeftS < leftS);
}

/*
TEST(BVHTest)
{
	int test_size = 1000;
	int numTests = 10;

	srand(6788);

	for (int i = 0; i < numTests; i++)
	{
		auto m1 = GetRandomMesh(test_size);
		auto m2 = GetRandomMesh(test_size);

		auto bvh1 = MakeBVH(m1);
		auto bvh2 = MakeBVH(m2);

		int collisionsBoxes = 0;
		for (auto& b1 : bvh1.boxes)
		{
			for (auto& b2 : bvh2.boxes)
			{
				if (b1.intersects(b2))
				{
					collisionsBoxes++;
				}
			}
		}

		int collisionsBVH = 0;

		bvh1.Intersect(bvh2, [&](int i, int j)
					   {
						   collisionsBVH++;
					   });

		//std::cout << collisionsBVH << std::endl;

		ASSERT_EQ(collisionsBoxes, collisionsBVH);
	}
}
*/

TEST(TriangleWalkInnerEdge)
{
	glm::dvec2 a(0.0, 0.0);
	glm::dvec2 b(1.0, 0.0);
	glm::dvec2 c(0.5, 1.0);

	glm::dvec2 d(0.5, 0);
	glm::dvec2 e(0.5, 0.5);

	webifc::TriangulateWithBoundaries twb;
	auto tris = twb.triangulate(a, b, c, std::vector<glm::dvec2>{ d, e });

	ASSERT_EQ(tris.size(), 4);
}

TEST(TriangleWalkOuterEdge)
{
	glm::dvec2 a(0.0, 0.0);
	glm::dvec2 b(1.0, 0.0);
	glm::dvec2 c(0.0, 1.0);

	glm::dvec2 d(0.25, 0);

	webifc::TriangulateWithBoundaries twb;
	auto tris = twb.triangulate(a, b, c, std::vector<glm::dvec2>{ d });

	ASSERT_EQ(tris.size(), 2);
}

TEST (TriangleAreaTest)
{
	glm::dvec3 a (0.0, 0.0, 0.0);
	glm::dvec3 b (1.0, 0.0, 0.0);
	glm::dvec3 c (0.0, 1.0, 0.0);
	ASSERT_EQ_EPS (areaOfTriangle (a, b, c), 0.5, EPS_TINY);
}

TEST(TriangleWalkCenter)
{
	glm::dvec2 a(0.0, 0.0);
	glm::dvec2 b(1.0, 0.0);
	glm::dvec2 c(0.0, 1.0);

	glm::dvec2 d(0.25, 0.25);

	webifc::TriangulateWithBoundaries twb;
	auto tris = twb.triangulate(a, b, c, std::vector<glm::dvec2>{ d });

	ASSERT_EQ(tris.size(), 3);
}

TEST(NewTest)
{
	// load model
	// get mesh for line
	// compute volume
	// check volume
}

