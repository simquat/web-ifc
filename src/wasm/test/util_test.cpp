
#include <stack>

#include "../deps/tinycpptest/TinyCppTest.hpp"
#include "../include/util.h"
#include "../include/math/triangulate-with-boundaries.h"
#include "../include/math/intersect-mesh-mesh.h"


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

TEST(BVHTest)
{
	int test_size = 1000;
	int numTests = 100;

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

		std::cout << collisionsBVH << std::endl;

		ASSERT_EQ(collisionsBoxes, collisionsBVH);
	}
}

TEST(TriangleWalkInnerEdge)
{
	glm::dvec2 a(0.0, 0.0);
	glm::dvec2 b(1.0, 0.0);
	glm::dvec2 c(0.5, 1.0);

	glm::dvec2 d(0.5, 0);
	glm::dvec2 e(0.5, 0.5);

	auto tris = triangulate(a, b, c, std::vector<glm::dvec2>{ d, e });

	ASSERT_EQ(tris.size(), 4);
}

TEST(TriangleWalkOuterEdge)
{
	glm::dvec2 a(0.0, 0.0);
	glm::dvec2 b(1.0, 0.0);
	glm::dvec2 c(0.0, 1.0);

	glm::dvec2 d(0.25, 0);

	auto tris = triangulate(a, b, c, std::vector<glm::dvec2>{ d });

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

	auto tris = triangulate(a, b, c, std::vector<glm::dvec2>{ d });

	ASSERT_EQ(tris.size(), 3);
}

TEST(NewTest)
{
	// load model
	// get mesh for line
	// compute volume
	// check volume
}

