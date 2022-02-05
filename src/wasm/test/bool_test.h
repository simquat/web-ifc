
#pragma once

#include <stack>
#include <iostream>

#include "../deps/tinycpptest/TinyCppTest.hpp"
#include "../include/util.h"
#include "../include/web-ifc.h"
#include "../include/web-ifc-geometry.h"


template<typename T, typename R>
bool E_EQ(std::string message, T t, R r)
{
    if (t != r)
    {
        std::cout << message << " Expected: " << t << " got: " << r << std::endl;

        return false;
    }

    return true;
}

struct BoolTestOptions
{
    bool dumpOutputMesh = true;
    std::wstring testDir;
    webifc::LoaderSettings loaderSettings;
};

struct BoolOutput
{
    webifc::IfcGeometry geometry;
    bool result = true;

    BoolOutput& ExpectedFaces(uint32_t numFaces)
    {
        result = result && E_EQ("Number of faces", numFaces, geometry.numFaces);

        return *this;
    }

    bool operator()()
    {
        return result;
    }
};

BoolOutput TestFile(BoolTestOptions testOptions, std::wstring filename, uint32_t expressID)
{
    std::wstring absFilename = testOptions.testDir + filename;

    webifc::IfcLoader loader(testOptions.loaderSettings);
    loader.LoadFile(absFilename);

    webifc::IfcGeometryLoader geomLoader(loader);

    BoolOutput output;
    
    output.geometry = geomLoader.GetFlattenedGeometry(expressID);

    if (testOptions.dumpOutputMesh)
    {
        DumpIfcGeometryToPath(output.geometry, absFilename + L".obj", 1.0 / 1000);
    }

    return output;
}

TEST(BoolTests)
{
    BoolTestOptions testOptions;
    {
        testOptions.testDir = L"../../../testdata/bools/";
        testOptions.dumpOutputMesh = true;
    
        webifc::LoaderSettings loaderSettings;
        loaderSettings.COORDINATE_TO_ORIGIN = true;
        loaderSettings.USE_FAST_BOOLS = true;
        loaderSettings.DUMP_CSG_MESHES = false;

        testOptions.loaderSettings = loaderSettings;
    }

    ASSERT(TestFile(testOptions, L"1_many_coplanar_subtractions.ifc", 1066540).ExpectedFaces(902)());
}