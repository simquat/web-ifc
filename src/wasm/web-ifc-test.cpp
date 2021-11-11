/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */
 
#include <iostream>
#include <fstream>
#include <filesystem>
#include <numeric>
#include <algorithm>
#include <future>

#include "include/web-ifc.h"
#include "include/web-ifc-geometry.h"
#include "include/math/triangulate-with-boundaries.h"
#include "include/ifc2x4.h"

std::string ReadFile(std::wstring filename)
{
    std::ifstream t(filename);
    std::stringstream buffer;
    buffer << t.rdbuf();
    return buffer.str();
}

void SpecificLoadTest(webifc::IfcLoader& loader, webifc::IfcGeometryLoader& geometryLoader, uint64_t num)
{
    auto walls = loader.GetExpressIDsWithType(ifc2x4::IFCSLAB);

    bool writeFiles = true;
    
    auto mesh = geometryLoader.GetMesh(num);

    if (writeFiles)
    {
        geometryLoader.DumpMesh(mesh, L"TEST.obj");
    }
}

std::vector<webifc::IfcFlatMesh> LoadAllTest(webifc::IfcLoader& loader, webifc::IfcGeometryLoader& geometryLoader)
{
    std::vector<webifc::IfcFlatMesh> meshes;

    for (auto type : ifc2x4::IfcElements)
    {
        auto elements = loader.GetExpressIDsWithType(type);

        for (int i = 0; i < elements.size(); i++)
        {
            auto mesh = geometryLoader.GetFlatMesh(elements[i]);

            /*
            for (auto& geom : mesh.geometries)
            {
                if (!geometryLoader.HasCachedGeometry(geom.geometryExpressID))
                {
                    printf("asdf");
                }
                auto flatGeom = geometryLoader.GetCachedGeometry(geom.geometryExpressID);
                flatGeom.GetVertexData();
            }
            */

            meshes.push_back(mesh);
        }
    }

    return meshes;
}

std::vector<std::vector<uint32_t>> GetJobs(webifc::IfcLoader& loader)
{
    int MAX_JOB_SIZE = 100;

    std::vector<std::vector<uint32_t>> jobs;

    for (auto type : ifc2x4::IfcElements)
    {
        auto elements = loader.GetExpressIDsWithType(type);
        if (!elements.empty())
        {
            for (int offset = 0; offset < elements.size(); offset += MAX_JOB_SIZE)
            {
                int count = std::min(offset + MAX_JOB_SIZE, (int)elements.size()) - offset;
                std::vector<uint32_t> job(count);
                for (int i = 0; i < count; i++)
                {
                    job[i] = elements[offset + i];
                }
                jobs.push_back(std::move(job));
            }
        }
    }

    return jobs;
}

std::vector<webifc::IfcFlatMesh> LoadAllMTTest(webifc::IfcLoader& loader)
{
    int numWorkers = 8;

    std::vector<webifc::IfcLoader> loaders(numWorkers);
    std::vector<std::unique_ptr<webifc::IfcGeometryLoader>> geomLoaders(numWorkers);
    std::vector<std::future<void>> futures(numWorkers);

    auto jobs = GetJobs(loader);
    int jobCounter = 0;
    std::mutex jobMtx;

    std::cout << jobs.size() << "jobs" << std::endl;

    for (auto& job : jobs)
    {
        //std::cout << job.size() << std::endl;
    }

    for (int i = 0; i < numWorkers; i++)
    {
        // read only copy of the original loader
        loaders[i] = webifc::IfcLoader(loader);
        geomLoaders[i] = std::make_unique<webifc::IfcGeometryLoader>(loaders[i]);

        int worker = i;

        futures[i] = std::async([&geomLoaders, &jobMtx, &jobCounter, &jobs, worker]()
                                {
                                    while (true)
                                    {
                                        int jobIndex = 0;
                                        jobMtx.lock();
                                        jobIndex = jobCounter++;
                                        jobMtx.unlock();
                                        if (jobIndex >= jobs.size())
                                        {
                                            break;
                                        }
                                        auto& job = jobs[jobIndex];

                                        for (auto& id : job)
                                        {
                                            geomLoaders[worker]->GetFlatMesh(id);
                                        }
                                    }
                               });
    }


    for (int i = 0; i < numWorkers; i++)
    {
        futures[i].get();
    }

    return {};
}

void DumpRefs(std::unordered_map<uint32_t, std::vector<uint32_t>>& refs)
{
    std::ofstream of("refs.txt");

    int32_t prev = 0;
    for (auto& it : refs)
    {
        if (!it.second.empty())
        {
            for (auto& i : it.second)
            {
                of << (((int32_t)i) - (prev));
                prev = i;
            }
        }
    }
}

struct BenchMarkResult
{
    std::string file;
    long long timeMS;
    long long sizeBytes;
};

void Benchmark()
{
    std::vector<BenchMarkResult> results;
    std::string path = "../../../benchmark/ifcfiles";
    for (const auto& entry : std::filesystem::directory_iterator(path))
    {
        if (entry.path().extension().string() != ".ifc")
        {
            continue;
        }
        
        std::wstring filePath = entry.path().wstring();
        std::string filename = entry.path().filename().string();

        std::string content = ReadFile(filePath);

        webifc::IfcLoader loader;
        auto start = webifc::ms();
        {
            loader.LoadFile(content);
        }
        auto time = webifc::ms() - start;

        BenchMarkResult result;
        result.file = filename;
        result.timeMS = time;
        result.sizeBytes = entry.file_size();
        results.push_back(result);
        
        std::cout << "Reading " << result.file << " took " << time << "ms" << std::endl;
    }

    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "Results:" << std::endl;
    
    double avgMBsec = 0;
    for (auto& result : results)
    {
        double MBsec = result.sizeBytes / 1000.0 / result.timeMS;
        avgMBsec += MBsec;
        std::cout << result.file << ": " << MBsec << " MB/sec" << std::endl;
    }

    avgMBsec /= results.size();

    std::cout << std::endl;
    std::cout << "Average: " << avgMBsec << " MB/sec" << std::endl;

    std::cout << std::endl;
    std::cout << std::endl;

}

void TestTriangleDecompose()
{
    const int NUM_TESTS = 100;
    const int PTS_PER_TEST = 100;
    const int EDGE_PTS_PER_TEST = 10;

    const double scaleX = 650;
    const double scaleY = 1;

    glm::dvec2 a(0, 0);
    glm::dvec2 b(scaleX, 0);
    glm::dvec2 c(0, scaleY);

    for (int i = 0; i < NUM_TESTS; i++)
    {
        srand(i);

        std::vector<glm::dvec2> points;

        // random points
        for (int j = 0; j < PTS_PER_TEST; j++)
        {
            points.push_back({
                webifc::RandomDouble(0, scaleX),
                webifc::RandomDouble(0, scaleY)
            });
        }

        // points along the edges
        for (int j = 0; j < EDGE_PTS_PER_TEST; j++)
        {
            glm::dvec2 e1 = b - a;
            glm::dvec2 e2 = c - a;
            glm::dvec2 e3 = b - c;

            points.push_back(a + e1 * webifc::RandomDouble(0, 1));
            points.push_back(a + e2 * webifc::RandomDouble(0, 1));
            points.push_back(c + e3 * webifc::RandomDouble(0, 1));
        }

        std::vector<webifc::Loop> loops;

        for (auto& pt : points)
        {
            //if (pt.x > scaleX / 2)
            {
                webifc::Loop l;
                l.hasOne = true;
                l.v1 = pt;
                loops.push_back(l);
            }
        }

        std::cout << "Start test " << i << std::endl;

        bool swapped = false;
        webifc::TriangulateBoundaries tribound;
        auto triangles = tribound.triangulate(a, b, c, loops, swapped);

        // webifc::IsValidTriangulation(triangles, points);

        std::vector<webifc::Point> pts;

        for (auto& pt : points)
        {
            webifc::Point p;
            p.x = pt.x;
            p.y = pt.y;
            pts.push_back(p);
        }

        webifc::DumpSVGTriangles(triangles, webifc::Point(), webifc::Point(), L"triangles.svg", pts);
    }
}

int main()
{
    std::cout << "Hello web IFC test!\n";

    // TestTriangleDecompose();

    // return 0;

    //Benchmark();

    //return 0;


    std::string content = ReadFile(L"D:/web-ifc-obb/benchmark/ifcfiles/20200518Yangsan Pr-HARDWARE.ifc");
    //std::string content = ReadFile(L"D:/web-ifc-obb/benchmark/ifcfiles/rac_basic_sample_project.ifc");
    //std::string content = ReadFile(L"D:/web-ifc/src/wasm/build/output.ifc");

    webifc::LoaderSettings set;
    set.COORDINATE_TO_ORIGIN = true;
    set.DUMP_CSG_MESHES = false;
    set.USE_FAST_BOOLS = true;

    webifc::IfcLoader loader(set);


    auto start = webifc::ms();
    loader.LoadFile(content);

    //std::ofstream outputStream(L"D:/web-ifc/benchmark/ifcfiles/output.ifc");
    //outputStream << loader.DumpAsIFC();
    //exit(0);
    auto time = webifc::ms() - start;

    std::cout << "Reading took " << time << "ms" << std::endl;


    /*
    std::ofstream outputFile("output.ifc");
    outputFile << loader.DumpSingleObjectAsIFC(120119);
    outputFile.close();
    */

    webifc::IfcGeometryLoader geometryLoader(loader);

    start = webifc::ms();

    //SpecificLoadTest(loader, geometryLoader, 2615);
    LoadAllMTTest(loader);
    //auto meshes = LoadAllTest(loader, geometryLoader);
    auto trans = webifc::FlattenTransformation(geometryLoader.GetCoordinationMatrix());

    auto errors = loader.GetAndClearErrors();

    for (auto error : errors)
    {
        std::cout << error.expressID << " " << error.ifcType << " " << std::to_string((int)error.type) << " " << error.message << std::endl;
    }

    time = webifc::ms() - start;

    std::cout << "Generating geometry took " << time << "ms" << std::endl;

    std::cout << "Done" << std::endl;
}
