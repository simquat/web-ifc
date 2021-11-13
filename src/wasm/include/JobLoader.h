#pragma once

#include <vector>
#include <algorithm>
#include <future>

#include "web-ifc.h"
#include "web-ifc-geometry.h"

const size_t MAX_JOB_SIZE = 32;

struct Job
{
    std::vector<uint32_t> items;

    Job(size_t s) :
        items(s)
    {

    }
};

struct Progress
{
    size_t completed;
    size_t total;
    double ratio;
};

std::vector<Job> GetJobs(webifc::IfcLoader& loader)
{
    std::vector<Job> jobs;

    for (auto type : ifc2x4::IfcElements)
    {
        auto elements = loader.GetExpressIDsWithType(type);
        if (!elements.empty())
        {
            for (size_t offset = 0; offset < elements.size(); offset += MAX_JOB_SIZE)
            {
                size_t count = std::min(offset + MAX_JOB_SIZE, elements.size()) - offset;

                Job job(count);

                for (size_t i = 0; i < count; i++)
                {
                    job.items[i] = elements[offset + i];
                }

                jobs.push_back(std::move(job));
            }
        }
    }

    return jobs;
}

class JobLoader
{

public:

    struct TransferrableGeometry
    {
        glm::dvec4 color;
        std::array<double, 16> flatTransformation;
        uint32_t geometryExpressID;
        uint32_t v_offset;
        uint32_t v_size;
        uint32_t i_offset;
        uint32_t i_size;
    };

    struct TransferrableMesh
    {
        uint32_t expressID;
        std::vector<TransferrableGeometry> geometries;
    };

    struct TransferBuffer
    {
        std::vector<float> vertexData;
        std::vector<uint32_t> indexData;
        std::vector<TransferrableMesh> meshes;

        void Reset()
        {
            vertexData.clear();
            indexData.clear();
            meshes.clear();
        }
    };

    struct CompletedJob
    {
        int jobIndex;
        std::unique_ptr<TransferBuffer> buffer;
    };

    void Start(const webifc::IfcLoader& loader, const std::vector<Job>& jobs, const std::function<void(const Job&, const TransferBuffer&, Progress)>& jobCallback)
    {
        int numWorkers = std::thread::hardware_concurrency();

        std::vector<webifc::IfcLoader> loaders(numWorkers);
        std::vector<std::unique_ptr<webifc::IfcGeometryLoader>> geomLoaders(numWorkers);
        std::vector<std::future<void>> futures(numWorkers);
        std::vector<std::unique_ptr<TransferBuffer>> transferBuffers;
        std::mutex transferBufferMutex;

        std::vector<CompletedJob> completedJobs;
        std::mutex completedJobsMtx;

        int jobCounter = 0;
        size_t jobCompletedCounter = 0;
        size_t totalNumJobs = jobs.size();
        std::mutex jobMtx;

        size_t maxPendingJobs = 0;

        for (int i = 0; i < numWorkers; i++)
        {
            // read only copy of the original loader
            loaders[i] = webifc::IfcLoader(loader);
            geomLoaders[i] = std::make_unique<webifc::IfcGeometryLoader>(loaders[i]);

            int worker = i;

            futures[i] = std::async([&, worker]()
                                    {
                                        while (true)
                                        {
                                            // get job
                                            int jobIndex = 0;
                                            jobMtx.lock();
                                            jobIndex = jobCounter++;
                                            jobMtx.unlock();
                                            if (jobIndex >= jobs.size())
                                            {
                                                break;
                                            }
                                            auto& job = jobs[jobIndex];

                                            auto& geomLoader = geomLoaders[worker];

                                            std::unique_ptr<TransferBuffer> transferBuffer;
                                            transferBufferMutex.lock();
                                            if (!transferBuffers.empty())
                                            {
                                                transferBuffer = std::move(transferBuffers.back());
                                                transferBuffers.pop_back();
                                            }
                                            transferBufferMutex.unlock();

                                            if (!transferBuffer)
                                            {
                                                transferBuffer = std::make_unique<TransferBuffer>();
                                            }

                                            // work on job
                                            geomLoader->ClearCachedGeometry();
                                            size_t totalISize = 0;
                                            size_t totalVSize = 0;
                                            for (auto& id : job.items)
                                            {
                                                auto flatMesh = geomLoader->GetFlatMesh(id);
                                                TransferrableMesh tmesh;
                                                tmesh.expressID = flatMesh.expressID;
                                                for (auto& geom : flatMesh.geometries)
                                                {
                                                    TransferrableGeometry tgeom;
                                                    tgeom.color = geom.color;
                                                    tgeom.flatTransformation = webifc::FlattenTransformation(geom.transformation);
                                                    tgeom.geometryExpressID = geom.geometryExpressID;

                                                    auto& cachedGeom = geomLoader->GetCachedGeometry(tgeom.geometryExpressID);
                                                 
                                                    totalISize += cachedGeom.indexData.size();
                                                    totalVSize += cachedGeom.vertexData.size();

                                                    tmesh.geometries.push_back(tgeom);
                                                }
                                                transferBuffer->meshes.push_back(tmesh);
                                            }

                                            transferBuffer->indexData.reserve(totalISize);
                                            transferBuffer->vertexData.reserve(totalVSize);

                                            for (auto& tmesh : transferBuffer->meshes)
                                            {
                                                for (auto& tgeom : tmesh.geometries)
                                                {
                                                    // copy geom to transferbuffer
                                                    auto& cachedGeom = geomLoader->GetCachedGeometry(tgeom.geometryExpressID);

                                                    tgeom.i_offset = transferBuffer->indexData.size();
                                                    tgeom.i_size = cachedGeom.indexData.size();
                                                    tgeom.v_offset = transferBuffer->vertexData.size();
                                                    tgeom.v_size = cachedGeom.vertexData.size();

                                                    for (int i = 0; i < cachedGeom.indexData.size(); i++)
                                                    {
                                                        transferBuffer->indexData.push_back(cachedGeom.indexData[i]);
                                                    }

                                                    for (int i = 0; i < cachedGeom.vertexData.size(); i++)
                                                    {
                                                        transferBuffer->vertexData.push_back(cachedGeom.vertexData[i]);
                                                    }
                                                }
                                            }

                                            CompletedJob completedJob;
                                            completedJob.jobIndex = jobIndex;
                                            completedJob.buffer = std::move(transferBuffer);

                                            // signal job done
                                            completedJobsMtx.lock();
                                            completedJobs.push_back(std::move(completedJob));
                                            completedJobsMtx.unlock();
                                        }
                                    });
        }
        
        while (true)
        {
            std::vector<CompletedJob> jobsForCallback;
            {
                // copy over the completed jobs
                completedJobsMtx.lock();
                jobsForCallback = std::move(completedJobs);
                completedJobsMtx.unlock();
            }

            jobCompletedCounter += jobsForCallback.size();

            maxPendingJobs = std::max(maxPendingJobs, jobsForCallback.size());

            if (jobsForCallback.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            else
            {
                for (auto& completedJob : jobsForCallback)
                {
                    const Job& j = jobs[completedJob.jobIndex];

                    Progress progress;
                    progress.completed = jobCompletedCounter;
                    progress.total = totalNumJobs;
                    progress.ratio = static_cast<double>(jobCompletedCounter) / totalNumJobs;

                    jobCallback(j, *completedJob.buffer.get(), progress);

                    completedJob.buffer->Reset();

                    transferBufferMutex.lock();
                    transferBuffers.push_back(std::move(completedJob.buffer));
                    transferBufferMutex.unlock();
                }
            }

            if (jobCompletedCounter == jobs.size())
            {
                // all jobs are done
                break;
            }
        }

        std::cout << "Max pending: " << maxPendingJobs << std::endl;
    }
};