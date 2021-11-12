#pragma once

#include <vector>
#include <algorithm>
#include <future>

#include "web-ifc.h"
#include "web-ifc-geometry.h"

const size_t MAX_JOB_SIZE = 30;

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
    void Start(const webifc::IfcLoader& loader, const std::vector<Job>& jobs, const std::function<void(const Job&, Progress)>& jobCallback)
    {
        int numWorkers = std::thread::hardware_concurrency();

        std::vector<webifc::IfcLoader> loaders(numWorkers);
        std::vector<std::unique_ptr<webifc::IfcGeometryLoader>> geomLoaders(numWorkers);
        std::vector<std::future<void>> futures(numWorkers);

        std::vector<int> completedJobs;
        std::mutex completedJobsMtx;

        int jobCounter = 0;
        size_t jobCompletedCounter = 0;
        size_t totalNumJobs = jobs.size();
        std::mutex jobMtx;

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
                                            int jobIndex = 0;
                                            jobMtx.lock();
                                            jobIndex = jobCounter++;
                                            jobMtx.unlock();
                                            if (jobIndex >= jobs.size())
                                            {
                                                break;
                                            }
                                            auto& job = jobs[jobIndex];
                                            
                                            geomLoaders[worker]->ClearCachedGeometry();
                                            for (auto& id : job.items)
                                            {
                                                geomLoaders[worker]->GetFlatMesh(id);
                                            }

                                            completedJobsMtx.lock();
                                            completedJobs.push_back(jobIndex);
                                            completedJobsMtx.unlock();
                                        }
                                    });
        }
        
        while (true)
        {
            int activeWorkers = 0;
            for (int i = 0; i < numWorkers; i++)
            {
                std::chrono::milliseconds span(0);
                if (futures[i].wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout)
                {
                    activeWorkers++;
                }
            }


            std::vector<int> jobsForCallback;
            {
                // copy over the completed jobs
                completedJobsMtx.lock();
                jobsForCallback = completedJobs;
                completedJobs.clear();
                completedJobsMtx.unlock();
            }

            if (jobsForCallback.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            else
            {
                for (auto& jobIndex : jobsForCallback)
                {
                    const Job& j = jobs[jobIndex];
                    Progress p;
                    p.completed = jobCompletedCounter;
                    p.total = totalNumJobs;
                    p.ratio = static_cast<double>(jobCompletedCounter) / totalNumJobs;
                    jobCallback(j, p);
                }
            }

            if (activeWorkers == 0)
            {
                // all jobs are done
                break;
            }
        }
    }
};