#ifndef THREADPOOL_HPP
#define THREADPOOL_HPP
 
#include <thread>
#include <mutex>
#include <future>
#include <iostream>
#include <random> 

template <typename JobType, typename ResultType>
class ThreadPool{
public:

    ThreadPool(std::function<ResultType(JobType)> func){
        m_procCount = std::thread::hardware_concurrency();
        m_processorFunction = func;
        m_terminate = false;

        //Create new worker threads
        for(uint32_t i = 0; i < m_procCount; i ++){
            m_workers.push_back(std::thread(&ThreadPool<JobType, ResultType>::worker, this));
        }
    }

    ~ThreadPool(){
        m_terminate = true;

        for(uint32_t i = 0; i < m_procCount; i ++){
            m_workers.at(i).join();
        }
    }

    std::future<ResultType> enqueue(JobType job){
        std::promise<ResultType> result_promise;
        std::pair<std::promise<ResultType>, JobType> promise_job_pair(result_promise, job);

        m_jobLock.lock();
        m_jobsQueue.push_back(promise_job_pair);
        m_jobLock.unlock();

        return result_promise.get_future();
    }

private:

    uint32_t m_procCount = 0;
    bool m_terminate = false;

    std::mutex m_jobLock;
    std::vector<std::pair<std::promise<ResultType>, JobType>> m_jobsQueue;
    std::vector<std::thread> m_workers;
    std::function<ResultType(JobType)> m_processorFunction = nullptr;

    void worker(){
        while(!m_terminate){
            
            bool hasJob = false;
            std::pair<std::promise<ResultType>, JobType> f_job_p;

            m_jobLock.lock();
            if(!m_jobsQueue.empty()){
                hasJob = true;
                f_job_p = std::move(m_jobsQueue.back());
                m_jobsQueue.pop_back();
            }else{
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            m_jobLock.unlock();
            if(!hasJob){continue;}
            ResultType res = m_processorFunction(f_job_p.second);
            f_job_p.first.set_value(res);
        }    
    }

};





#endif //THREADPOOL_HPP