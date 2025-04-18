#pragma once
#include <vector>
#include <queue>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <algorithm>

// 线程将交替执行任务，如果是循环任务，则将任务重新加入队列
// 如果非循环任务，则执行一次后，任务将被移除

namespace thread_pool{
    struct Task{
        std::string name;
        std::function<void()> func;
        bool loop;
        double interval;
        double last_time;
        bool operator==(const Task& other) const {
        return name == other.name;
        }
    };


    class ThreadPool {
    public:
        ThreadPool(size_t num_threads) : stop(false) {
            for (size_t i = 0; i < num_threads; ++i) {
                workers.emplace_back([this] {
                    while (true) {
                        std::function<void()> func;
                        thread_pool::Task task;
                        {
                            std::unique_lock<std::mutex> lock(queue_mutex);
                            cv.wait(lock, [this] { return stop || !tasks.empty(); });
                            if (stop) return;
                            task = std::move(tasks.front());
                            tasks.erase(tasks.begin());
                            func = task.func;
                        }
                        if (task.loop && get_current_time() - task.last_time > task.interval) func(); 
                        if (task.loop){
                            task.last_time = get_current_time();
                            this->enqueue(task);
                        }
                    }
                });
            }
        }

        void enqueue(thread_pool::Task task) {
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                tasks.emplace_back(std::move(task));
            }
            cv.notify_one();
        }

        bool task_exist(const std::string& name){
            std::lock_guard<std::mutex> lock(queue_mutex);
            for (auto& task : tasks) {
                if (task.name == name) return true;
            }
            return false;
        }

        bool remove_task(const std::string& name){
            std::lock_guard<std::mutex> lock(queue_mutex);
            for (auto& task : tasks) {
                if (task.name == name) {
                    tasks.erase(std::remove(tasks.begin(), tasks.end(), task), tasks.end());
                    return true;
                }
            }
            return false;
        }
        
        double get_current_time(){
            return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        }

        ~ThreadPool() {
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                stop = true;
            }
            cv.notify_all();
            for (auto& worker : workers) {
                worker.join();
            }
        }

    private:
        std::vector<std::thread> workers;
        std::vector<Task> tasks;
        std::mutex queue_mutex;
        std::condition_variable cv;
        bool stop;
    };
}
