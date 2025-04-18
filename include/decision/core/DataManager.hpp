#pragma once
#include <string>
#include <vector>
#include <any>
#include <functional>

namespace data_manager {
    enum DataType{
        DATA_TYPE_FLOAT = 0,
        DATA_TYPE_DOUBLE = 1,
        DATA_TYPE_INT = 2,
        DATA_TYPE_STRING = 3,
        DATA_TYPE_VECTOR = 4,
    };

    struct DataEntity
    {
        int dataId;
        int dataType;
        std::string dataName;
        std::vector<std::any> data;
        std::vector<double> unixtime;
    };

    class DataManager {
    public:
        DataManager();
        ~DataManager();

        // 注册数据类型
        int registerDataSchema(const std::string& dataType);

        // 注册数据输入器
        template<typename T>
        void registerDataIngestor(std::function<void(T)>);

        // 获取数据
        void fetchData(const int& dataEntityId);

    };

    DataManager::DataManager(){

    }

    DataManager::~DataManager(){

    }

    int DataManager::registerDataSchema(const std::string& dataType){
        return 0;
    }

    void DataManager::fetchData(const int& dataEntityId){

    }

    template<typename T>
    void DataManager::registerDataIngestor(std::function<void(T)>){

    }

}

