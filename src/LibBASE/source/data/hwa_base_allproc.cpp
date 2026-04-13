#include "hwa_base_allproc.h"

namespace hwa_base
{
    // construct
     base_all_proc:: base_all_proc() : base_data()
    {
    }

     base_all_proc:: base_all_proc(base_log spdlog) : base_data(spdlog)
    {
    }
    // copy construct
     base_all_proc:: base_all_proc(const  base_all_proc &Other) : base_data(Other.spdlog())
    {
        this->_mapData = Other._mapData;
    }

     base_all_proc::~ base_all_proc()
    {
    }

    // assign function
    void  base_all_proc::operator=(const  base_all_proc &Other)
    {
        this->_mapData = Other._mapData;
    }

    void  base_all_proc::Add_Data(const std::string &type, base_data *data)
    {
        if (data)
        {
            _mapData[data->id_type()] = data;
        }
        else
        {
            SPDLOG_LOGGER_CRITICAL(_spdlog, "your {} is nullptr", type);
            throw std::logic_error("base_data pointer is nullptr!");
        }
    }

    base_data * base_all_proc::operator[](base_data::ID_TYPE type)
    {
        if (_mapData.find(type) == _mapData.end())
        {
            return nullptr;
        }
        return _mapData[type];
    }

    base_data * base_all_proc::operator[](base_data::ID_GROUP group)
    {
        for (auto data_iter = _mapData.begin(); data_iter != _mapData.end(); data_iter++)
        {
            if (data_iter->second->id_group() == group)
            {
                return data_iter->second;
            }
        }
        return nullptr;
    }
}