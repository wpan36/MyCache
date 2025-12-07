#pragma once

namespace myCache{

template <typename Key, typename Value>
class CachePolicy{
public:
    virtual ~CachePolicy(){}
    virtual void put(Key k, Value v) = 0;
    virtual Value get(Key k) = 0;
    virtual bool get(Key k, Value& v) = 0;
};

}