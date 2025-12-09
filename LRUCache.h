#pragma once

#include <unordered_map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <cmath>

#include "CachePolicy.h"

namespace myCache{

template<typename Key, typename Value> class LRUCache;

template<typename Key, typename Value>
class LRUNode{
    friend class LRUCache<Key, Value>;
private:
    Key key_;
    Value value_;
    std::weak_ptr<LRUNode<Key, Value>> prev_;
    std::shared_ptr<LRUNode<Key, Value>> next_;

public:
    LRUNode(const Key& k, const Value& v)
        : key_(k)
        , value_(v)
    {}
};

// LRUCache class
template<typename Key, typename Value>
class LRUCache : public CachePolicy<Key, Value>{

public:
    using LRUNodeType = LRUNode<Key, Value>;
    using NodePtr = std::shared_ptr<LRUNode<Key, Value>>;
    using NodeMap = std::unordered_map<Key, NodePtr>;

    LRUCache(int capacity) : capacity_(capacity){
        dummyHead_ = std::make_shared<LRUNodeType>(Key(), Value());
        dummyTail_ = std::make_shared<LRUNodeType>(Key(), Value());
        dummyHead_->next_ = dummyTail_;
        dummyTail_->prev_ = dummyHead_;
    }
    ~LRUCache() override = default;

    void put(const Key& key, const Value& value) override{
        if (capacity_ <= 0) return;
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = keyToNode_.find(key);
        if (it == keyToNode_.end()){   
            addNewNode(key, value);       
        }else{
            it->second->value_ = value;
            refreshExistingNode(it->second);
        }
    }

    bool get(Key k, Value& v) override{
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = keyToNode_.find(k);
        if (it == keyToNode_.end()){
            return false;
        }else{
            v = it->second->value_;
            refreshExistingNode(it->second);
            return true;
        }
    }
    
    Value get(Key k) override{
        Value value{};
        get(k, value);
        return value;
    }

    void remove(Key key){
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = keyToNode_.find(key);
        if (it != keyToNode_.end()){
            removeNodeFromList(it->second);
            keyToNode_.erase(key);
        }
    }

    int getCapacity(){
        return capacity_;
    }

private:
    NodePtr dummyHead_;
    NodePtr dummyTail_;
    int capacity_;
    NodeMap keyToNode_;
    std::mutex mutex_;

    void removeOneNodeFromLRU(){
        if (keyToNode_.size() == 0) return;
        NodePtr nodeToRemove = dummyTail_->prev_.lock();
        removeNodeFromList(nodeToRemove);
        keyToNode_.erase(nodeToRemove->key_);
    }

    void removeNodeFromList(NodePtr node){
        if (!node->prev_.expired() && node->next_){
            node->prev_.lock()->next_ = node->next_;
            node->next_->prev_ = node->prev_;
            node->next_.reset();
            node->prev_.reset();
        }
    }

    void insertNodeToList(NodePtr node){
        node->next_ = dummyHead_->next_;
        node->prev_ = dummyHead_;
        dummyHead_->next_->prev_ = node;
        dummyHead_->next_ = node;
    }

    void refreshExistingNode(NodePtr nodeToRefresh){
        removeNodeFromList(nodeToRefresh);
        insertNodeToList(nodeToRefresh);
    }

    void addNewNode(const Key& key, const Value& value){
        if (keyToNode_.size() >= capacity_){
            removeOneNodeFromLRU();
        }
        NodePtr newNodePtr = std::make_shared<LRUNodeType>(key, value);
        keyToNode_[key] = newNodePtr;
        insertNodeToList(newNodePtr);
    }

};

template<typename Key, typename Value>
class LRUKCache : public LRUCache<Key, Value>{
public:
    LRUKCache(int capacity, int k, int historySize)
    : LRUCache<Key, Value>(capacity)
    , k_(k)
    , historyRecord_(std::make_unique<LRUCache<Key, std::pair<int, std::optional<Value>>>>(historySize))
    {}
    ~LRUKCache() = default;

    bool get(Key key, Value& value) override{
        std::lock_guard<std::mutex> lock(mutexK_);
        bool inMainLRU = LRUCache::get(key, value);
        if (inMainLRU){
            return true;
        }

        //not in main LRU, update history list
        //First check if the element is in history list and update access count
        std::pair<int, std::optional<Value>> p;
        bool inHistoryRecord = historyRecord_->get(key, p);
        if (!inHistoryRecord){
            p = {1, std::nullopt};
            historyRecord_->put(key, {1, std::nullopt});
        }else{
            p.first++;
            historyRecord_->put(key, p);
        }

        //Then check if we need to add it to main LRU
        if (p.first >= k_s){
            //add new elemnt to main LRU and delete from history list
            //if we can find the value in history map, add to main LRU
            std::pair<int, std::optional<Value>> tempPair;
            historyRecord_->get(key, tempPair);
            if (tempPair.second){
                LRUCache::put(key, *(tempPair.second));

                //delete from history list
                historyRecord_->remove(key);

                value = *(tempPair.second);
                return true;
            }
        }
        value = Value{};
        return false;
    }

    Value get(Key key) override{
        Value value{};
        get(key, value);
        return value;
    }

    void put(Key key, Value value) override{
        std::lock_guard<std::mutex> lock(mutexK_);
        Value tempVal{};
        bool inMainLRU = LRUCache::get(key, tempVal);
        if (inMainLRU){
            LRUCache::put(key, value);
            return;
        }

        //not in main LRU, put to history list
        std::pair<int, std::optional<Value>> p;
        bool inHistory = historyRecord_->get(key, p);
        if (inHistory){
            p.first++;
            p.second = value;
            historyRecord_->put(key, p);
        }else{
            p = {1, value};
            historyRecord_->put(key, p);
        }

        if (p.first >= k_){
            LRUCache::put(key, value);
            historyRecord_->remove(key);
        }
    }

private:
    int k_;
    std::mutex mutexK_;
    std::unique_ptr<LRUCache<Key, std::pair<int, std::optional<Value>>>> historyRecord_; // key to {visit numbers, value}
};

template<typename Key, typename Value>
class HashLRUCache : public CachePolicy<Key, Value>{
public:
    HashLRUCache(int capacity, int shardNum){
        if (shardNum > 0){
            shardNum_ = shardNum;
        }else{
            unsigned int hc = std::thread::hardware_concurrency();
            shardNum_ = hc > 0 ? static_cast<int>(hc) : 1;
        }
        size_t shardSize = std::ceil(capacity / static_cast<double>(shardNum_));
        for (int i = 0; i < shardNum_; i++){
            sliceCache_.emplace_back(std::make_unique<LRUCache<Key, Value>>(shardSize));
        }
    }

    void put(Key key, Value value) override{
        size_t ind = Hash(key) % shardNum_;
        sliceCache_[ind]->put(key, value);
    }

    Value get(Key key) override{
        size_t ind = Hash(key) % shardNum_;
        return sliceCache_[ind]->get(key);
    }

    bool get(Key key, Value& value) override{
        size_t ind = Hash(key) % shardNum_;
        return sliceCache_[ind]->get(key, value);
    }

private:
    size_t Hash(const Key& key) const{
        return std::hash<Key>{}(key);
    }

private:
    int shardNum_;
    std::vector<std::unique_ptr<LRUCache<Key, Value>>> sliceCache_;
};

}