#pragma once

#include <unordered_map>
#include <memory>
#include <mutex>

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
            refreshExistingNode(keyToNode_[key]);
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

}