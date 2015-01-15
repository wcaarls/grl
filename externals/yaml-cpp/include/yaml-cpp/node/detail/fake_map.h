/* 
 * A deque that masquerades as a map. Ugly hack to make sure we
 * iterate over YAML maps in insertion order.
 *
 * (c) 2015 Wouter Caarls
 */

#ifndef FAKE_MAP_H_
#define FAKE_MAP_H_

#include <deque>

template<class key_type, class mapped_value>
class fake_map
{
  public:
    typedef std::pair<key_type, mapped_value> Item;
    typedef std::deque<Item> ItemQueue;
    
    typedef typename ItemQueue::iterator iterator;
    typedef typename ItemQueue::const_iterator const_iterator;

  protected:
    ItemQueue queue_;

  public:
    size_t size() const
    {
      return queue_.size();
    }
  
    mapped_value& operator[] (const key_type& k)
    {
      for (typename ItemQueue::iterator it=queue_.begin(); it != queue_.end(); ++it)
        if (it->first == k)
          return it->second;
      
      queue_.push_back(Item(k, mapped_value()));
      return queue_.back().second;
    }
    
    void erase(const iterator position)
    {
      queue_.erase(position);
    }
    
    void clear()
    {
      queue_.clear();
    }
  
    iterator begin()
    {
      return queue_.begin();
    }
    
    const_iterator begin() const
    {
      return queue_.begin();
    }
    
    iterator end()
    {
      return queue_.end();
    }
    
    const_iterator end() const
    {
      return queue_.end();
    }
};

#endif /* FAKE_MAP_H */
