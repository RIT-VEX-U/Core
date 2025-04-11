#pragma once

#include <map>

#include "core/utils/math/eigen_interface.h"

/**
 * This class implements a map of key-value pairs.
 * 
 * If there is not a pair with the given key in the map, the value will be a
 * linear interpolation of the preceding and following values.
 * 
 * @tparam KEY The type of the key.
 * @tparam VALUE The type of the value.
 */
template <typename KEY, typename VALUE> class InterpolatingMap {
  public:
    using MapType = std::map<KEY, VALUE, std::less<KEY>, Eigen::aligned_allocator<std::pair<const KEY, VALUE>>>;

    /**
     * Inserts a key value pair.
     * 
     * @param key The key.
     * @param vlue The value.
     */
    void insert(const KEY &key, const VALUE &value) { map_.insert({key, value}); }

    /**
     * Obtains the value at the given key.
     * 
     * If the key does not exactly match a pair in the map, it will interpolate
     * between the preceding and following pairs.
     * 
     * @param key The key.
     * 
     * @return The value.
     */
    VALUE operator[](const KEY &key) {
        // Get iterator for the upper bound of our key.
        typename MapType::const_iterator upper = map_.upper_bound(key);

        // If our key greater than the largest key return its value.
        if (upper == map_.end()) {
            return (--upper)->second;
        }   

        // If our key less than or equal to the smallest key return its value.
        if (upper == map_.begin()) {
            return upper->second;
        }

        // Get an iterator to the previous entry.
        typename MapType::const_iterator lower = upper;
        --lower;

        // Linear interpolate between the values of the first and second.
        const double delta = (key - lower->first) / (upper->first - lower->first);
        return delta * upper->second + (1.0 - delta) * lower->second;
    }

    /**
     * Clears the contents of the map.
     */
    void clear() { map_.clear(); }

  private:
    MapType map_;
};
