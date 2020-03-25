#ifndef MAP_H
#define MAP_H

#include <boost/multi_array.hpp>
#include <memory>

class Map {
 public:
  boost::multi_array<uint8_t, 2> image;
  float resolution;
  float orientation;
  int length;
  int width;
  int x;
  int y;
  int x_initial;
  int y_initial;
};

typedef std::shared_ptr<Map> MapPtr;

#endif /* MAP_H */
