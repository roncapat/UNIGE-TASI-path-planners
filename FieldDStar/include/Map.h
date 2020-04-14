#ifndef MAP_H
#define MAP_H

#include <memory>

class Map {
 public:
  std::shared_ptr<uint8_t[]> image;
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
