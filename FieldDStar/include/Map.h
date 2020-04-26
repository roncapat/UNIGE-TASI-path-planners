#ifndef MAP_H
#define MAP_H

#include <memory>

class Map {
 public:
  std::shared_ptr<uint8_t[]> image;
  int length;
  int width;
  int x;
  int y;
};

typedef std::shared_ptr<Map> MapPtr;

#endif /* MAP_H */
