// Copyright <2021> [Copyright rossihwang]

#include <grid_mapper/grid_mapper.hpp>

using namespace slamtk;

int main(int argc, char** argv) {
  BoundingBox bbox1;
  bbox1.add_point(cv::Point2f(0, 0));
  bbox1.add_point(cv::Point2f(100, 100));

  bbox1.show("bbox1");

  BoundingBox bbox2;
  bbox2.add_point(cv::Point2f(10, 10));
  bbox2.add_point(cv::Point2f(200, 200));
  bbox2.show("bbox2");

  BoundingBox bbox3;
  BoundingBox bbox4;
  bbox3 = bbox1 | bbox2;
  bbox4 |= bbox1;
  bbox4 |= bbox2;
  bbox3.show("bbox3");
  bbox4.show("bbox4");
  return 0;
}