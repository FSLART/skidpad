#pragma once
#include "my_node.hpp"
#include "types.hpp"

double distance(double x, double y, double x1, double y1);
std::vector<PathStruct> file_loader(std::string fileName);