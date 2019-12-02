#include "NodeClass.h"
#include <iostream>

#include "NodeClass.h"
NodeClass::NodeClass(std::array<double, 2> coordinate) :coord(coordinate)//constructor
{}

NodeClass::NodeClass()//constructor with no parameters
{
	coord = { 0,0 };
}
