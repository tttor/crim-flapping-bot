// @author vektor dewanto
#ifndef STRING_DATA_H
#define STRING_DATA_H

#include <string>
#include <stdio.h> // for snprintf(), printf(), etc
#include <vector>

namespace crim {

struct StringData{
  std::vector< std::vector<std::string> > content;
  
  void print();
};

}//namespace crim

#endif
