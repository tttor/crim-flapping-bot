// @author vektor dewanto
#ifndef STRING_DATA_H
#define	STRING_DATA_H

#include <vector>
#include <stdio.h> // for printf()

namespace crim{
  
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// DECLARATION
struct StringData{
  std::vector< std::vector<std::string> > content;
  
  void print();
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
void StringData::print() {
  printf("n_field= %ld\n", content.size());
  for(size_t i=0; i<content.size(); ++i) {
    printf("FIELD %ld with %ld subfields:\n", i, content.at(i).size());
    
    for(size_t j=0; j<content.at(i).size(); ++j) {
      printf("%s\n", content.at(i).at(j).c_str());
    }
    printf("%s","\n");
  }
}

}// namespace crim
#endif
