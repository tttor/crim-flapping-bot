#include "data-format/string_data.h"

using namespace crim;

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
