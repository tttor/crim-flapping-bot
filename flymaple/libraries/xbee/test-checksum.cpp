// Unit test: Xbee
// This sends back what is received
#include <wirish/wirish.h>

#include <xbee/hl_md5wrapper.h>
#include <xbee/hl_md5.h>

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
  hashwrapper *h = new md5wrapper();
	h->test();
  
  std::string target = "centerforroboticsandintelligentmachines";
	std::string md5 = h->getHashFromString(target);
  
  while (true) {
    SerialUSB.println(md5.c_str());
    delay(1000);
  }
  return 0;
}
