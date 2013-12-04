#include <iostream>
#include <serial-port/TimeoutSerial.h>

using namespace std;
using namespace boost;

int main(int argc, char* argv[])
{
    try {
 
        TimeoutSerial serial("/dev/ttyUSB0",9600);
        serial.setTimeout(posix_time::seconds(5));

        ////Text test
        //serial.writeString("Hello world\r\n");
        //cout<<serial.readStringUntil("\r\n")<<endl;
    
        ////Binary test
        //char values[]={0xde,0xad,0xbe,0xef};
        //serial.write(values,sizeof(values));
        //serial.read(values,sizeof(values));
        //for(unsigned int i=0;i<sizeof(values);i++)
        //{
            //cout<<static_cast<int>(values[i])<<endl;
        //}

        cout<<serial.readStringUntil("\n")<<endl;
        //cout << serial.readString(5) << endl;
        
        serial.close();
  
    } catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        return 1;
    }
}
