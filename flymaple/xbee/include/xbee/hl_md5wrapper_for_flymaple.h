/* 
 * hashlib++ - a simple hash library for C++
 * 
 * Copyright (c) 2007-2010 Benjamin Grüdelbach
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 	1)     Redistributions of source code must retain the above copyright
 * 	       notice, this list of conditions and the following disclaimer.
 * 
 * 	2)     Redistributions in binary form must reproduce the above copyright
 * 	       notice, this list of conditions and the following disclaimer in
 * 	       the documentation and/or other materials provided with the
 * 	       distribution.
 * 	     
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//----------------------------------------------------------------------	

/**
 *  @file 	hl_md5wrapper.h
 *  @brief	This file contains the definition of the md5wrapper
 *  		class.
 *  @date 	Mo 17 Sep 2007
 */  

//---------------------------------------------------------------------- 
//include protection
#ifndef MD5WRAPPER_H
#define MD5WRAPPER_H

//----------------------------------------------------------------------	
//hashlib++ includes
#include "hl_hashwrapper.h"
#include "hl_md5_for_flymaple.h"

//----------------------------------------------------------------------	
//STL includes
#include <string>

//----------------------------------------------------------------------	

/**
 *  @brief 	This class represents the MD5 wrapper-class
 *
 *  		You can use this class to easily create a md5 hash.
 *  		Just create an instance of md5wrapper and call the
 *  		inherited memberfunctions getHashFromString()
 *  		and getHashFromFile() to create a hash based on a
 *  		string or a file. 
 *
 *  		Have a look at the following example:
 *
 *  @include 	md5example.cpp
 *
 *  		md5wrapper implements resetContext(), updateContext()
 *  		and hashIt() to create a hash.
 */  
class md5wrapper : public hashwrapper
{
  private:
    void get_1digit_hex(unsigned int dec, char* hex);
    void get_2digit_hex(unsigned int dec, char* hex);
    
	protected:

		/**
		 * MD5 access
		 */
		MD5 *md5;

		/**
		 * MD5 context
		 */
		HL_MD5_CTX ctx;
	
		/**
		 *  @brief 	This method ends the hash process
		 *  		and returns the hash as string.
		 *
		 *  @return 	the hash as std::string
		 */  
		virtual std::string hashIt(void);

		/**
		 *  @brief 	This internal member-function
		 *  		convertes the hash-data to a
		 *  		std::string (HEX).
		 *
		 *  @param 	data The hash-data to covert into HEX
		 *  @return	the converted data as std::string
		 */  
		virtual std::string convToString(unsigned char *data);
    
    /**
		 *  @brief 	This internal member-function
		 *  		convertes the hash-data to a
		 *  		std::string (HEX) _but_ without using string stream and std::hex so that can be compiled 
     *      using gcc-arm-none-eabi-latest-linux32
		 *
		 *  @param 	data The hash-data to covert into HEX
		 *  @return	the converted data as std::string
		 */  
		virtual std::string convToString_2(unsigned char *data);
    
		/**
		 *  @brief 	This method adds the given data to the 
		 *  		current hash context.
		 *
		 *  @param 	data The data to add to the current context
		 *  @param 	len The length of the data to add
		 */  
		virtual void updateContext(unsigned char *data, unsigned int len);

		/**
		 *  @brief 	This method resets the current hash context.
		 *  		In other words: It starts a new hash process.
		 */  
		virtual void resetContext(void);

		/**
		 * @brief 	This method should return the hash of the
		 * 		test-string "The quick brown fox jumps over the lazy
		 * 		dog"
		 */
		virtual std::string getTestHash(void);

	public:

		/**
		 *  @brief 	default constructor
		 */  
		md5wrapper();

		/**
		 *  @brief 	default destructor
		 */  
		virtual ~md5wrapper();
};

//----------------------------------------------------------------------
// IMPLEMENTATION
//---------------------------------------------------------------------- 
//private member functions

/**
 *  @brief 	This method ends the hash process
 *  		and returns the hash as string.
 *
 *  @return 	the hash as std::string
 */  
std::string md5wrapper::hashIt(void)
{
	//create the hash
	unsigned char buff[16] = "";	
	md5->MD5Final((unsigned char*)buff,&ctx);

	//converte the hash to a string and return it
	return convToString_2(buff);	
}

/**
 *  @brief 	This internal member-function
 *  		convertes the hash-data to a
 *  		std::string (HEX).
 *
 *  @param 	data The hash-data to covert into HEX
 *  @return	the converted data as std::string
 */  
std::string md5wrapper::convToString(unsigned char *data)
{
  return std::string("something goes wrong!");
	///*
	 //* using a ostringstream to convert the hash in a
	 //* hex string
	 //*/
	//std::ostringstream os;
	//for(int i=0; i<16; ++i)
	//{
		///*
		 //* set the width to 2
		 //*/
		//os.width(2);

		///*
		 //* fill with 0
		 //*/
		//os.fill('0');

		///*
		 //* conv to hex
		 //*/
		//os << std::hex << static_cast<unsigned int>(data[i]);
	//}

	///*
	 //* return as std::string
	 //*/
	//return os.str();
}

/**
 *  @brief 	This internal member-function
 *  		convertes the hash-data to a
 *  		std::string (HEX) _but_ without using string stream and std::hex so that can be compiled 
 *      using gcc-arm-none-eabi-latest-linux32
 *
 *  @param 	data The hash-data to covert into HEX
 *  @return	the converted data as std::string
 */
std::string md5wrapper::convToString_2(unsigned char* data)
{
  std::string hex_str = "";
  
  for(int i=0; i<16; ++i) {
    char hex[2] = "";
    get_2digit_hex( static_cast<unsigned int>(data[i]), hex );
    
    hex_str += hex[1];
    hex_str += hex[0];
  }
  
  return hex_str;
}

void md5wrapper::get_1digit_hex(unsigned int dec, char* hex) {
  switch (dec) {
    case 0: *hex =  '0'; break;
    case 1: *hex =  '1'; break;  
    case 2: *hex =  '2'; break;  
    case 3: *hex =  '3'; break;  
    case 4: *hex =  '4'; break;  
    case 5: *hex =  '5'; break;    
    case 6: *hex =  '6'; break;  
    case 7: *hex =  '7'; break;
    case 8: *hex =  '8'; break;
    case 9: *hex =  '9'; break;
    case 10: *hex =  'a'; break;  
    case 11: *hex =  'b'; break;  
    case 12: *hex =  'c'; break;  
    case 13: *hex =  'd'; break;
    case 14: *hex =  'e'; break;
    case 15: *hex =  'f'; break;
  }
}

void md5wrapper::get_2digit_hex(unsigned int dec, char* hex) {
  unsigned int divider;
  unsigned int remainder;
  char c;
  
  // Lo digit
  divider = dec/16;
  remainder = dec%16;
  
  get_1digit_hex(remainder,&c);
  hex[0] = c;
  
  // Hi digit
  remainder = divider%16;
  
  get_1digit_hex(remainder,&c);
  hex[1] = c;
}

/**
 *  @brief 	This method adds the given data to the 
 *  		current hash context.
 *
 *  @param 	data The data to add to the current context
 *  @param 	len The length of the data to add
 */  
void md5wrapper::updateContext(unsigned char *data, unsigned int len)
{
	//update 
	md5->MD5Update(&ctx, data, len);
}

/**
 *  @brief 	This method resets the current hash context.
 *  		In other words: It starts a new hash process.
 */  
void md5wrapper::resetContext(void)
{
	//init md5
	md5->MD5Init(&ctx);
}

/**
 * @brief 	This method should return the hash of the
 * 		test-string "The quick brown fox jumps over the lazy
 * 		dog"
 */
std::string md5wrapper::getTestHash(void)
{
	return "9e107d9d372bb6826bd81d3542a419d6";
}

//---------------------------------------------------------------------- 
//public member functions

/**
 *  @brief 	default constructor
 */  
md5wrapper::md5wrapper()
{
	md5 = new MD5();
}

/**
 *  @brief 	default destructor
 */  
md5wrapper::~md5wrapper()
{
	delete md5;
}

//----------------------------------------------------------------------
//include protection
#endif

//----------------------------------------------------------------------
//EOF
