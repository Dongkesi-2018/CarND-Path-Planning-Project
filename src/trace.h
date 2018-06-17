#ifndef TRACE_H_
#define TRACE_H_

#include <iostream>
#include <string>
using std::string;
using std::cout;
using std::endl;

#define DEBUG_ 0

#if DEBUG_
#define trace_enter() do {cout << "Enter: " << __FUNCTION__ << endl;} while(0); 
#else
#define trace_enter()
#endif

#if DEBUG_
#define trace_exit()  do {cout << "Exit:  " << __FUNCTION__ << endl;} while(0);
#else
#define trace_exit()
#endif

#endif