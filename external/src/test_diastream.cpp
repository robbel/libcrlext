#include <iostream>
#include <gzstream.h>
#include "diastream.hpp"

using namespace std;

int main(int argc, char** argv) {
	diastream os("test.dia");
	
	os << DiaBeginDoc()
       << DiaBeginLayer("top") 
//       << DiaLine(0, 0, 10, 10)
       << DiaBox(2, 2, 5, 5)
       << DiaText(4, 4, "hi!", "#FF0000")
       << DiaArrow(0, 0, 1, 1)
	   << DiaEndLayer() 
	   << DiaEndDoc();
	
	os.close();
	return 0;	
}
