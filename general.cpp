#include "general.h"

#include <iostream>

using std::cerr;
using std::endl;

void SimpleException::print()
{
  cerr << "Exception '" << text.c_str() << "' in " << file << ":" << line << endl;
  BARF("ok");
}
