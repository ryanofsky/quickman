#ifndef general_h
#define general_h

#include<stdio.h>
#include<string>
#include<exception>

using std::string;

#define BARF(x)  throw SimpleException(x, __FILE__, __LINE__)
#define DOUT(x)    cout << x

class SimpleException
{
public:

  string text;
  char const * file;
  int line;

  SimpleException(string text_, char const * file_, int line_)
  : text(text_), file(file_), line(line_) { }
  
  void print();
};


class CFile
{
public:  
  CFile(char const * filename, char const * mode)
  {
    fp = fopen(filename, mode);
    if (!fp)
    {
      BARF(string("File ") + filename + " doesn't open.");
    }
  }

  void close()
  {
    if (fp)
    {
      if (fclose(fp))
      {
        BARF("File doesn't close");
      }
      fp = NULL;
    };
  }

  ~CFile()
  {
    close();
  }

  operator FILE *()
  {
    return fp;
  }

  FILE * fp;

private:
  CFile(CFile const & file) { }
  CFile & operator=(CFile const & file) { return *this; }
};



#endif

