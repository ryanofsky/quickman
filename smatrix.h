#ifndef smatrix_h
#define smatrix_h

// Symmetrix Matrix Container Class
//
// Elements are accessed with the function call () operator.
//
// Stored in memory like:
//
//     0 1 2 3 4 5
//
//  0  X
//  1  X X
//  2  X X X
//  3  X X X X
//  4  X X X X X
//  5  X X X X X X

#include <valarray>
using std::min;

template<typename T>
class SMatrix
{
private:
  T * data;
  size_t dim;

  size_t inline pos(size_t x, size_t y)
  {
    return x > y ? (y*(y+1))/2 + x : (x*(x+1))/2 + y;
  }

  void cleanup()
  {
    if (data != NULL)
    {
      delete[] data;
      data = NULL;
    }
  }

public:

  SMatrix() : data(NULL), dim(0) { }

  SMatrix(size_t dim)
  {
    resize(dim);
  }

  ~SMatrix()
  {
    cleanup();
  }

  T & operator() (int x, int y)
  {
    return data[pos(x,y)];
  }

  T const & operator() (int x, int y) const
  {
    return data[pos(x,y)];
  }
  
  int size()
  {
    return dim;
  }
  
  void resize(size_t ndim)
  {
    size_t size = pos(dim,0);
    size_t nsize = pos(ndim,0);
    size = min(size,nsize);
    T * ndata = new T[nsize];
    try
    {
      for(int i=0; i < size; ++i)
        ndata[i] = data[i];
      cleanup();
    }
    catch(...)
    {
      delete [] ndata;
      throw;
    }
    dim = ndim;
    data = ndata;
  }  

private:
  SMatrix(SMatrix const &) { }
  SMatrix & operator=(SMatrix const &) { return *this; }
};

#endif
