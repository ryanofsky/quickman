#ifndef general_h
#define general_h

#include<stdio.h>
#include<string>
#include<exception>
#include<algorithm>
#include<assert.h>

using std::string;
using std::swap;

#define BARF(x)  throw SimpleException(x, __FILE__, __LINE__)
#define DOUT(x)    cerr << x

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

/*

mheap functions are stl-like generic algorithms.

they are designed to work with heaps where you want to store (in some external)
data structure the ordering of elements inside the heaps. this is useful, for
example, if you want to change the priority of single elements in the heap 
without having to resort the whole thing.

The UpdateFunctor parameters are what allow you to keep track of the ordering.
Each time an element is moved from one position to another in the heap, the
UpdateFunctor is called with arguments (HeapItem, Newpos) where HeapItem can
be a copy or a reference to the heap item that is being moved and Newpos is
an iterator that points to the new position in the heap. 

*/


// Exactly like pop_heap, but with an UpdateFunctor
template<class RandomAccessIterator, class StrictWeakOrdering, class UpdateFunctor>
void pop_mheap(RandomAccessIterator first, RandomAccessIterator last, StrictWeakOrdering comp, UpdateFunctor update)
{
  assert(last > first);
  if (first == last - 1) return;
  
  update(*first, last - 1);
  update(*(last - 1), first);
  swap(*first,*(last-1));
  
  int i = 0;
  int heapsize = last - first - 1;  
  for(;;)
  {
    int l = i * 2 + 1;
    int r = l + 1;
    int max;

    if(l < heapsize && comp(*(first + i), *(first + l)))
      max = l;
    else
      max = i;
    if(r < heapsize && comp(*(first + max), *(first + r)))
      max = r;
  
    if (max == i) break;
    
    RandomAccessIterator imax = first + max;
    RandomAccessIterator ii = first + i;
    
    update(*imax, ii);
    update(*ii, imax);
    swap(*imax, *ii);
    i = max;
  }
}

/*
Call decreasekey_mheap when you've decreased the value of a
key in the heap and you want to restore the heap ordering
*/
template<class RandomAccessIterator, class StrictWeakOrdering, class UpdateFunctor>
void decreasekey_mheap(RandomAccessIterator first, RandomAccessIterator key, StrictWeakOrdering comp, UpdateFunctor update)
{
  RandomAccessIterator me, parent;
  me = key;
  while(me != first)
  {
    parent = first + (me - first - 1) / 2;
    if (comp(*parent,*me)) // key of me > key of parent
    {
      update(*me, parent);
      update(*parent, me);
      swap(*me, *parent);
    }
    me = parent;
  };
}

#endif

