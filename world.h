#ifndef world_h
#define world_h

class World
{
  enum { NONE = -1 };
  
  struct coord
  {
    int x,y;
    int shapeno;
    
    coord() : x(0), y(0), shapeno(NONE), vertextno(NONE) : { }
    coord(int x_, int y_, int shapeno_ = 0)
    : x(x_), y(y_), shapeno(shapeno_) {}
    
    coord operator+(coord c)
    {
      return coord(x + c.x, y + c.y);
    }

    coord operator-(coord c)
    {
      return coord(x - c.x, y - c.y);
    }    
    
    bool equals(coord const & c) const
    {
      return x == c.x && y == c.y;
    }
    
    double distanceTo(coord c)
    {
      double dx = x - c.x;
      double dy = y - c.y;
      return sqrt(dx*dx + dy*dy);
    }
    
    bool leftof(coord P, coord Q)
    {
    }
    
  }
  
  struct shape
  {
    int startidx;
    int vertices;
    
    shape() : startidx(0), vertices(0) {}
    shape(int startidx_, int vertices_) 
    : startidx(startidx_), vertices(vertices_) { } 
  }
  
  
  
  
  vector<coord> coordinates;
  vector<coord> gcoordinates;
  vector<shape> shapes;
  
  
  void readShapes(char const * file)
  { 
    int coordno = 0;
    int shapeno = 0;
    int vertexno = 0;
    
    finstream f(file);
    while(!f.eof())
    {
      coord & c = coordinates[coordno];
      c.x = f.token;
      c.y = f.token;
      c.shapeno = shapeno;
      c.vertexno = vertexno;
      if (f.token)
      {
        shapes[shapeno] = shape(coordno, vertexno);
        ++shapeno;
        vertexno = 0;
      }  
      else
        ++vertexno;
    }
  }
  
  void growshapes()
  {
    
  }
};

#endif