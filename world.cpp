
World::coord World::robot[] = 
{
  coord(0,0),
  coord(370,0),
  coord(370,550),
  coord(0,550),
  coord(185, 275) 
}

void World::read(const char * filename)
{
function seek(FILE * fp)
{
  const size_t BUFFERSIZE = 4096;
  char buffer[BUFFERSIZE];

  int shapeno = 0;
  int vertexno = 0;
  int shapestart = 0;
  enum { XCOORD, YCOORD, CRUFT } state;

  state = XCOORD;
  int newlines(0);
  
  string num = "";
  
  for(buffer_start = 0;;buffer_start += BUFFERSIZE;)
  {
    fseek(fp,buffer_start);
    size_t buffer_size = fread(buffer,sizeof(char), BUFFERSIZE, fp);

    for(size_t p = 0; p <= buffer_size; ++p)
    {
      char c = p == buffer_size ? 0 : buffer[p];
      
      is_whitespace = c <= 32;
      is_num = isdigit(c);
      is_line = c == '\n' || c == '\r';
      
      if (is_num)
      {
        num += c;
        newlines = 0;
      }
      else
      {
        if (num.length() > 0) // new field found
        {
          if (newlines > 0)
          {
            ++vertexno;  
            state = XCOORD;
            vertices[vertexno] = coord(0,0,shapeno);
            if (newlines > 1)
            {
              ++shapeno;
              shapes[shapeno] = shape(vertexno,0);
            }
          }
         
          int i = atoi(num.c_str());
          
          if (state == XCOORD)
            vertices[vertexno].x = i;
          else if (state == YCOORD)
          {
            vertices[vertexno].y = i;
            ++shapes[shapeno].vertices;
          }
          else
            cerr << "Cruft '" << num << "' found." << endl;
          
          if (state < CRUFT) ++state;
          num = "";
        } // num.length() > 0
        
        if (is_line)
        {
          if(!last_line || lastc == c))
            ++newlines; 
          else
            c = lastc;
        }
      }
      
      lastc = c; last_line = is_line;
    } // for p
    if (buffersize == 0) break;
  }
}

void World::findGrown()
{
  vector<coord> gpoints;
  
  size_t vertexno = 0;
  
  vector<coord> points;
  vector<coord> hull;
  
  coord rreference = robot[DIM(robot) - 1];
  
  for(int shape = 0; shape < shapes.length; ++shape)
  {
    points.resize(0);
    for(int vertex = 0; vertex < shapes[shape].vertices; ++vertex)
    {
      int vindex shapes[shape].startidx + vertex;
      for(int rvertex = 0; rvertex < DIM(robot) - 1; ++rvertex)
        points.push_back(points[vertex] + robot[rvertex] - rreference);
    }
    convexHull(points, hull);
    typedef vector<coord>::iterator ivcoord;
    
    gshapes[shape] = shape(vertexno,0);
    for(ivcoord i = hull.begin(); i != hull.end(); ++i)
    {
      gcoordinates[vertexno] = *i;
      ++vertexno;
    }
    gshapes[shape].vertices = vertexno - gshapes[shape].startidx;
  }
  
  enum Visibility { UNKNOWN, VISIBLE, BLOCKED };
  
  int gpl = gpoints.length;
  smatrix<visibility> gvisible(gpl);
  smatrix<bool> merge(shapes.length());
  int gintersect
  
  for(int p = 0; p < gpl; ++p)
  for(int q = 0; q < p; ++q)
  {
    coord & P = gpoints[p];
    coord & Q = gpoints[q];
    if (P.shapeno == Q.shapeno)
    {
      gintersect(p,q) = (p adjacent to q) ? VISIBLE : BLOCKED;
    }
    else
    {
      bool blocked = false;
      for(int r = 0; p < gpl; ++p)
      for(int s = 0; q < p; ++q)
      {
        coord & R = gpoints[r];
        coord & S = gpoints[s];
        
        if (intersects(p,q,r,s)) 
        {
          if (p.shapeno == q.shapeno && r.shapeno == s.shapeno)
            merge(p.shapeno, r.shapeno) = true;
          else
          {
            blocked = true;
            goto intersection_found;
          }
        }
      }
      intersection_found:
      gintersect(p,q) = blocked ? VISIBLE : BLOCKED;
    }
  }
  
  // handle merges...
  
  
};

void convexHull(vector<coord> const & points, vector<coord> & hull)
{
  const double BADANGLE = -6000.0;
  int l = points.length;
  
  // find rightmost, lowest point
  int pivot = 0;
  for(int i = 1; i < l; ++i)
  {
    if (point[i].x > point[pivot].x || (point[i].x == point[pivot].x && point[i].y > point[pivot].y))
      pivot = i;
  }
  
  // find angles about pivot
  vector<ccoord> ccoords(l);
  
  class ccoord // cached coordinate
  {
    coord c;
    double a;
    ccoord(coord c_, double a_) : c(c_), a(a_) { } 
  }
  
  typedef vector<ccoord> tccoords;
  tccoords ccoords(l);
  
  
  class csort
  (
    coord pivot;
    csort(coord pivot_) : pivot(pivot_) { } 
    
    operator(ccoord a, ccoord b)
    {
      if (a.a != b.a)
        return a.a < b.a;
      else if (a.a == BADANGLE)
        return false;
      else
        return pivot.distanceTo(a.c) < pivot.distanceTo(b.c);
    }
  )
  
  for(int i = 0; i < l; ++i)
  {
    if (point[i].equals(pivot))
      ccords[i] = ccoord(point[i],BADANGLE);
    else
      ccords[i] = ccoord(point[i], atan2(point[i].y - pivot.y, point[i].x - pivot[x]));
  }
  
  sort(ccoords.begin(), ccoords.end(), csort(points[pivot]));
  
  hull.resize(0);
  hull.push_back(ccoords[0].c);
  hull.push_back(points.back()->c);
  
  int i = 1;
  while(i < N)
  {
    tccoords::const_iterator top(coords.end());
    --top;
    tccoords::const_iterator ptop(top);
    --ptop;
    
    vector<ccoord>
    if (ccoords[i].c.leftOf(*ptop, *top))
    {
      hull.push_back(ccoords[i].c);
      ++i;
    }
    else
      hull.pop_back();
  }
}

void intersects(coord s1, coord e1, coord s2, coord e2)
{
  
}


