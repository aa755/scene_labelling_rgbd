#include <stdio.h>
#include <math.h>
#include <iostream>
using namespace std;

class ColorRGB{
  public:
   float r,g,b;
   ColorRGB(float rgb)
   {
       int rgbi=*reinterpret_cast<int*>(&rgb);
       parseColorRGB(rgbi);
   }

   ColorRGB(int rgbi)
   {
       parseColorRGB(rgbi);
   }


   static float distance(ColorRGB c1,ColorRGB c2)
   {
       return sqrt(pow(c1.r-c2.r,2)+pow(c1.g-c2.g,2)+pow(c1.b-c2.b,2));
   }

   void parseColorRGB(int rgbi)
   {
       int ri=(rgbi&(0xff0000))>>16;
       int gi=(rgbi&(0xff00))>>8;
       int bi=(rgbi&(0xff));
       r=ri/255.0;
       g=gi/255.0;
       b=bi/255.0;
   }

   float getFloatRep()
   {
       int color=(((int)(r*255))<<16)+(((int)(g*255))<<8)+(((int)(b*255)));
       return *reinterpret_cast<float*>(&color);
   }

   float squaredError(ColorRGB c)
   {
       return pow(r-c.r,2)+pow(g-c.g,2)+pow(b-c.b,2);
   }

   void print()
   {
       std::cerr<<r*255<<" "<<g*255<<" "<<b*255<<endl;
   }
};
