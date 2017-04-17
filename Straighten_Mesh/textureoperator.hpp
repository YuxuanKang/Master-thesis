#ifndef TEXTUREOPERATOR_H
#define TEXTUREOPERATOR_H
#include <math.h>
#include <CImg.h>
#include <string>
#include <vector>
#include <map>
#include "utilities.hpp"
#include "objloader.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>
using namespace std;
using namespace cimg_library;
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
struct RGB{
   public:
    int R,G,B;
    RGB(){};
    RGB(int in_r,int in_g,int in_b):R(in_r),G(in_g),B(in_b){};
};

double RGBdisdance(RGB p1, RGB p2)
{
    double Rdis= abs(p1.R-p2.R);
    double Gdis= abs(p1.G-p2.G);
    double Bdis= abs(p1.B-p2.B);
    return Rdis+Gdis+Bdis;
}

map<int,RGB> loadTexture(string path, std::vector<string> out_texture,std::vector<vec2 > uvs,std::vector< unsigned int > vertexIndices,std::vector< unsigned int > uvIndices)
{
    string previous_file="";
    map<int,RGB> RGBvalues;
    map<int,int> texture_num;
    CImg<unsigned char> img;
    for(int i=0;i<uvIndices.size();i+=3)
    {
       int face_index=i/3;
       if(previous_file!=out_texture.at(face_index))
       {
           previous_file=out_texture.at(face_index);
           string filepath=path+out_texture.at(face_index)+".jpg";
           img.clear();
           img.load(filepath.c_str());
           img.blur(10);
           //img=img.RGBtoHSV();
       }
       int width=img.width();
       int height=img.height();
       for (int j=0;j<3;j++)
       {
           int uvidx=uvIndices.at(i+j)-1;
           int vertexidx=vertexIndices.at(i+j)-1;
           double u=uvs.at(uvidx).x;
           double v=uvs.at(uvidx).y;
           int c=(int)(width*u+0.5);
           int r=(int)(height*(1-v)+0.5);
           int R=(int)img(c,r,0,0);
           int G=(int)img(c,r,0,1);
           int B=(int)img(c,r,0,2);
           RGB pixel(R,G,B);

           /*if(R==0&&G==0&&B==0)
           {
              for(int row=-5;row<=5;row++)
              {
                  for(int col=-5;col<=5;col++)
                  {
                        if(r+row<0||r+row>=height||c+col<0||c+col>=width)
                        {
                            continue;
                        }
                        R=(int)img(c+col,r+row,0,0);
                        G=(int)img(c+col,r+row,0,1);
                        B=(int)img(c+col,r+row,0,2);
                        if(R+G+B>30)
                        {
                            break;
                        }
                  }
              }
           }*/



           if(RGBvalues.count(vertexidx)==0)
           {
               RGBvalues[vertexidx]=pixel;
               texture_num[vertexidx]=1;
           }
           else
           {
               RGB newPixel(RGBvalues[vertexidx].R+pixel.R,RGBvalues[vertexidx].G+pixel.G,RGBvalues[vertexidx].B+pixel.B);
               RGBvalues[vertexidx]=newPixel;
               texture_num[vertexidx]++;
           }
       }
       //}

    }
    for ( const auto &myPair : RGBvalues )
    {

            int key=myPair.first;
            int num=texture_num[key];
            RGB value(RGBvalues[key].R,RGBvalues[key].G,RGBvalues[key].B);
            value.R/=num;
            value.G/=num;
            value.B/=num;
            RGBvalues[key]=value;
     }

    return RGBvalues;
}
#endif // TEXTUREOPERATOR_H
