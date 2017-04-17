#include <iostream>
#include <fstream>
#include <CImg.h>
#define cimg_use_jpeg
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <utility> // defines std::pair
#include <list>
//#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>
#include "objloader.hpp"
#include "meshoperator.hpp"
#include "textureoperator.hpp"
#include <math.h>
#include <map>

#include <CGAL/Simple_cartesian.h>
#include "meshsimplification.hpp"
// Type declarations
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::FT                                           FT;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
// In Efficient_RANSAC_traits the basic types, i.e., Point and Vector types
// as well as iterator type and property maps, are defined.
typedef CGAL::Shape_detection_3::Efficient_RANSAC_traits
  <Kernel, Pwn_vector, Point_map, Normal_map>                Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>    Efficient_ransac;
typedef CGAL::Shape_detection_3::Plane<Traits>               Plane;

int main(int argc, char* argv[])
{

      std::vector<vec2 > uvs;
      std::vector< unsigned int > vertexIndices, uvIndices, normalIndices;
      Pwn_vector original_points;
      map<string,int> out_map;
      vector<string> out_texture;
      map <int,RGB> texture_map;
      Point_with_normal* temp_projected_points ;
      std::vector<vec3> plane_vertices;
      std::vector<int> face_classification; //  Same index as faces, 0=none points snapped 1=1 point snapped etc.
      std::vector<Kernel::Plane_3> planes;
      std::vector<Plane*> mainplanes;
      std::vector< Pwn_vector > planes_points;
      int* vertex_snapped ;
      int * vertex_region;
      Pwn_vector projected_points; // Result
      int main_plane_num=0;

      //----------------------------------- Load data -----------------------------------

      Pwn_vector points=loadOBJ("data/test.obj",vertexIndices,uvIndices,normalIndices,uvs,out_texture);
      original_points = points;
      temp_projected_points= new Point_with_normal[original_points.size()];
      vertex_snapped=new int[original_points.size()];
      vertex_region=new int [original_points.size()];
      for (int i=0;i<temp_vertices.size();i++)
      {
          temp_projected_points[i].first=original_points.at(i).first;
          temp_projected_points[i].second=original_points.at(i).second;
          vertex_snapped[i]=-1;
          vertex_region[i]=-1;
      }

      std::vector<int>* neighbor;
      neighbor=new std::vector<int>[points.size()];
      Pwn_vector testpoints=getPoints(points,vertexIndices,neighbor);
      out_map=ConstructMap(original_points);
      texture_map=loadTexture("data/materials_textures/",out_texture,uvs,vertexIndices,uvIndices);
      //writeOBJtest("../Straightening_Mesh/data/test.obj",vertexIndices,uvIndices,normalIndices,original_points,uvs,out_texture);
      std::cout << points.size() << " points" << endl;
      writeTexuredVertices("texturedvertices",original_points,texture_map);
      projected_points.clear();
      for(int i=0;i<original_points.size();i++)
      {
        Point_with_normal p;
        p.first=temp_projected_points[i].first;
        p.second=temp_projected_points[i].second;
        projected_points.push_back(p);

      }
      face_classification=classfyFaces(vertexIndices,vertex_snapped);
      writeOBJ("data/testtexture.obj",vertexIndices,uvIndices,normalIndices,projected_points,uvs,face_classification,vertex_snapped,vertex_region,out_texture);
      //----------------------------------Global fitting----------------------------------
       // Sets probability to miss the largest primitive at each iteration.
       // Detect shapes with at least 1000 points.
       // Sets maximum Euclidean distance between a point and a shape.
       // Sets maximum Euclidean distance between points to be clustered.
       // Sets maximum normal deviation.

       Efficient_ransac::Shape_range shapes = executeRANSAC(testpoints,0.01,1000,0.02,2,0.95);
   //----------------------Save planes and corresponding points------------------------
       ArrangePlanes(shapes,testpoints,planes,planes_points,mainplanes);

   //------------------------------ Add plane constraint--------------------------------
        //refinePlanes(planes,planes_points,0.2,5);
        //std::cout<<planes.size()<<" planes are kept"<<endl;
   //-------------------------------Snap points to planes-------------------------------
       int plane_index=0;
       for(int i=0;i<planes.size();i++)
       {
           snap_points(plane_index,planes.at(plane_index),planes_points.at(i),
                       temp_projected_points,vertex_snapped,plane_vertices,out_map);
           plane_index++;
           main_plane_num++;
       }
       //outputVertices("../Straightening_Mesh/data/Snapped_facade.obj",original_points,vertex_snapped);

       face_classification=classfyFaces(vertexIndices,vertex_snapped);




       // ------------------------------------------------------Region growing-------------------------------------------

       std::vector< Pwn_vector > region_segments;
       std::vector<int> neighbor_plane;
       region_segments=RegionGrow(original_points,testpoints,vertex_snapped,vertex_region,neighbor,out_map,texture_map);
       projected_points.clear();
       for(int i=0;i<original_points.size();i++)
       {
         Point_with_normal p;
         p.first=temp_projected_points[i].first;
         p.second=temp_projected_points[i].second;
         projected_points.push_back(p);

       }
       writeOBJ("data/globalfitting.obj",vertexIndices,uvIndices,normalIndices,projected_points,uvs,face_classification,vertex_snapped,vertex_region,out_texture);
       //------------------------------------------Local fitting-----------------------------

       int * segment_fitted;   // segment_fitted[i] stores the number of planes fitted on segment i
      segment_fitted=new int [region_segments.size()];
      for(int i=0;i<region_segments.size();i++)
      {
          segment_fitted[i]=0;
      }

      for(int i=0;i<region_segments.size();i++)
      {
          if(region_segments.at(i).size()>50)
          {
              Efficient_ransac::Shape_range shapes_segment=executeRANSAC(region_segments.at(i),0.1,50,0.05,0.5,0.95);
              std::vector< Pwn_vector > planes_points_segment;
              int plane_num=ArrangePlanes(shapes_segment,region_segments.at(i),planes,planes_points_segment,mainplanes,true);
              segment_fitted[i]=plane_num;

             for(int j=0;j<plane_num;j++)
              {

                  snap_points(plane_index,planes.at(plane_index),planes_points_segment.at(j),
                              temp_projected_points,vertex_snapped,plane_vertices,out_map);
                  plane_index++;
              }
          }
      }
        face_classification=classfyFaces(vertexIndices,vertex_snapped);
        writePlanes("data/planes.obj",plane_vertices);
      //----------------------------------------------------SplitClusters------------------------------------

      neighbor_plane.clear();
      std::vector< Pwn_vector > new_region_segments=SplitCluster(original_points,planes.size(),vertex_snapped,vertex_region,neighbor_plane,neighbor,out_map);


      projected_points.clear();
      for(int i=0;i<original_points.size();i++)
      {
        Point_with_normal p;
        p.first=temp_projected_points[i].first;
        p.second=temp_projected_points[i].second;
        projected_points.push_back(p);

      }
      writeOBJ("data/localfitting.obj",vertexIndices,uvIndices,normalIndices,projected_points,uvs,face_classification,vertex_snapped,vertex_region,out_texture);
      //----------------------------------------------------Remove spikes-------------------------
        for(int i=0;i<new_region_segments.size();i++)
        {
            // if segment is not fitted by any plane
            if(new_region_segments.at(i).size()<10&&neighbor_plane.at(i)!=-1)
            {
                // Snap the points of the segment to its main plane
                    int plane_id=neighbor_plane.at(i);
                    snap_points(plane_id,planes.at(plane_id),new_region_segments.at(i),temp_projected_points,vertex_snapped,plane_vertices,out_map);

            }
        }




        //---------------------------------remove spikes----------------------------------------------------------------
        //removeSpikes(original_points,planes,temp_projected_points,neighbor,vertex_snapped,out_map );

        //-------------------------------------simplify the mesh-----------------------------

      /*std::vector<double> x;
      std::vector<double> y;
      std::vector<double> z;
      for(int i=0;i<vertexIndices.size();i++)
        {
            int index=vertexIndices.at(i);
            Kernel::Point_3 p= temp_projected_points[index].first;
            x.push_back(p.x());
            y.push_back(p.y());
            z.push_back(p.z());
        }
        LoadMesh(vertexIndices,x,y,z);
        //-------------------------------------output----------------------------------------
        projected_points.clear();
        for(int i=0;i<original_points.size();i++)
        {
          Point_with_normal p;
          p.first=temp_projected_points[i].first;
          p.second=temp_projected_points[i].second;
          projected_points.push_back(p);

        }
      */

        projected_points.clear();
        for(int i=0;i<original_points.size();i++)
        {
          Point_with_normal p;
          p.first=temp_projected_points[i].first;
          p.second=temp_projected_points[i].second;
          projected_points.push_back(p);

        }

       writeOBJ("data/finalresult.obj",vertexIndices,uvIndices,normalIndices,projected_points,uvs,face_classification,vertex_snapped,vertex_region,out_texture);

       new_region_segments=SplitCluster(original_points,planes.size(),vertex_snapped,vertex_region,neighbor_plane,neighbor,out_map);
       projected_points.clear();
       for(int i=0;i<original_points.size();i++)
       {
         Point_with_normal p;
         p.first=temp_projected_points[i].first;
         p.second=temp_projected_points[i].second;
         projected_points.push_back(p);

       }
       writeOBJ("data/mergecolor.obj",vertexIndices,uvIndices,normalIndices,projected_points,uvs,face_classification,vertex_snapped,vertex_region,out_texture);




       //writeOBJtest("data/result.obj",vertexIndices,uvIndices,normalIndices,original_points,uvs,out_texture);
       outputVertices("data/Snapped_Vertices.obj",original_points,vertex_snapped);
       return EXIT_SUCCESS;
}
