#include <map>
#include <CGAL/Point_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection_3.h>
#include <CGAL/regularize_planes.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Plane_3.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include "utilities.hpp"
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef Kernel::FT                                           FT;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef CGAL::Shape_detection_3::Efficient_RANSAC_traits
  <Kernel, Pwn_vector, Point_map, Normal_map>                Traits;
typedef CGAL::Shape_detection_3::Plane<Traits>               Plane;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>    Efficient_ransac;
using namespace std;
#define max_num 99999999
#define min_num -99999999
#define PI 3.14159265
double calculateAngle(Kernel::Vector_3 v1, Kernel::Vector_3 v2){
    double dot=(v1.x()*v2.x()+v1.y()*v2.y()+v1.z()*v2.z())/(v1.squared_length()*v2.squared_length());
    return acos(dot)*180.0/PI;
}


bool getPlaneVertices(Kernel::Plane_3 plane3d,vector<vec3> &plane_vertices,double min3d_x,double min3d_y,double min3d_z,double max3d_x,double max3d_y,double max3d_z){
    Kernel::Point_3 *p =new Kernel::Point_3[8];
    p[0]=Kernel::Point_3(min3d_x,min3d_y,min3d_z);
    p[1]=Kernel::Point_3(max3d_x,min3d_y,min3d_z);
    p[2]=Kernel::Point_3(max3d_x,max3d_y,min3d_z);
    p[3]=Kernel::Point_3(min3d_x,max3d_y,min3d_z);
    p[4]=Kernel::Point_3(min3d_x,min3d_y,max3d_z);
    p[5]=Kernel::Point_3(max3d_x,min3d_y,max3d_z);
    p[6]=Kernel::Point_3(max3d_x,max3d_y,max3d_z);
    p[7]=Kernel::Point_3(min3d_x,max3d_y,max3d_z);
    double min2d_x=max_num;
    double min2d_y=max_num;
    double max2d_x=min_num;
    double max2d_y=min_num;
    for (int i=0;i<8;i++){
        if(plane3d.to_2d(p[i]).x()<min2d_x){
            min2d_x=plane3d.to_2d(p[i]).x();
        }
        if(plane3d.to_2d(p[i]).x()>max2d_x){
            max2d_x=plane3d.to_2d(p[i]).x();
        }
        if(plane3d.to_2d(p[i]).y()<min2d_y){
            min2d_y=plane3d.to_2d(p[i]).y();
        }
        if(plane3d.to_2d(p[i]).y()>max2d_y){
            max2d_y=plane3d.to_2d(p[i]).y();
        }
    }
    Kernel::Point_2 p1_2d(min2d_x,min2d_y);
    Kernel::Point_2 p2_2d(max2d_x,min2d_y);
    Kernel::Point_2 p3_2d(max2d_x,max2d_y);
    Kernel::Point_2 p4_2d(min2d_x,max2d_y);
    vec3 v1,v2,v3,v4;
    v1.x=plane3d.to_3d(p1_2d).x(); v1.y=plane3d.to_3d(p1_2d).y(); v1.z=plane3d.to_3d(p1_2d).z();
    v2.x=plane3d.to_3d(p2_2d).x(); v2.y=plane3d.to_3d(p2_2d).y(); v2.z=plane3d.to_3d(p2_2d).z();
    v3.x=plane3d.to_3d(p3_2d).x(); v3.y=plane3d.to_3d(p3_2d).y(); v3.z=plane3d.to_3d(p3_2d).z();
    v4.x=plane3d.to_3d(p4_2d).x(); v4.y=plane3d.to_3d(p4_2d).y(); v4.z=plane3d.to_3d(p4_2d).z();
    plane_vertices.push_back(v1);plane_vertices.push_back(v2);plane_vertices.push_back(v3);plane_vertices.push_back(v4);
    return true;
}
std::vector<int> classfyFaces(std::vector< unsigned int >  vertexIndices,int* vertex_snapped)
{
    std::vector<int> classified;
    for (int vertexindex=0;vertexindex<vertexIndices.size()-3;vertexindex+=3){
        int snapped_num=0;
        int idx1=vertexIndices.at(vertexindex)-1;
        int idx2=vertexIndices.at(vertexindex+1)-1;
        int idx3=vertexIndices.at(vertexindex+2)-1;
        if (vertex_snapped[idx1]!=-1){
            snapped_num++;
        }
        if (vertex_snapped[idx2]!=-1){
            snapped_num++;
        }
        if (vertex_snapped[idx3]!=-1){
            snapped_num++;
        }
        classified.push_back(snapped_num);
    }
    return classified;

}
std::vector<int> * getTopology(int point_num,std::vector< unsigned int >  vertexIndices)
{
    std::vector<int> *neighbor;
    neighbor=new std::vector<int>[point_num];
    for (int vertexindex=0;vertexindex<vertexIndices.size()-3;vertexindex+=3)
    {
       int idx1=vertexIndices.at(vertexindex)-1;
       int idx2=vertexIndices.at(vertexindex+1)-1;
       int idx3=vertexIndices.at(vertexindex+2)-1;
       if(std::find(neighbor[idx1].begin(), neighbor[idx1].end(), idx2)==neighbor[idx1].end())
       {
             neighbor[idx1].push_back(idx2);
       }
       if(std::find(neighbor[idx1].begin(), neighbor[idx1].end(), idx3)==neighbor[idx1].end())
       {
             neighbor[idx1].push_back(idx3);
       }
       if(std::find(neighbor[idx2].begin(), neighbor[idx2].end(), idx1)==neighbor[idx2].end())
       {
             neighbor[idx2].push_back(idx1);
       }
       if(std::find(neighbor[idx2].begin(), neighbor[idx2].end(), idx3)==neighbor[idx2].end())
       {
             neighbor[idx2].push_back(idx3);
       }
        if(std::find(neighbor[idx3].begin(), neighbor[idx3].end(), idx1)==neighbor[idx3].end())
        {
             neighbor[idx3].push_back(idx1);
        }
         if(std::find(neighbor[idx3].begin(), neighbor[idx3].end(), idx2)==neighbor[idx3].end())
        {
             neighbor[idx3].push_back(idx2);
        }

   }
    return neighbor;
}
std::vector< vector<int> > RegionGrow(Pwn_vector points,int num_planes,int* vertex_snapped,int* vertex_region,std::vector<int> &neighbor_plane,std::vector<int> * neighbor,map<string,int> out_map, bool small_scale=false)
{
    std::vector<int> unsnapped;
    std::vector< vector<int> > region_segments;
    int *segment_neighbor;
    segment_neighbor=new int[num_planes];
    // Record which points are not snapped into the vector
    if(small_scale==false){
        for(int i=0;i<points.size();i++)
        {
            string key=gen_key_bucket(points.at(i).first);
            int original_index=out_map[key];
            if (vertex_snapped[original_index]==-1){
                unsnapped.push_back(original_index);
            }
        }
    }
    else{
        for(int i=0;i<points.size();i++)
        {
            string key=gen_key_bucket(points.at(i).first);
            int original_index=out_map[key];
            unsnapped.push_back(original_index);
        }
    }
    // ------------------------------------------------Region growing--------------------------------------------
    int region_index=0;
     while(unsnapped.empty()==false)
     {
        int seed=unsnapped.back();
        unsnapped.pop_back();
        std::vector<int> region;
        if (neighbor[seed].empty()==true)
        {
            continue;
        }
        region.push_back(seed);
        int region_plane=vertex_snapped[seed];
        //Kernel::Vector_3 mean_normal(points.at(seed).second.x(),points.at(seed).second.y(),points.at(seed).second.z());
        int edgepoint=0;
        for (int i=0;i<num_planes;i++)
        {
            segment_neighbor[i]=0;
        }
        std::vector<int> ::iterator itr=region.begin();
        for (int pt_index=0;pt_index<region.size();pt_index++)
        {
            int index= region.at(pt_index);
            for(int i=0;i<neighbor[index].size();i++)
            {
                int neighbor_index=neighbor[index].at(i);
                //Kernel::Vector_3 normal(points.at(neighbor_index).second.x(),points.at(neighbor_index).second.y(),points.at(neighbor_index).second.z());
                //double dot_product=mean_normal*normal;
                if(std::find(region.begin(), region.end(), neighbor_index)==region.end()&&vertex_snapped[neighbor_index]==region_plane)
                {
                    region.push_back(neighbor_index);
                    vertex_region[neighbor_index]=region_index;
                    unsnapped.erase(std::remove(unsnapped.begin(), unsnapped.end(), neighbor_index), unsnapped.end());
                   // mean_normal=mean_normal+normal;
                }
                else if(vertex_snapped[neighbor_index]!=-1)
                {
                    int plane_id= vertex_snapped[neighbor_index];
                    segment_neighbor[plane_id]++;
                    edgepoint++;
                }

            }
           itr++;
        }
        // Check if all edges are snapped to the same plane, if so, store the plane id
            region_segments.push_back(region);
            region_index++;
            bool flag=false;
            for(int i=0;i<num_planes;i++)
            {
                if(segment_neighbor[i]>edgepoint*0.5)
                {
                    neighbor_plane.push_back(i);
                    flag=true;
                    break;
                }
            }
            // if not, store -1
            if(flag==false)
            {
                neighbor_plane.push_back(-1);
            }
    }
    return region_segments;

}
template <typename T>
void Append(std::vector<T>& a, const std::vector<T>& b)
{
    a.reserve(a.size() + b.size());
    a.insert(a.end(), b.begin(), b.end());
}

/*void refinePlanes(std::vector<Kernel::Plane_3> &planes,std::vector< vector<Point_with_normal> > &planes_points,double dis_threshold,double angle_threshold)
{
    //----------------------------------------Compare two planes---------------------------------------
    int *merge_map;
    merge_map=new int[planes.size()];
    for(int i=0;i<planes.size();i++)
    {
        merge_map[i]=-1;
    }
    for(int plane_id1=0;plane_id1<planes.size();plane_id1++)
    {
        for(int plane_id2=0;plane_id2<planes.size();plane_id2++)
        {
            if(plane_id1!=plane_id2&&merge_map[plane_id1]==-1&&merge_map[plane_id2]==-1)
            {
                Kernel::Plane_3 plane1=planes.at(plane_id1);
                Kernel::Plane_3 plane2=planes.at(plane_id2);
                Kernel::Vector_3 normal1=plane1.base2();
                Kernel::Vector_3 normal2=plane2.base2();
                vector<Point_with_normal>  plane1_points=planes_points.at(plane_id1);
                vector<Point_with_normal>  plane2_points=planes_points.at(plane_id2);
                double angle=calculateAngle(normal1,normal2);
                if(angle<angle_threshold)
                {
                    // calculate distance betweeen two planes: average distance of all points from one plane to another
                    double sum_distance=0;
                    for(int pt_id=0;pt_id<plane1_points.size();pt_id++)
                    {
                        Point_with_normal p= plane1_points.at(pt_id);
                        sum_distance+=plane2->squared_distance(p.first);
                    }
                    double avg_distance=sum_distance/plane1_points.size();
                    if(avg_distance<dis_threshold)
                    {
                         if(plane1_points.size()>=plane2_points.size())
                         {
                            merge_map[plane_id2]=plane_id1;
                         }
                         else
                         {
                             merge_map[plane_id1]=plane_id2;
                         }
                    }

                }
            }
            else
            {
                 continue;
            }
        }
    }
    int planesize=planes.size();
    for(int i=planesize-1;i>=0;i--)
    {
        if(merge_map[i]!=-1)
        {
            int index=merge_map[i];
            planes_points.at(index).reserve(planes_points.at(index).size()+planes_points.at(i).size());
            planes_points.at(index).insert(planes_points.at(index).end(),planes_points.at(i).begin(),planes_points.at(i).end());
            planes.erase(planes.begin()+i);
            planes_points.erase(planes_points.begin()+i);

        }
    }

}*/


Efficient_ransac::Shape_range executeRANSAC(Pwn_vector & points,double probability, int min_points,double epsilon ,double cluster_epsilon, double normal_threshold)
{
    // Instantiates shape detection engine.
    Efficient_ransac ransac;
    // Provides the input data.
    ransac.set_input(points);

    // Register shapes for detection
    ransac.add_shape_factory<Plane>();
    // Sets parameters for shape detection.
    Efficient_ransac::Parameters parameters;
    // Sets probability to miss the largest primitive at each iteration.
    parameters.probability = probability;

    // Detect shapes with at least 800 points.
    parameters.min_points = min_points;
    // Sets maximum Euclidean distance between a point and a shape.
    parameters.epsilon = epsilon;

    // Sets maximum Euclidean distance between points to be clustered.
    parameters.cluster_epsilon = cluster_epsilon ;

    // Sets maximum normal deviation.
    // 0.9 < dot(surface_normal, point_normal);
    parameters.normal_threshold = normal_threshold;
    // Detects shapes
    ransac.detect(parameters);// Regularize detected planes

    // Regularize detected planes
    CGAL::regularize_planes (ransac,
                             true, // Regularize parallelism
                             true, // Regularize orthogonality
                             false, // Do not regularize coplanarity
                             true, // Regularize Z-symmetry (default)
                             5); // 10 degrees of tolerance for parallelism/orthogonality
    std::cout << ransac.shapes().end() - ransac.shapes().begin() << " detected shapes, "
      << ransac.number_of_unassigned_points()
      << " unassigned points." << std::endl;
    return ransac.shapes();

}
void snap_single_point(int planeIndex,Point_with_normal pt,Kernel::Plane_3 plane,Point_with_normal* temp_projected_points,int *vertex_snapped,map<string,int> out_map)
{
     Point_with_normal projected_point;
     projected_point.first=plane.projection(pt.first);
     projected_point.second=pt.second;
     double dis= (projected_point.first.x()-pt.first.x())*(projected_point.first.x()-pt.first.x())
             +(projected_point.first.y()-pt.first.y())*(projected_point.first.y()-pt.first.y());
     if(dis<10){
         string key=gen_key_bucket(pt.first);
         int original_index=out_map[key];
         temp_projected_points[original_index]=projected_point;
         vertex_snapped[original_index]=planeIndex;
     }
}
void snap_points(int planeIndex,const Kernel::Plane_3 plane,Pwn_vector plane_points,Point_with_normal* temp_projected_points,int* vertex_snapped,std::vector<vec3>& plane_vertices,map<string,int> out_map,double threshold=1)
{
    int point_index=0;
    //std::cout<<"Plane: "<<count<<endl;
    double min3d_x,min3d_y,min3d_z;
    double max3d_x,max3d_y,max3d_z;
       min3d_x=min3d_y=min3d_z=max_num;
       max3d_x=max3d_y=max3d_z=min_num;
       // Using Shape_base::info() for printing
       // Sums distances of points to detected shapes.
       FT sum_distances = 0;
       // Iterates through point indices assigned to each detected shape.
         while (point_index!= plane_points.size()) {
          // Retrieves point
          const Point_with_normal &p = plane_points.at( point_index);
          // Adds Euclidean distance between point and shape.
          //sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));
          // Proceeds with next point.
          // Project the points to the corresponding planes.
              // Plane shape can also be converted to Kernel::Plane_3
              Point_with_normal projected_point;
              projected_point.first=plane.projection(p.first);
              projected_point.second=p.second;
              double dis= (projected_point.first.x()-p.first.x())*(projected_point.first.x()-p.first.x())
                      +(projected_point.first.y()-p.first.y())*(projected_point.first.y()-p.first.y())
                      +(projected_point.first.z()-p.first.z())*(projected_point.first.z()-p.first.z());
              dis=sqrt(dis);
              if(dis<threshold){
                  string key=gen_key_bucket(p.first);
                  int original_index=out_map[key];
                  temp_projected_points[original_index]=projected_point;
                  vertex_snapped[original_index]=planeIndex;
              }
              else{
                  point_index++;
                  continue;
              }
              //find bounding box of the plane
              if(projected_point.first.x()<min3d_x)
              {
                  min3d_x=p.first.x();
              }
              if(projected_point.first.x()>max3d_x)
              {
                  max3d_x=p.first.x();
              }
              if(projected_point.first.y()<min3d_y)
              {
                  min3d_y=p.first.y();
              }
              if(projected_point.first.y()>max3d_y)
              {
                  max3d_y=p.first.y();
              }
              if(projected_point.first.z()<min3d_z)
              {
                  min3d_z=p.first.z();
              }
              if(projected_point.first.y()>max3d_z)
              {
                  max3d_z=p.first.z();
              }
             point_index++;
          }
         //----------------------find bounding box of the plane----------------
           getPlaneVertices(plane,plane_vertices,min3d_x,min3d_y,min3d_z,max3d_x,max3d_y,max3d_z);



           // Computes and prints average distance.
             //FT average_distance = sum_distances / shape->indices_of_assigned_points().size();
             //std::cout << " average distance: " << average_distance << std::endl;
       // Proceeds with next detected shape.
}

Pwn_vector getPointVector(Pwn_vector original_points,vector<int> region_segment)
{
    Pwn_vector points;
    for (int i=0;i<region_segment.size();i++)
    {
        int vertexIndex=region_segment.at(i);
        points.push_back(original_points.at(vertexIndex));
    }
    return points;
}

int ArrangePlanes(Efficient_ransac::Shape_range shapes,Pwn_vector points ,std::vector<Kernel::Plane_3> &planes,std::vector< Pwn_vector > &planes_points)
{
    Efficient_ransac::Shape_range::iterator it = shapes.begin();
    int plane_num=0;
    while(it!=shapes.end())
    {
         std::cout << (*it)->info()<<endl;
         boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
         vector< Point_with_normal> plane_points;
         std::vector<std::size_t>::const_iterator index_it = (*it)->indices_of_assigned_points().begin();
         if(Plane* plane = dynamic_cast<Plane*>(it->get()))
         {
             while(index_it !=(*it)->indices_of_assigned_points().end())
             {
                 const Point_with_normal &p = *(points.begin() + (*index_it));
                 plane_points.push_back(p);
                 index_it++;
             }
             Kernel::Plane_3 plane3d=static_cast<Kernel::Plane_3>(*plane);
             planes.push_back(plane3d);
             plane_num++;
         }
         planes_points.push_back(plane_points);
         it++;
    }
    return plane_num;
}
Pwn_vector getPoints(Pwn_vector points, std::vector< unsigned int > vertexIndices,std::vector<int> *neighbor)
{

    Pwn_vector result;
    vector<int> indices;
    for(int i=0;i<vertexIndices.size()-3;i+=3)
    {
       int idx1=vertexIndices.at(i)-1;
       int idx2=vertexIndices.at(i+1)-1;
       int idx3=vertexIndices.at(i+2)-1;
       if(std::find(indices.begin(), indices.end(), idx1)==indices.end())
       {
           indices.push_back(idx1);
           result.push_back(points.at(idx1));
       }
       if(std::find(indices.begin(), indices.end(), idx2)==indices.end())
       {
           indices.push_back(idx2);
           result.push_back(points.at(idx2));
       }
       if(std::find(indices.begin(), indices.end(), idx3)==indices.end())
       {
           indices.push_back(idx3);
           result.push_back(points.at(idx3));
       }
       if(std::find(neighbor[idx1].begin(), neighbor[idx1].end(), idx2)==neighbor[idx1].end())
       {
             neighbor[idx1].push_back(idx2);
       }
       if(std::find(neighbor[idx1].begin(), neighbor[idx1].end(), idx3)==neighbor[idx1].end())
       {
             neighbor[idx1].push_back(idx3);
       }
       if(std::find(neighbor[idx2].begin(), neighbor[idx2].end(), idx1)==neighbor[idx2].end())
       {
             neighbor[idx2].push_back(idx1);
       }
       if(std::find(neighbor[idx2].begin(), neighbor[idx2].end(), idx3)==neighbor[idx2].end())
       {
             neighbor[idx2].push_back(idx3);
       }
        if(std::find(neighbor[idx3].begin(), neighbor[idx3].end(), idx1)==neighbor[idx3].end())
        {
             neighbor[idx3].push_back(idx1);
        }
         if(std::find(neighbor[idx3].begin(), neighbor[idx3].end(), idx2)==neighbor[idx3].end())
        {
             neighbor[idx3].push_back(idx2);
        }
    }
    return result;
}

bool check_snap(int pt,std::vector<int>* neighbor,int* vertex_snapped)
{

    for(int index=0;index<neighbor[pt].size();index++)
    {
        int point_index=neighbor[pt].at(index);
        int plane_index=vertex_snapped[point_index];
        if(neighbor[pt].size()==0){
            continue;
        }
        if(vertex_snapped[pt]!=plane_index&&plane_index!=-1)
        {
            return true;
        }
    }
    return false;
}
int if_snap(std::vector<int> neighbor,int* vertex_snapped)
{
    vector<int> neighbor_planes;
    vector<int> num_neighbor_planes;
    for(int index=0;index<neighbor.size();index++)
    {
            int point_index=neighbor.at(index);
            int plane_index=vertex_snapped[point_index];
            if(plane_index!=-1)
            {
                std::vector<int>::iterator it=std::find(neighbor_planes.begin(), neighbor_planes.end(), plane_index);
                if(it==neighbor_planes.end())
                {
                    neighbor_planes.push_back(plane_index);
                    num_neighbor_planes.push_back(0);
                }
                else
                {
                   auto pos = std::distance(neighbor_planes.begin(), it);
                   num_neighbor_planes.at(pos)++;
                 }
            }
    }
    for(int plane_index=0;plane_index<neighbor_planes.size();plane_index++)
    {
        int plane_id=neighbor_planes.at(plane_index);
        if(num_neighbor_planes.at(plane_index)>0.5*neighbor.size())
        {
            return plane_id;
        }
    }
    return -1;
}
bool removeSpikes(Pwn_vector points,std::vector<Kernel::Plane_3> planes,Point_with_normal* temp_projected_points,std::vector<int>* neighbor,int* vertex_snapped,map<string,int> out_map )
{
    std::vector<int> allpoints;
//    int *segment_neighbor;
//    int num_planes= planes.size();
//    segment_neighbor=new int[num_planes];
    for(int i=0;i<points.size();i++)
    {
        string key= gen_key_bucket(points.at(i).first);
        int original_index= out_map[key];

        if(check_snap(original_index,neighbor,vertex_snapped))
        {
            allpoints.push_back(original_index);
        }
    }
    int *plane_num;
    plane_num=new int [planes.size()];
    /*for(int i=0;i<allpoints.size();i++)
    {
        int pointIndex=allpoints.at(i);
        //allpoints.pop_back();
        //std::vector<int> region;
        //region.push_back(seed);
        //for (int pt_index=0;pt_index<region.size();pt_index++)
        //{
            //int pointIndex= region.at(0);
            int planeIndex=if_snap(neighbor[pointIndex],vertex_snapped);
            if(planeIndex!=-1)
            {
                cout<<allpoints.size()-i<<" left"<<endl;
                snap_single_point(planeIndex,points.at(pointIndex),planes.at(planeIndex),temp_projected_points,vertex_snapped,out_map);
                //allpoints.erase(std::remove(allpoints.begin(), allpoints.end(), pointIndex), allpoints.end());
                //add all its neighbors
                for(int i=0;i<neighbor[pointIndex].size();i++)
                {
                    int neighborIndex=neighbor[pointIndex].at(i);
                    if(std::find(region.begin(), region.end(),neighborIndex)==region.end())
                    {
                        region.push_back(neighborIndex);
                    }
                    //allpoints.erase(std::remove(allpoints.begin(), allpoints.end(), neighborIndex), allpoints.end());
                }
            }

        //}

    }*/
    for(int i=0;i<allpoints.size();i++){
        vector<int> neighbor_planes;
        vector<int> num_neighbor_planes;
        int pt=allpoints.at(i);
        for(int index=0;index<neighbor[pt].size();index++)
        {
                int point_index=neighbor[pt].at(index);
                int plane_index=vertex_snapped[point_index];
                if(plane_index!=-1)
                {
                    std::vector<int>::iterator it=std::find(neighbor_planes.begin(), neighbor_planes.end(), plane_index);
                    if(it==neighbor_planes.end())
                    {
                        neighbor_planes.push_back(plane_index);
                        num_neighbor_planes.push_back(1);
                    }
                    else
                    {
                       auto pos = std::distance(neighbor_planes.begin(), it);
                       num_neighbor_planes.at(pos)=num_neighbor_planes.at(pos)+1;
                       if(num_neighbor_planes.at(pos)>0.5*neighbor[pt].size())
                       {
                           snap_single_point(plane_index,points.at(pt),planes.at(plane_index),temp_projected_points,vertex_snapped,out_map);
                           break;
                       }
                     }
                }
        }

   }




}

