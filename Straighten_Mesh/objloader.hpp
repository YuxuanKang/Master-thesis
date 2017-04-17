#ifndef OBJLOADER_H
#define OBJLOADER_H
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <utility> // defines std::pair
#include <list>
#include <CGAL/mst_orient_normals.h>
#include <math.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/IO/read_xyz_points.h>
#include<list>
#include "utilities.hpp"
using namespace std;
// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;



class vec3
{
    public:
        double x,y,z;
    vec3(){};
    vec3(double in_x,double in_y,double in_z):x(in_x),y(in_y),z(in_z){};
    vec3 normalize(){
        double len= sqrt(x*x+y*y+z*z);
        vec3 re(0,0,0);
        if (len!=0){
            re.x=x/len;
            re.y=y/len;
            re.z=z/len;
            return re;
        }
        else{
            re.x=0;
            re.y=0;
            re.z=0;
            return re;
        }

    }
    vec3 operator +(vec3 v){
        vec3 re(0,0,0);
        re.x=re.x+v.x;
        re.y=re.y+v.y;
        re.z=re.z+v.z;
        return re;

    }
    double length(){
        return sqrt(x*x+y*y+z*z);
    }

};
struct vec2{
   public:
    double x,y;
    vec2(){};
    vec2(double in_x,double in_y):x(in_x),y(in_y){};
};

std::vector<vec3 > temp_vertices;
std::vector<vec2 > temp_uvs;
std::vector<vec3 > temp_normals;
#include "textureoperator.hpp"

bool OBJtoPwn(std::vector< vec3 > vertice,std::vector<vec3> normals, Pwn_vector & points)
{
    for (int i=0;i<vertice.size();i++){
        vec3 vec= vertice.at(i);
        vec3 nor= normals.at(i);
        Kernel::Point_3 point3(vec.x,vec.y,vec.z);
        Kernel::Vector_3 normal3(nor.x,nor.y,nor.z);
        std::pair<Kernel::Point_3, Kernel::Vector_3>  pwn;
        pwn.first=point3;
        pwn.second=normal3;
        points.push_back(pwn);
    }
    return true;
}

Pwn_vector loadOBJ(const char* path,std::vector< unsigned int > & vertexIndices,std::vector< unsigned int > & uvIndices,std::vector< unsigned int > & normalIndices,std::vector<vec2 > &out_uvs,std::vector <string> &texture_map){
    // vertexIndices:  Indices of vertices for faces
    // uvIndices: Indices of pixels in images
    // normalIndices:  empty of files without normals
    // points: vector of points, first is [x,y,z] coordinates
    std::list<Point_with_normal> points_list;
    Pwn_vector points;
    FILE * file = fopen(path, "r");
    if( file == NULL ){
        printf("Impossible to open the file !\n");
    }
   int normal_num=0;
   string texture_index;
    while( 1 ){

        char lineHeader[128];
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop.

        // else : parse lineHeader
        char str1[11];
        char str2[1];
        if (strcmp(lineHeader,"Normal")==0){
            fscanf(file,"%s %s %i",str1,str2,&normal_num);
        }
        if ( strcmp( lineHeader, "v" ) == 0 ){
            vec3 vertex;
            double temp_x,temp_y,temp_z;
            fscanf(file, "%lf %lf %lf\n", &vertex.x, &vertex.y, &vertex.z );
            temp_vertices.push_back(vertex);
            Kernel::Point_3 pt(vertex.x,vertex.y,vertex.z);
            Kernel::Vector_3 vec(0,0,0);
            Point_with_normal pv;
            pv.first=pt;
            pv.second=vec;
            points_list.push_back(pv);
        }else if ( strcmp( lineHeader, "vt" ) == 0 ){
            vec2 uv;
            fscanf(file, "%lf %lf\n", &uv.x, &uv.y );
            temp_uvs.push_back(uv);
        }else if ( strcmp( lineHeader, "vn" ) == 0 ){
            vec3 normal;
            fscanf(file, "%lf %lf %lf\n", &normal.x, &normal.y, &normal.z );
            temp_normals.push_back(normal);
        }else if(strcmp(lineHeader,"usemtl")==0)
        {
            char texture[128];
            fscanf(file,"%s",texture);
            texture_index.assign(texture);
        }
        else if ( strcmp( lineHeader, "f" ) == 0 ){
            unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
            // if there is no normal in the file, read all the face information and calculate normal. Then add calculated normal to the file
            if (normal_num==0){
                /*int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0],&normalIndex[0], &vertexIndex[1], &uvIndex[1],&normalIndex[1] ,&vertexIndex[2], &uvIndex[2],&normalIndex[2]);
                if (matches != 9){
                    printf("File can't be read by our simple parser : ( Try exporting with other options\n");
                    return false;x
                    }*/
                int matches = fscanf(file, "%d/%d %d/%d %d/%d\n", &vertexIndex[0], &uvIndex[0],&vertexIndex[1], &uvIndex[1],&vertexIndex[2], &uvIndex[2]);
                if (matches != 6)
                {
                    printf("File can't be read by our simple parser : ( Try exporting with other options\n");
                }
                vertexIndices.push_back(vertexIndex[0]);
                vertexIndices.push_back(vertexIndex[1]);
                vertexIndices.push_back(vertexIndex[2]);
                uvIndices    .push_back(uvIndex[0]);
                uvIndices    .push_back(uvIndex[1]);
                uvIndices    .push_back(uvIndex[2]);
                /*normalIndices.push_back(normalIndex[0]);
                normalIndices.push_back(normalIndex[1]);
                normalIndices.push_back(normalIndex[2]);
                */
                if(texture_index.empty()==false)
                {
                    texture_map.push_back(texture_index);
                }
            }
            // if there is already normal in the file, just combine vertice information and normal information, transfer the format to CGAL format
            else{
                /*int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0],&normalIndex[0], &vertexIndex[1], &uvIndex[1],&normalIndex[1] ,&vertexIndex[2], &uvIndex[2],&normalIndex[2]);
                if (matches != 9){
                    printf("File can't be read by our simple parser : ( Try exporting with other options\n");
                    return false;
                    }*/
                int matches = fscanf(file, "%d/%d %d/%d %d/%d\n", &vertexIndex[0], &uvIndex[0],&vertexIndex[1], &uvIndex[1],&vertexIndex[2], &uvIndex[2]);
                if (matches != 6){
                    printf("File can't be read by our simple parser : ( Try exporting with other options\n");
                    }
                vertexIndices.push_back(vertexIndex[0]);
                vertexIndices.push_back(vertexIndex[1]);
                vertexIndices.push_back(vertexIndex[2]);
                uvIndices    .push_back(uvIndex[0]);
                uvIndices    .push_back(uvIndex[1]);
                uvIndices    .push_back(uvIndex[2]);
                /*normalIndices.push_back(normalIndex[0]);
                normalIndices.push_back(normalIndex[1]);
                normalIndices.push_back(normalIndex[2]);*/
                if(texture_index.empty()==false)
                {
                    texture_map.push_back(texture_index);
                }
            }

        }
    }
    // --------If the file doesn't contain normal information, iterate all vertices, find all incident faces to each vertex, calculate normal for each vertex----------

    if (normal_num==0){
        /*std::vector<vec3> incidentFaces;
        vec3* normals ;
        normals= new vec3[temp_vertices.size()];
        for (int i=0;i<temp_vertices.size();i++){
            normals[i]=vec3(0,0,0);
        }
        for (int vertexindex=0;vertexindex<vertexIndices.size()-3;vertexindex+=3)
        {
            int idx1=vertexIndices.at(vertexindex)-1;
            int idx2=vertexIndices.at(vertexindex+1)-1;
            int idx3=vertexIndices.at(vertexindex+2)-1;
            vec3 p1=temp_vertices.at(idx1);
            vec3 p2=temp_vertices.at(idx2);
            vec3 p3=temp_vertices.at(idx3);
            vec3 normal;
            double a1= p2.x-p1.x;
            double a2= p2.y-p1.y;
            double a3= p2.z-p1.z;
            double b1= p3.x-p1.x;
            double b2= p3.y-p1.y;
            double b3= p3.z-p1.z;
            normal.x=  a2*b3-a3*b2;
            normal.y=  a3*b1-a1*b3;
            normal.z=  a1*b2-a2*b1;
            normal.normalize();
            normals[idx1]=normals[idx1]+normal;
            normals[idx2]=normals[idx2]+normal;
            normals[idx3]=normals[idx3]+normal;

        }
        */
         Pwn_vector points_vec_original{std::begin(points_list),std::end(points_list)};
        const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
            CGAL::pca_estimate_normals<Concurrency_tag>(points_list.begin(), points_list.end(),
                                       CGAL::First_of_pair_property_map<Point_with_normal>(),
                                       CGAL::Second_of_pair_property_map<Point_with_normal>(),
                                       nb_neighbors);
            /*std::list<Point_with_normal>::iterator unoriented_points_begin =
                  CGAL::mst_orient_normals(points_list.begin(), points_list.end(),
                                             CGAL::First_of_pair_property_map<Point_with_normal>(),
                                             CGAL::Second_of_pair_property_map<Point_with_normal>(),
                                             nb_neighbors);*/
           Pwn_vector points_vec{std::begin(points_list),std::end(points_list)};
           map<string,int> points_map=ConstructMap(points_vec);
           for(int i=0;i<points_vec_original.size();i++)
           {
               Kernel::Point_3 pt=points_vec_original.at(i).first;
               string key = gen_key_bucket(pt);
               int index=points_map[key];
               Kernel::Vector_3 normal=points_vec.at(index).second;
               Point_with_normal pn(pt,normal);
               points.push_back(pn);
           }
    }
    else
    {
       OBJtoPwn(temp_vertices,temp_normals,points);
    }
    //OBJtoPwn(temp_vertices,temp_normals,points);
    for(int i=0;i<temp_uvs.size();i++){
        vec2 uv(temp_uvs.at(i).x,temp_uvs.at(i).y);
        out_uvs.push_back(uv);
    }
    return points;
}
bool writePlanes(const char* path,std::vector <vec3 >  in_vertices){
    FILE *file= fopen(path, "w+");
    fprintf(file,"%s\n","# Planes generated by CGAL ransac algorithm");
    for (int i=0; i<in_vertices.size();i++){
        fprintf(file,"%s %.6f %.6f %.6f \n","v",in_vertices.at(i).x,in_vertices.at(i).y,in_vertices.at(i).z);
    }
    fprintf(file,"%s","usemtl fx-i-0\n");
    for (int i=1; i<=in_vertices.size();i+=4){
        fprintf(file,"%s %i %i %i \n","f",i,i+1,i+2);
        fprintf(file,"%s %i %i %i \n","f",i,i+2,i+3);
    }
    fclose(file);
}


bool writeOBJ(const char* path,std::vector< unsigned int >  vertexIndices,std::vector< unsigned int > uvIndices,std::vector< unsigned int > normalIndices,Pwn_vector points,std::vector <vec2 > uvs,std::vector<int> face_classification,int* vertex_snapped,int *vertex_region,vector<string> texture_map){
    FILE *file= fopen(path, "w+");
    fprintf(file,"%s\n","# Snapped mesh by CGAL ransac algorithm");
    fprintf(file,"%s\n","# COORDINATE_SYSTEM:");
    fprintf(file,"%s%i\n","# Number of Geometry Coordinates  : ",points.size());
    fprintf(file,"%s%i\n","# Number of Texture  Coordinates  : ",uvs.size());
    fprintf(file,"%s%i\n","# Number of Normal   Coordinates  : ",points.size());
    fprintf(file,"%s\n","mtllib ./cluster.mtl");

    for (int i=0; i<points.size();i++){
        switch(vertex_region[i]%8){
        case -1:
            fprintf(file,"%s","usemtl cluster-1\n");
            break;
        case 0:
                    fprintf(file,"%s","usemtl cluster0\n");
            break;
        case 1:
                fprintf(file,"%s","usemtl cluster1\n");
        break;
        case 2:
                fprintf(file,"%s","usemtl cluster2\n");
        break;
        case 3:
                fprintf(file,"%s","usemtl cluster3\n");
        break;
        case 4:
                fprintf(file,"%s","usemtl cluster4\n");
        break;
        case 5:
                fprintf(file,"%s","usemtl cluster5\n");
        break;
        case 6:
                fprintf(file,"%s","usemtl cluster6\n");
        break;
        case 7:
                fprintf(file,"%s","usemtl cluster7\n");
        break;
        }

        if (vertex_snapped[i]==true){
            fprintf(file,"%s\n","mtllib ./vertex_classification.mtl");
            fprintf(file,"%s","usemtl class1\n");
        }

        fprintf(file,"%s %.6lf %.6lf %.6lf \n","v",points.at(i).first.x(),points.at(i).first.y(),points.at(i).first.z());
    }
    for (int i=0; i<uvs.size();i++){
        fprintf(file,"%s %.6f %.6f \n","vt",uvs.at(i).x,uvs.at(i).y);
    }
    for (int i=0; i<points.size();i++){
        fprintf(file,"%s %.6f %.6f %.6f \n","vn",points.at(i).second.x(),points.at(i).second.y(),points.at(i).second.z());
    }
    //fprintf(file,"%s\n","mtllib ./materials.mtl");
    int face_class=4;
    string previous_texture_index="";
    fprintf(file,"%s\n","mtllib ./face_classification.mtl");
    for (int i=0; i<vertexIndices.size()-3;i+=3){
        int face_index=i/3;
        if (face_classification.at(face_index)==face_class){
              //do nothing
        }
        else if (face_classification.at(face_index)==0){
            fprintf(file,"%s","usemtl class0\n");
            face_class=face_classification.at(face_index);
        }
        else if (face_classification.at(face_index)==1){
            fprintf(file,"%s","usemtl class1\n");
            face_class=face_classification.at(face_index);
        }
        else if (face_classification.at(face_index)==2){
            fprintf(file,"%s","usemtl class2\n");
            face_class=face_classification.at(face_index);
        }
        else if (face_classification.at(face_index)==3){
            fprintf(file,"%s","usemtl class3\n");
            face_class=face_classification.at(face_index);
        }


        int vertex1=vertexIndices.at(i);
        int vertex2=vertexIndices.at(i+1);
        int vertex3=vertexIndices.at(i+2);

            /*string texture_index= texture_map.at(face_index);
            if(texture_index!=previous_texture_index)
            {
                fprintf(file,"%s %s\n","usemtl",texture_index.c_str());
                previous_texture_index=texture_index;
            }*/

        fprintf(file,"%s %d/%d %d/%d %d/%d \n","f",
                vertex1,uvIndices.at(i),
                vertex2,uvIndices.at(i+1),
                vertex3,uvIndices.at(i+2));
    }
    fclose(file);
}


bool writeOBJtest(const char* path,std::vector< unsigned int >  vertexIndices,std::vector< unsigned int > uvIndices,std::vector< unsigned int > normalIndices,Pwn_vector points,std::vector <vec2 > uvs,vector<string> texture_map){
    FILE *file= fopen(path, "w+");
    fprintf(file,"%s\n","# Snapped mesh by CGAL ransac algorithm");
    fprintf(file,"%s\n","# COORDINATE_SYSTEM:");
    fprintf(file,"%s\n","mtllib ./materials.mtl");
    fprintf(file,"%s%i\n","# Number of Geometry Coordinates  : ",points.size());
    fprintf(file,"%s%i\n","# Number of Texture  Coordinates  : ",uvs.size());
    fprintf(file,"%s%i\n","# Number of Normal   Coordinates  : ",points.size());
    for (int i=0; i<points.size();i++)
    {
        fprintf(file,"%s %.6f %.6f %.6f\n","v",points.at(i).first.x(),points.at(i).first.y(),points.at(i).first.z());
    }
    for (int i=0; i<uvs.size();i++)
    {
        fprintf(file,"%s %.6f %.6f\n","vt",uvs.at(i).x,uvs.at(i).y);
    }
    for (int i=0; i<points.size();i++)
    {
        fprintf(file,"%s %.6f %.6f %.6f\n","vn",points.at(i).second.x(),points.at(i).second.y(),points.at(i).second.z());
    }
    string previous_texture_index="";
    for (int i=0; i<vertexIndices.size()-3;i+=3)
    {
        int face_index=i/3;
        int v1=vertexIndices.at(i)-1;
        int v2=vertexIndices.at(i+1)-1;
        int v3=vertexIndices.at(i+2)-1;
        if (points.at(v1).first.x()>435&&points.at(v1).first.x()<460&&points.at(v1).first.y()>6530&&points.at(v1).first.y()<6550&&points.at(v1).first.z()>1)
        {

            int vertex1=vertexIndices.at(i);
            int vertex2=vertexIndices.at(i+1);
            int vertex3=vertexIndices.at(i+2);
            string texture_index= texture_map.at(face_index);
            if(texture_index!=previous_texture_index)
            {
                 fprintf(file,"%s %s\n","usemtl",texture_index.c_str());
                 previous_texture_index=texture_index;
             }
            fprintf(file,"%s %d/%d %d/%d %d/%d \n","f",
                    vertex1,uvIndices.at(i),
                    vertex2,uvIndices.at(i+1),
                    vertex3,uvIndices.at(i+2));
        }
    }
    fclose(file);
}
bool outputVertices(const char* path,Pwn_vector points,int* vertex_snapped)
{
    FILE *file= fopen(path, "w+");
    for(int i=0;i<points.size();i++)
    {
        if(vertex_snapped[i]!=-1)
        {
            fprintf(file,"%s %.6f %.6f %.6f\n","v",points.at(i).first.x(),points.at(i).first.y(),points.at(i).first.z());
        }
    }

    return true;
}

string gen_key_texture(int R,int G, int B) {
  string r = to_string(R);
  string g = std::to_string(G);
  string b = std::to_string(B);
  return (r + " " + g + " " + b);
}

bool writeTexuredVertices(string filename,Pwn_vector points,map<int,RGB> texture )
{
    map<string,RGB> texturemap;
    string filepath1="data/"+filename+".mtl";
    const char* mtlpath=filepath1.c_str();
    string filepath2="data/"+filename+".obj";
    const char* objpath=filepath2.c_str();
    FILE *file= fopen(mtlpath, "w+");
    fprintf(file,"%s","newmtl untextured\n");
    fprintf(file,"%s %.4f %.4f %.4f\n","Ka",1.0000,1.0000,1.0000);
    fprintf(file,"%s %.4f %.4f %.4f\n","Kd",1.0000,1.0000,1.0000);
    fprintf(file,"%s %.4f %.4f %.4f\n","Ks",1.0000,1.0000,1.0000);
    fprintf(file,"%s %i\n","illum",2);
    fprintf(file,"%s %.4f\n","Ns",60.0000);
    fprintf(file,"\n");
    for(int i=0;i<points.size();i++)
    {
        if(texture.count(i)==1)
        {

            int R=(double)texture[i].R;
            int G=(double)texture[i].G;
            int B=(double)texture[i].B;
            string key=gen_key_texture(R,G,B);
            if(texturemap.count(key)==0)
            {
                RGB rgbvalue(R,G,B);
                texturemap[key]=rgbvalue;
                double r=(double)texture[i].R/255;
                double g=(double)texture[i].G/255;
                double b=(double)texture[i].B/255;
                fprintf(file,"%s%i%i%i\n","newmtl rgb",R,G,B);
                fprintf(file,"%s %.4f %.4f %.4f\n","Ka",r,g,b);
                fprintf(file,"%s %.4f %.4f %.4f\n","Kd",r,g,b);
                fprintf(file,"%s %.4f %.4f %.4f\n","Ks",r,g,b);
                fprintf(file,"%s %i\n","illum",2);
                fprintf(file,"%s %.4f\n","Ns",60.0000);
                fprintf(file,"\n");

            }
        }
    }
    fclose(file);
    FILE *objfile= fopen(objpath, "w+");
    fprintf(file,"%s ./%s%s\n","mtllib",filename.c_str(),".mtl");
    for (int i=0; i<points.size();i++)
    {

        if(texture.count(i)==1)
        {
            int R=(double)texture[i].R;
            int G=(double)texture[i].G;
            int B=(double)texture[i].B;
            fprintf(file,"%s%i%i%i\n","usemtl rgb",R,G,B);
            fprintf(objfile,"%s %.6f %.6f %.6f\n","v",points.at(i).first.x(),points.at(i).first.y(),points.at(i).first.z());
        }

    }
    fclose(objfile);
    return true;
}
#endif
