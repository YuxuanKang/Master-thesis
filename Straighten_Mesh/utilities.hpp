#ifndef UTILITIES_H
#define UTILITIES_H

#include<string>
using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef Kernel::FT                                           FT;
typedef std::vector<Point_with_normal>                       Pwn_vector;

string gen_key_bucket(Kernel::Point_3 p) {
  string x = to_string(p.x());
  x = x.substr(0, x.find_first_of(".") + 8);
  string y = std::to_string(p.y());
  y = y.substr(0, y.find_first_of(".") + 8);
  string z = std::to_string(p.z());
  z = z.substr(0, z.find_first_of(".") + 8);
  return (x + " " + y + " " + z);
}

map<string,int> ConstructMap(Pwn_vector points){
    map<string,int> out_map;
    for(int i=0;i<points.size();i++){
        string key = gen_key_bucket(points.at(i).first);
        out_map[key]=i;
    }

    return out_map;

}

#endif // UTILITIES_H
