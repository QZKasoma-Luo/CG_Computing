////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
// Shortcut to avoid  everywhere, DO NOT USE IN .h
using namespace Eigen;
using namespace std;
////////////////////////////////////////////////////////////////////////////////

const std::string root_path = DATA_DIR;

// Computes the determinant of the matrix whose columns are the vector u and v
double inline det(const Vector2d &u, const Vector2d &v)
{
    // TODO
    double determinant = u.x() * v.y() - u.y() * v.x();
    return determinant;
}

int sign(double value){
    if(value > 0.0) return 1; //return 1
    if(value < 0.0) return -1;
    return 0; //return 0 for collinear condition 
}

// Return true iff [a,b] intersects [c,d]
bool intersect_segment(const Vector2d &a, const Vector2d &b, const Vector2d &c, const Vector2d &d)
{
    // TODO
    bool flag = true;
    int orientation_CtoAB = sign(det(b-a, c-b)); //Calculates the orientation of point C to the segment AB
    int orientation_DtoAB = sign(det(b-a, d-b)); //Calculates the orientation of point D to the segment AB
    int orientation_AtoCD = sign(det(d-c, a-d)); //Calculates the orientation of point A to the segment CD
    int orientation_BtoCD = sign(det(d-c, b-d)); //Calculates the orientation of point B to the segment CD

    if(orientation_CtoAB != orientation_DtoAB && orientation_AtoCD != orientation_BtoCD) return flag;

    if (orientation_CtoAB == 0){
        bool isInXBoundary_C = (c.x() <= max(a.x(), b.x())) && (c.x() >= min(a.x(), b.x())); //check if the point is within the boundary of x-axis
        bool isinYBoundary_C = (c.y() <= max(a.y(), b.y())) && (c.y() >= min(a.y(), b.y())); // check if the point is within the boundary of y-axis
        if(isInXBoundary_C && isinYBoundary_C){
            return flag;
        }
    };
    if (orientation_DtoAB == 0){
        bool isInXBoundary_D = (d.x() <= max(a.x(), b.x())) && (d.x() >= min(a.x(), b.x()));
        bool isinYBoundary_D = (d.y() <= max(a.y(), b.y())) && (d.y() >= min(a.y(), b.y()));
        if(isInXBoundary_D && isinYBoundary_D){
            return flag;
        }
    } 
    if (orientation_AtoCD == 0){
        bool isInXBoundary_A = (a.x() <= max(c.x(), d.x())) && (a.x() >= min(c.x(), d.x()));
        bool isinYBoundary_A = (a.y() <= max(c.y(), d.y())) && (a.y() >= min(c.y(), d.y()));
        if(isInXBoundary_A && isinYBoundary_A){
            return flag;
        }
    }
    if (orientation_BtoCD == 0){
        bool isInXBoundary_B = (b.x() <= max(c.x(), d.x())) && (c.x() >= min(c.x(), d.x()));
        bool isinYBoundary_B = (b.y() <= max(c.y(), d.y())) && (c.y() >= min(c.y(), d.y()));
        if(isInXBoundary_B && isinYBoundary_B){
            return flag;
        }
    }

    flag = false;
    return flag;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const std::vector<Vector2d> &poly, const Vector2d &query)
{
    // 1. Compute bounding box and set coordinate of a point outside the polygon
    // TODO
    double minX = poly[0].x(), maxX = poly[0].x();
    double minY = poly[0].y(), maxY = poly[0].y();

    for (const auto& p : poly) {
        minX = min(minX, p.x());
        maxX = max(maxX, p.x());
        minY = min(minY, p.y());
        maxY = max(maxY, p.y());
    }
    Vector2d outside(maxX+1, maxY+1);
    // 2. Cast a ray from the query point to the 'outside' point, count number of intersections
    // TODO
    int intersect_count = 0;
    for(int i = 0; i < poly.size(); i++){
        auto& current = poly[i];
        auto& next = poly[(i+1) % poly.size()];  // Ensure the polygon is closed
        if (intersect_segment(query, outside, current, next)) { //if the point cross one of the boundary
            intersect_count++;
        }
    }
    //odd num of intersection means the point is in the boundary
    return intersect_count%2 == 1;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Vector2d> load_xyz(const std::string &filename)
{
    std::vector<Vector2d> points;
    std::ifstream in(filename);
    // TODO
    int sumPoints = 0;
    in >> sumPoints; //read the first line of the .xyz file to get how many points we have

    double x,y,z; //cerate xyz variables to save the coordinate 

    for(int i = 0; i < sumPoints; i++){
        in >> x >> y >> z;
        points.push_back(Vector2d(x,y)); //push each points coordinate to the points array
    }

    return points;
}

void save_xyz(const std::string &filename, const std::vector<Vector2d> &points)
{
    // TODO
    ofstream outPut(filename); // Create the file as the filename
    if(!outPut){
        cerr << "file cannot be cerated or writing:" << filename << endl;
        return;
    }

    for(const auto& point : points){ //save the point to the file, 
        outPut << point[0] << " " << point[1] << " 0\n"; //only save the x and y, set z to zero
    }
    
}

std::vector<Vector2d> load_obj(const std::string &filename)
{
    std::ifstream in(filename);
    std::vector<Vector2d> points;
    std::vector<Vector2d> poly;
    char key;
    while (in >> key)
    {
        if (key == 'v')
        {
            double x, y, z;
            in >> x >> y >> z;
            points.push_back(Vector2d(x, y));
        }
        else if (key == 'f')
        {
            std::string line;
            std::getline(in, line);
            std::istringstream ss(line);
            int id;
            while (ss >> id)
            {
                poly.push_back(points[id - 1]);
            }
        }
    }
    return poly;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    const std::string points_path = root_path + "/points.xyz";
    const std::string poly_path = root_path + "/polygon.obj";

    std::vector<Vector2d> points = load_xyz(points_path);

    ////////////////////////////////////////////////////////////////////////////////
    //Point in polygon
    std::vector<Vector2d> poly = load_obj(poly_path);
    std::vector<Vector2d> result;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (is_inside(poly, points[i]))
        {
            result.push_back(points[i]);
        }
    }
    save_xyz("output.xyz", result);

    return 0;
}
