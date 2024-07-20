////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh

private:
    // builds the bvh recursively
    int build_recursive(const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, int from, int to, int parent, std::vector<int> &triangles);
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "bunny.off");

// Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; // 45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

// Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

// Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
// Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

// Fills the different arrays
void setup_scene()
{
    // Loads file
    std::ifstream in(mesh_filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    // setup tree
    bvh = AABBTree(vertices, facets);

    // Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    // Vector containing the list of tringle indices
    std::vector<int> triangles(F.rows());
    std::iota(triangles.begin(), triangles.end(), 0);

    root = build_recursive(V, F, centroids, 0, triangles.size(), -1, triangles);
}

int AABBTree::build_recursive(const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, int from, int to, int parent, std::vector<int> &triangles)
{
    // Scene is empty, so is the aabb tree
    if (to - from == 0)
    {
        return -1;
    }

    // If there is only 1 triangle left, then we are at a leaf
    if (to - from == 1)
    {
        int index = triangles[from];
        Node leaf_node;
        leaf_node.bbox = bbox_from_triangle(V.row(F(index, 0)), V.row(F(index, 1)), V.row(F(index, 2)));
        leaf_node.parent = parent;
        leaf_node.left = -1;
        leaf_node.right = -1;
        leaf_node.triangle = index;
        nodes.push_back(leaf_node);
        return nodes.size() - 1;
    }

    AlignedBox3d centroid_box;

    // TODO Use AlignedBox3d to find the box around the current centroids
    for (int i = from; i < to; ++i)
    {
        Vector3d x = centroids.row(triangles[i]);
        centroid_box.extend(x);
    }

    // Diagonal of the box
    Vector3d extent = centroid_box.diagonal();

    // TODO find the largest dimension
    int longest_dim = 0;
    if (extent[1] > extent[0])
    {
        longest_dim = 1;
    }
    else if (extent[2] > extent[longest_dim])
    {
        longest_dim = 2;
    }

    // TODO sort centroids along the longest dimension
    std::sort(triangles.begin() + from, triangles.begin() + to, [&](int f1, int f2)
              {
        //TODO sort the **triangles** along the centroid largest dimension
        // return true if triangle f1 comes before triangle f2
        bool flag = false;
        if (centroids(f1, longest_dim) < centroids(f2, longest_dim))
        {
            flag = true;
        }
        return flag; });

    // TODO Create a new internal node and do a recursive call to build the left and right part of the tree
    // TODO finally return the correct index
    int mid = (from + to) / 2;
    Node internal_node;
    internal_node.left = build_recursive(V, F, centroids, from, mid, nodes.size(), triangles);
    internal_node.right = build_recursive(V, F, centroids, mid, to, nodes.size(), triangles);
    internal_node.parent = parent;
    internal_node.bbox = nodes[internal_node.left].bbox.merged(nodes[internal_node.right].bbox);
    internal_node.triangle = -1;
    nodes.push_back(internal_node);
    return nodes.size() - 1;
}

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // TODO
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.

    Vector3d edge_u = b - a;
    Vector3d edge_v = c - a;

    Matrix3d M;
    M.col(0) = edge_u;
    M.col(1) = edge_v;
    M.col(2) = -ray_direction;

    Vector3d M2 = ray_origin - a;

    Vector3d sol = M.colPivHouseholderQr().solve(M2);

    double u = sol[0];
    double v = sol[1];
    double t = sol[2];

    if (u >= 0 && v >= 0 && (u + v) <= 1 && t >= 1e-8)
    {
        p = ray_origin + t * ray_direction;
        N = edge_u.cross(edge_v).normalized();
        return t;
    }

    return -1;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // TODO
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.
    Vector3d inv_direction = ray_direction.cwiseInverse();
    Vector3d t_min = (box.min() - ray_origin).cwiseProduct(inv_direction);
    Vector3d t_max = (box.max() - ray_origin).cwiseProduct(inv_direction);

    for (int i = 0; i < 3; i++)
    {
        if (inv_direction[i] < 0)
        {
            std::swap(t_min[i], t_max[i]);
        }
    }

    double max_t = std::max(std::max(t_min[0], t_min[1]), t_min[2]);
    double min_t = std::min(std::min(t_max[0], t_max[1]), t_max[2]);
    return max_t <= min_t && min_t >= 0;
}
bool method2_BVH_helper(double &nearest_t, const Vector3d &rayOrigin, const Vector3d &rayDirection, AABBTree::Node &node, Vector3d &p, Vector3d &N)
{
    // Check for ray-box intersection
    bool flag = false;
    if (!ray_box_intersection(rayOrigin, rayDirection, node.bbox))
    {
        return flag;
    }

    // If it's an internal node, traverse both children
    if (node.triangle == -1)
    {
        bool leftHit = method2_BVH_helper(nearest_t, rayOrigin, rayDirection, bvh.nodes[node.left], p, N);
        bool rightHit = method2_BVH_helper(nearest_t, rayOrigin, rayDirection, bvh.nodes[node.right], p, N);
        return leftHit || rightHit;
    }

    // Process leaf node: check ray-triangle intersection
    Vector3d tempPoint, tempNormal;
    Vector3d vertexA = vertices.row(facets(node.triangle, 0));
    Vector3d vertexB = vertices.row(facets(node.triangle, 1));
    Vector3d vertexC = vertices.row(facets(node.triangle, 2));
    double t = ray_triangle_intersection(rayOrigin, rayDirection, vertexA, vertexB, vertexC, tempPoint, tempNormal);

    // Update closest intersection if this one is closer
    if (t > 0 && (nearest_t < 0 || t < nearest_t))
    {
        nearest_t = t;
        p = tempPoint;
        N = tempNormal;
        flag = true;
    }

    return flag;
}
// Finds the closest intersecting object returns its index
// In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;
    bool flag = false;
    // TODO
    // Method (1): Traverse every triangle and return the closest hit.
    // double min_t = std::numeric_limits<double>::max();

    // for (int i = 0; i < facets.rows(); i++)
    // {

    //     double t = ray_triangle_intersection(ray_origin, ray_direction, vertices.row(facets(i, 0)).transpose(), vertices.row(facets(i, 1)).transpose(), vertices.row(facets(i, 2)).transpose(), tmp_p, tmp_N);
    //     if (t >= 0 && (t < min_t || min_t < 0))
    //     {
    //         min_t = t;
    //         p = tmp_p;
    //         N = tmp_N;
    //         flag = true;
    //     }
    // }

    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.
    double nearest_t = std::numeric_limits<double>::max();
    flag = method2_BVH_helper(nearest_t, ray_origin, ray_direction, bvh.nodes[bvh.root], p, N);

    return flag;
}

// bonus 3

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction)
{
    // Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        Vector4d diff_color = obj_diffuse_color;

        // Diffuse contribution
        const Vector3d Li = (light_position - p).normalized();
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    // Rendering equation
    Vector4d C = ambient_color + lights_color;

    // Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    // TODO
    double image_y = focal_length * tan(field_of_view / 2);
    double image_x = image_y * aspect_ratio;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}
