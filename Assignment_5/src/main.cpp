// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

// Utilities for the Assignment
#include "raster.h"

#include <gif.h>
#include <fstream>

#include <Eigen/Geometry>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

// Image height
const int H = 480;

// Camera settings
const double near_plane = 1.5; // AKA focal length
const double far_plane = near_plane * 100;
const double field_of_view = 0.7854; // 45 degrees
const double aspect_ratio = 1.5;
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 3);
const Vector3d camera_gaze(0, 0, -1);
const Vector3d camera_top(0, 1, 0);

// Object
const std::string data_dir = DATA_DIR;
const std::string mesh_filename(data_dir + "bunny.off");
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)

// Material for the object
const Vector3d obj_diffuse_color(0.5, 0.5, 0.5);
const Vector3d obj_specular_color(0.2, 0.2, 0.2);
const double obj_specular_exponent = 256.0;

// Lights
std::vector<Vector3d> light_positions;
std::vector<Vector3d> light_colors;
// Ambient light
const Vector3d ambient_light(0.3, 0.3, 0.3);

// Fills the different arrays
void setup_scene()
{
    // Loads file
    std::ifstream in(mesh_filename);
    if (!in.good())
    {
        std::cerr << "Invalid file " << mesh_filename << std::endl;
        exit(1);
    }
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

    // Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16);
}

void build_uniform(UniformAttributes &uniform)
{
    // TODO: setup uniform

    // TODO: setup camera, compute w, u, v
    Vector3d w = -(camera_gaze.normalized());
    Vector3d u = camera_top.cross(w).normalized();
    Vector3d v = w.cross(u);

    // TODO: compute the camera transformation
    Matrix4d camera_transformation = Matrix4d::Zero();
    camera_transformation.block<3, 1>(0, 0) = u;
    camera_transformation.block<3, 1>(0, 1) = v;
    camera_transformation.block<3, 1>(0, 2) = w;
    camera_transformation.block<3, 1>(0, 3) = camera_position;
    camera_transformation(3, 3) = 1.0;

    // TODO: setup projection matrix
    double t = near_plane * tan(field_of_view / 2);
    double f = -far_plane;
    double n = -near_plane;
    double r = t * aspect_ratio;
    double l = -r;
    double b = -t;
    Matrix4d P;
    if (is_perspective)
    {
        // TODO setup prespective camera
        P << 2 * near_plane / (r - l), 0, (r + l) / (r - l), 0,
            0, 2 * near_plane / (t - b), (t + b) / (t - b), 0,
            0, 0, (f + n) / (n - f), 2 * f * n / (n - f),
            0, 0, -1, 0;
    }
    else
    {
        P << 2 / (r - l), 0, 0, -(r + l) / (r - l),
            0, 2 / (t - b), 0, -(t + b) / (t - b),
            0, 0, -2 / (f - n), -(f + n) / (f - n),
            0, 0, 0, 1;
    }

    uniform.setMvpMatrix(P * camera_transformation.inverse());
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: fill the shader
        VertexAttributes out_va;
        // Divide all components by the w component for perspective division
        out_va.position = uniform.getMvpMatrix() * va.position;
        return out_va;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous)
    {
        // TODO: fill the shader
        int r = static_cast<int>(fa.color[0] * 255);
        int g = static_cast<int>(fa.color[1] * 255);
        int b = static_cast<int>(fa.color[2] * 255);
        int a = static_cast<int>(fa.color[3] * 255);
        return FrameBufferAttributes(r, g, b, a);
    };

    // std::vector<VertexAttributes> vertex_attributes;
    // TODO: build the vertex attributes from vertices and facets
    std::vector<VertexAttributes> vertex_attributes;
    vertex_attributes.reserve(3 * facets.rows()); // 根据需要调整大小，避免在循环中分配内存

    for (int i = 0; i < facets.rows(); ++i)
    {
        vertex_attributes.clear();
        vertex_attributes.reserve(3);

        for (int j = 0; j < 3; ++j)
        {
            int idx = facets(i, j);
            Eigen::Vector3d v = vertices.row(idx);
            vertex_attributes.emplace_back(v[0], v[1], v[2]);
        }

        rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
    }
}

Eigen::Matrix4d compute_rotation(const double alpha)
{
    // TODO: Compute the rotation matrix of angle alpha on the y axis around the object barycenter
    Eigen::Matrix4d res;
    double barycenter_x = 1.0, barycenter_y = 2.0, barycenter_z = 3.0;

    Eigen::Matrix4d translate_to_origin = Eigen::Matrix4d::Identity();
    translate_to_origin(0, 3) = -barycenter_x;
    translate_to_origin(1, 3) = -barycenter_y;
    translate_to_origin(2, 3) = -barycenter_z;

    Eigen::Matrix4d translate_back = Eigen::Matrix4d::Identity();
    translate_back(0, 3) = barycenter_x;
    translate_back(1, 3) = barycenter_y;
    translate_back(2, 3) = barycenter_z;

    Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
    rotation(0, 0) = cos(alpha);
    rotation(0, 2) = sin(alpha);
    rotation(2, 0) = -sin(alpha);
    rotation(2, 2) = cos(alpha);

    // Combine transformations
    res = translate_back * rotation * translate_to_origin;
    return res;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    Matrix4d trafo = compute_rotation(alpha);
    uniform.setMvpMatrix(uniform.getMvpMatrix() * trafo);

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: fill the shader
        VertexAttributes va_out;
        va_out.position = uniform.getMvpMatrix() * va.position;
        return va_out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous)
    {
        // TODO: fill the shader
        return FrameBufferAttributes(fa.color[0], fa.color[1], fa.color[2], fa.color[3]);
    };

    std::vector<VertexAttributes> vertex_attributes;

    // TODO: generate the vertex attributes for the edges and rasterize the lines
    // TODO: use the transformation matrix

    // Estimate number of lines: 3 lines per triangle
    vertex_attributes.reserve(facets.rows() * 3 * 2); // Each triangle has 3 edges, 2 vertices per edge

    // Apply the transformation matrix directly in the loop
    Matrix4d transform = uniform.getMvpMatrix(); // Transformation matrix from uniform

    for (int i = 0; i < facets.rows(); i++)
    {
        // get triangle
        Vector3i triangle = facets.row(i);

        // get vertices of triangle
        Vector3d a = vertices.row(triangle[0]);
        Vector3d b = vertices.row(triangle[1]);
        Vector3d c = vertices.row(triangle[2]);

        // add lines of triangle to vertex attributes

        // ab line
        vertex_attributes.push_back(VertexAttributes(a[0], a[1], a[2]));
        vertex_attributes.push_back(VertexAttributes(b[0], b[1], b[2]));

        // ac line
        vertex_attributes.push_back(VertexAttributes(a[0], a[1], a[2]));
        vertex_attributes.push_back(VertexAttributes(c[0], c[1], c[2]));

        // bc line
        vertex_attributes.push_back(VertexAttributes(b[0], b[1], b[2]));
        vertex_attributes.push_back(VertexAttributes(c[0], c[1], c[2]));
    }

    rasterize_lines(program, uniform, vertex_attributes, 0.5, frameBuffer);
}

void get_shading_program(Program &program)
{
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: transform the position and the normal
        // TODO: compute the correct lighting
        return va;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: create the correct fragment
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous)
    {
        // TODO: implement the depth check
        return FrameBufferAttributes(fa.color[0], fa.color[1], fa.color[2], fa.color[3]);
    };
}

void flat_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);
    Eigen::Matrix4d trafo = compute_rotation(alpha);

    std::vector<VertexAttributes> vertex_attributes;
    // TODO: compute the normals
    // TODO: set material colors

    for (int i = 0; i < facets.rows(); i++)
    {
        // get triangle
        Vector3i triangle = facets.row(i);

        // get vertices of triangle
        Vector3d a = vertices.row(triangle[0]);
        Vector3d b = vertices.row(triangle[1]);
        Vector3d c = vertices.row(triangle[2]);

        // compute normal of triangle
        Vector3d N = ((b - a).cross(c - a)).normalized();

        // create vertices
        VertexAttributes va(a[0], a[1], a[2]);
        VertexAttributes vb(b[0], b[1], b[2]);
        VertexAttributes vc(c[0], c[1], c[2]);

        // add normals to vertices
        va.normal = N;
        vb.normal = N;
        vc.normal = N;

        // set material colors
        va.color << 1, 1, 1;
        vb.color << 1, 1, 1;
        vc.color << 1, 1, 1;

        // add to vertex attributes
        vertex_attributes.push_back(va);
        vertex_attributes.push_back(vb);
        vertex_attributes.push_back(vc);
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

void pv_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    Eigen::Matrix4d trafo = compute_rotation(alpha);

    // TODO: compute the vertex normals as vertex normal average

    std::vector<VertexAttributes> vertex_attributes;
    // TODO: create vertex attributes
    // TODO: set material colors

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

int main(int argc, char *argv[])
{
    setup_scene();

    int W = H * aspect_ratio;
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(W, H);
    vector<uint8_t> image;

    simple_render(frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("simple.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    wireframe_render(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("wireframe.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    flat_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("flat_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    pv_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("pv_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    // TODO: add the animation

    return 0;
}
