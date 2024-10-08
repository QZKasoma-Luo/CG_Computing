// C++ include
#include <iostream>
#include <string>
#include <vector>

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
    Vector3d w = -1 * (camera_gaze.normalized());
    Vector3d u = (camera_top.cross(w)).normalized();
    Vector3d v = w.cross(u);
    // TODO: compute the camera transformation
    Matrix4d camera_view_transformed;
    camera_view_transformed.setZero(); // Initialize all matrix elements to zero

    // Set the rotation part
    camera_view_transformed.block<3, 1>(0, 0) = u;
    camera_view_transformed.block<3, 1>(0, 1) = v;
    camera_view_transformed.block<3, 1>(0, 2) = w;

    // Set the translation part
    camera_view_transformed.block<3, 1>(0, 3) = camera_position;

    // Set the homogenous coordinate part
    camera_view_transformed.row(3) << 0, 0, 0, 1;

    uniform.camera_view_transformed = camera_view_transformed.inverse();

    // TODO: setup projection matrix

    // set up the orthographic projection matrix
    uniform.orth_matrix = Eigen::Matrix4d::Identity();
    double height = 2 * tan(field_of_view / 2) * near_plane;
    double width = aspect_ratio * height;

    uniform.orth_matrix(0, 0) = 2 / width;
    uniform.orth_matrix(1, 1) = 2 / height;
    uniform.orth_matrix(2, 2) = -2 / (far_plane - near_plane);
    uniform.orth_matrix(2, 3) = -(far_plane + near_plane) / (far_plane - near_plane);

    Matrix4d P;
    if (is_perspective)
    {
        // TODO setup prespective camera
    }
    else
    {
    }
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: fill the shader
        VertexAttributes shader_out;
        shader_out.position = uniform.orth_matrix * uniform.camera_view_transformed * va.position;
        return shader_out;
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
    // TODO: build the vertex attributes from vertices and facets
    // Efficiently reserve space for all vertex attributes to avoid reallocation
    vertex_attributes.reserve(facets.size() * 3);

    // Loop over all facets and create vertex attributes for each vertex
    for (int i = 0; i < facets.rows(); ++i)
    {
        for (int j = 0; j < 3; ++j)
        { // Each facet has three vertices
            int vertexIndex = facets(i, j);
            const Vector3d &vertexPos = vertices.row(vertexIndex);
            vertex_attributes.emplace_back(vertexPos[0], vertexPos[1], vertexPos[2]);
        }
    }
    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

Matrix4d compute_rotation(const double alpha)
{
    // TODO: Compute the rotation matrix of angle alpha on the y axis around the object barycenter
    Matrix4d res;

    return res;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    Matrix4d trafo = compute_rotation(alpha);

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // TODO: fill the shader
        VertexAttributes shader_out;
        shader_out.position = uniform.orth_matrix * uniform.camera_view_transformed * va.position;
        return shader_out;
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

    // std::vector<VertexAttributes> vertex_attributes;
    frameBuffer.setZero();
    std::vector<VertexAttributes> vertex_attributes;
    vertex_attributes.reserve(facets.rows() * 6); // Reserving space for all vertex attributes

    // Generate all line vertex attributes in one loop
    for (int i = 0; i < facets.rows(); i++)
    {
        const Vector3i &triangle = facets.row(i);
        for (int j = 0; j < 3; j++)
        {
            const Vector3d &start = vertices.row(triangle[j]);
            const Vector3d &end = vertices.row(triangle[(j + 1) % 3]); // Loop back to the first vertex
            vertex_attributes.emplace_back(start[0], start[1], start[2]);
            vertex_attributes.emplace_back(end[0], end[1], end[2]);
        }
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
