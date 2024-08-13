#pragma once

#include <Eigen/Core>

class VertexAttributes
{
public:
    Eigen::Vector4d position;
    Eigen::Vector3d normal; // Add this line to include normal vector
    Eigen::Vector3d color;  // Add this line to include color
    VertexAttributes(double x = 0, double y = 0, double z = 0, double w = 1)
    {
        position << x, y, z, w;
    }

    // Interpolates the vertex attributes
    static VertexAttributes interpolate(
        const VertexAttributes &a,
        const VertexAttributes &b,
        const VertexAttributes &c,
        const double alpha,
        const double beta,
        const double gamma)
    {
        VertexAttributes r;
        r.position = alpha * a.position + beta * b.position + gamma * c.position;
        return r;
    }
};

class FragmentAttributes
{
public:
    FragmentAttributes(double r = 0, double g = 0, double b = 0, double a = 1)
    {
        color << r, g, b, a;
    }

    Eigen::Vector4d color;
};

class FrameBufferAttributes
{
public:
    FrameBufferAttributes(double r = 0, double g = 0, double b = 0, double a = 255)
    {
        color << r, g, b, a;
    }

    Eigen::Matrix<double, 4, 1> color;
};

class UniformAttributes
{
public:
    Eigen::Matrix4d camera_view_transformed; // store the camera view matrix
    Eigen::Matrix4d orth_matrix;             // store the projection matrix
    std::vector<Eigen::Vector3d> light_positions;
};