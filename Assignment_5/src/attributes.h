#pragma once

#include <Eigen/Core>
using namespace Eigen;
class VertexAttributes
{
public:
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

    Eigen::Vector4d position;
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
    FrameBufferAttributes(double r = 0, double g = 0, double b = 0, double a = 1)
    {
        color << r, g, b, a;
    }

    Eigen::Matrix<double, 4, 1> color;
};

class UniformAttributes
{
public:
    // Constructor
    UniformAttributes() : mvpMatrix(Matrix4d::Identity()) {} // Initialize with the identity matrix

    // Getter for the transformation matrix
    const Matrix4d &getMvpMatrix() const
    {
        return mvpMatrix;
    }

    // Setter for the transformation matrix
    void setMvpMatrix(const Matrix4d &matrix)
    {
        mvpMatrix = matrix;
    }

private:
    Matrix4d mvpMatrix; // Stores the Model-View-Projection matrix
};