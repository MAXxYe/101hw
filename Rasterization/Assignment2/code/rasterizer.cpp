// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}


/*
static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //std::cout<<"normal integer call of insideCheck"<<std::endl;
    // arr pointer parameter
    Eigen::Vector3f A = _v[0];
    Eigen::Vector3f B = _v[1];
    Eigen::Vector3f C = _v[2];
    Eigen::Vector3f PA,PB,PC;
    PA << A.x()-x, A.y()-y, 0;
    PB << B.x()-x, B.y()-y, 0;
    PC << C.x()-x, C.y()-y, 0;
    float xx = PA.cross(PB).z();
    float yy = PB.cross(PC).z();
    float zz = PC.cross(PA).z();
    return (xx>=0&&yy>=0&&zz>=0)||(xx<=0&&yy<=0&&zz<=0);
}
*/
static bool insideTriangle(double x, double y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //std::cout<<"MSAA insideCheck"<<std::endl;
    // arr pointer parameter
    Eigen::Vector3f A = _v[0];
    Eigen::Vector3f B = _v[1];
    Eigen::Vector3f C = _v[2];
    Eigen::Vector3f PA,PB,PC;
    PA << A.x()-x, A.y()-y, 0;
    PB << B.x()-x, B.y()-y, 0;
    PC << C.x()-x, C.y()-y, 0;
    double xx = PA.cross(PB).z();
    double yy = PB.cross(PC).z();
    double zz = PC.cross(PA).z();
    return (xx>=0&&yy>=0&&zz>=0)||(xx<=0&&yy<=0&&zz<=0);

}
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {

    
    auto v = t.toVector4();
    //Eigen::Vector4f v = Vector4f(t.x(), t.y(), t.z(), 1.0f);
    // bounding box setting:
    //auto v = t.v;
    Eigen::Vector4f A = v[0];
    Eigen::Vector4f B = v[1];
    Eigen::Vector4f C = v[2];
    float Bbox_xMax = fmax(fmax(A.x(), B.x()), C.x());
    float Bbox_yMax = fmax(fmax(A.y(), B.y()), C.y());
    float Bbox_xMin = fmin(fmin(A.x(), B.x()), C.x());
    float Bbox_yMin = fmin(fmin(A.y(), B.y()), C.y());
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    /*
    for(int x=Bbox_xMin;x<=(int)Bbox_xMax+1;x++){
        for(int y=Bbox_yMin;y<=(int)Bbox_yMax+1;y++){
            if(insideTriangle(x,y,t.v)){
                 auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                 float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                 float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                 z_interpolated *= w_reciprocal;
                 if(z_interpolated<=depth_buf[get_index(x,y)]){
                     depth_buf[get_index(x,y)] = z_interpolated;
                     //set_pixel(Eigen::Vector3f(x,y,1.0),t.getColor());
                     Vector3f color = alpha * t.color[0]  + beta * t.color[1] + gamma * t.color[2];
                     frame_buf[get_index(x,y)] = color*255.0;
                     //Eigen::Vector3f point = Eigen::Vector3f(x,y,1.0f);
                 }
            }
        }
    }
    */
    // MSAA inplementation
    
    int MSAA_times = 4;
    for(int x=Bbox_xMin;x<=(int)Bbox_xMax+1;x++){
        for(int y=Bbox_yMin;y<=(int)Bbox_yMax+1;y++){
                double delta = 1.0f / MSAA_times;
                double coverage = (insideTriangle(x-delta,y-delta,t.v)+insideTriangle(x-delta,y+delta,t.v)+\
                                   insideTriangle(x+delta,y-delta,t.v)+insideTriangle(x+delta,y+delta,t.v)+\
                                   insideTriangle(x-delta,y,t.v)+insideTriangle(x+delta,y,t.v)+insideTriangle(x,y+delta,t.v)+\
                                   insideTriangle(x,y-delta,t.v)+insideTriangle(x,y,t.v))/ 9.0f;
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if(coverage>1e-9f && z_interpolated<=depth_buf[get_index(x,y)]){
                     depth_buf[get_index(x,y)] = z_interpolated;
                     //set_pixel(Eigen::Vector3f(x,y,1.0f),coverage*t.getColor());
                     Vector3f color = alpha * t.color[0]  + beta * t.color[1] + gamma * t.color[2];
                     color *= coverage;
                     frame_buf[get_index(x,y)] = color*255.0;
            
                }
            
        }
    }
    
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{255, 255, 255});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h); // rame_buf.resize(w * h,{0,0,0});
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on