#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <iostream>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>
#include <pcl/registration/transforms.h>
#include <pcl/surface/convex_hull.h>

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <boost/thread.hpp>

#include "point_struct.h"
#include "definitions.h"

using namespace std;
using namespace Eigen;

struct THull {
    pcl::PointCloud<pcl::PointXYZ> hull;
    vector<pcl::Vertices> poly;
    double area;
    Eigen::Vector4f centroid;
    THull() {area = -1;}
    THull(pcl::PointCloud<pcl::PointXYZ> &h, vector<pcl::Vertices> &p) {
        hull = h;
        poly = p;
        area = -1;
    }
};

template <typename PointT, typename D>
void getBoundingBoxAndRadius3D(pcl::PointCloud<PointT> pcd, Eigen::Vector4f centroid,
        D &minX, D &maxX, D &minY, D &maxY, D &minZ, D &maxZ,
        D &minR, D &maxR, float*& vr) {
    minX = minY = minZ = minR = 1e160;
    maxX = maxY = maxZ = maxR = -1e160;
    vr = new float[pcd.points.size()];
    for (size_t i=0;i<pcd.points.size();i++) {
        PointT p = pcd.points[i];
        vr[i] = sqrt( sqr(p.x - centroid(0)) + sqr(p.y - centroid(1)) + sqr(p.z - centroid(2)) );
        minX = MIN(minX,p.x); maxX = MAX(maxX,p.x);
        minY = MIN(minY,p.y); maxY = MAX(maxY,p.y);
        minZ = MIN(minZ,p.z); maxZ = MAX(maxZ,p.z);
        minR = MIN(minR,vr[i]); maxR = MAX(maxR,vr[i]);
    }
}
template <typename PointT, typename D>
void getBoundingBoxAndRadius(pcl::PointCloud<PointT> pcd, Eigen::Vector4f centroid,
        D &minX, D &maxX, D &minY, D &maxY, D &minZ, D &maxZ,
        D &minR, D &maxR, float*& vr) {
    minX = minY = minZ = minR = 1e160;
    maxX = maxY = maxZ = maxR = -1e160;
    vr = new float[pcd.points.size()];
    for (size_t i=0;i<pcd.points.size();i++) {
        PointT p = pcd.points[i];
        vr[i] = sqrt( sqr(p.x - centroid(0)) + sqr(p.y - centroid(1)) );
        minX = MIN(minX,p.x); maxX = MAX(maxX,p.x);
        minY = MIN(minY,p.y); maxY = MAX(maxY,p.y);
        minZ = MIN(minZ,p.z); maxZ = MAX(maxZ,p.z);
        minR = MIN(minR,vr[i]); maxR = MAX(maxR,vr[i]);
    }
}

template <typename PointT, typename D>
void getBoundingBox(pcl::PointCloud<PointT> pcd,
        D &minX, D &maxX, D &minY, D &maxY, D &minZ, D &maxZ) {
    minX = minY = minZ = 1e160;
    maxX = maxY = maxZ = -1e160;
    for (size_t i=0;i<pcd.points.size();i++) {
        PointT p = pcd.points[i];
        minX = MIN(minX,p.x); maxX = MAX(maxX,p.x);
        minY = MIN(minY,p.y); maxY = MAX(maxY,p.y);
        minZ = MIN(minZ,p.z); maxZ = MAX(maxZ,p.z);
    }
}

Eigen::Matrix4f Rx(float t) {
    Eigen::Matrix4f Ti;
    float c = cos(t), s = sin(t);
    Ti << 
        1, 0, 0, 0,
        0, c, -s,0,
        0, s, c, 0,
        0, 0, 0, 1;
    return Ti;
}

Eigen::Matrix4f Ry(float t) {
    Eigen::Matrix4f Ti;
    float c = cos(t), s = sin(t);
    Ti << 
        c, 0, s, 0,
        0, 1, 0, 0,
        -s,0, c, 0,
        0, 0, 0, 1;
    return Ti;
}

Eigen::Matrix4f Rz(float t) {
    Eigen::Matrix4f Ti;
    float c = cos(t), s = sin(t);
    Ti << 
        c,-s, 0, 0,
        s, c, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return Ti;
}

// Z-Y-Z' Euler angles.  Angles specified in radians.
Eigen::Matrix4f rotationMatrix(float tz, float ty, float tzp) {
    return Rz(tz) * Ry(ty) * Rx(tzp);
}

// Z-Y-Z' Euler angles.  Angles specified in radians.
Eigen::Matrix4f rotationMatrixDegrees(float tz, float ty, float tzp) {
    return rotationMatrix(tz*PI/180.0, ty*PI/180.0, tzp*PI/180.0);
}

float randFloat(float max, float min = 0.0, int nSteps = 1000) {
    float r = rand() % nSteps;
    return min + r / nSteps*(max-min);
}

Eigen::Matrix4f randRotation() {
    return rotationMatrix(randFloat(PI,-PI), randFloat(PI,0), randFloat(PI,-PI));
}

void translate(Eigen::Matrix4f &T, Eigen::Vector4f r) {
    T(0,3) += r(0);
    T(1,3) += r(1);
    T(2,3) += r(2);
    //T(3,3) += r(3);
}

template <typename PointT>
PointT findHighestPoint(pcl::PointCloud<PointT> &o, PointT p) {
    PointT h = p;
    double th = 10; 
    for (int i=0;i<o.points.size();i++) {
        if (sqr(o.points[i].x-p.x)+sqr(o.points[i].y-p.y)< th &&
                o.points[i].z > h.z) {
            h = o.points[i];
        }
    }
    return h;
}

template <typename PointT>
PointT findLowestPoint(pcl::PointCloud<PointT> &o) {
    PointT lowest;
    lowest.z = 1e37;
    for (int i=0;i<o.points.size();i++) {
        if (o.points[i].z < lowest.z) {
            lowest = o.points[i];
        }
    }
    return lowest;
}

Matrix4f computeTransformXYZYPR(double x, double y, double z, double yaw, double pitch, double roll)
{
    Matrix4f Ti = rotationMatrix(yaw, pitch, roll);
    Matrix4f eye = Eigen::Matrix4f::Identity();
    Vector4f tr, centroid;
    tr << x, y, z, 0;
    translate(Ti,tr);
    return Ti;

}
// Rotate the pcd w.r.t its centroid, and then translate 
template <typename PointT>
void transformXYZYPR(pcl::PointCloud<PointT> &pcd, pcl::PointCloud<PointT> &target,
        // YPR in radians
        double x, double y, double z, double yaw, double pitch, double roll)
{   
    Matrix4f Ti = computeTransformXYZYPR(x, y, z, yaw,  pitch, roll);
    
    for(int i=0;i<3;i++)
        target.sensor_origin_(i)=Ti(i,3);
    
    pcl::transformPointCloud<PointT>/*<pcl::PointCloud<PointT>*/(pcd,target,Ti);
}

        // YPR in radians



// Transforms obj's lowest point to env's frame,
// yaw,pitch,roll is rotation about obj's frame centered at obj_centroid
// x,y,z is position in env's frame centered at env_centroid
// Moves the lowest point in obj (after rotation) to (x,y,z) in env's frame
template <typename PointT, typename D>
std::pair< Eigen::Matrix4f, pcl::PointCloud<PointT> > 
transformToEnv(pcl::PointCloud<PointT> &obj, pcl::PointCloud<PointT> &env,
        Eigen::Vector4f obj_centroid, Eigen::Vector4f env_centroid,
        D x, D y, D z, D yaw, D pitch, D roll) {
    pcl::PointCloud<PointT> obj_t;
    // Rotate obj about obj_centroid in obj's frame
    Eigen::Matrix4f Ti = rotationMatrix(yaw/180.0*PI, pitch/180.0*PI, roll/180.0*PI);
    Eigen::Matrix4f eye = Eigen::Matrix4f::Identity();
    translate(eye,-obj_centroid);
    Ti = Ti * eye;
    pcl::transformPointCloud(obj, obj_t, Ti);
    // Find lowest point in rotated obj
    PointT lowestPt = findLowestPoint(obj_t);
    Eigen::Vector4f lowest;
    lowest << lowestPt.x, lowestPt.y, lowestPt.z, 0;
    // Move the lowest point to (x,y,z) in env's frame centered at env_centroid
    Eigen::Vector4f tr;
    tr << x, y, z, 0;
    tr = tr + env_centroid - lowest;
    translate(Ti,tr);
    pcl::transformPointCloud(obj, obj_t, Ti);
    return std::make_pair(Ti, obj_t);
}

template <typename PointT>
pcl::PointCloud<PointT>
rotate_about_point(pcl::PointCloud<PointT> &cloud, Eigen::Matrix4f rot, Eigen::Vector4f pt) {
    pcl::PointCloud<PointT> cloud_t;
    Eigen::Matrix4f eye = Eigen::Matrix4f::Identity();
    translate(eye,-pt);
    Eigen::Matrix4f Ti = rot*eye;
    translate(Ti, pt);
    pcl::transformPointCloud(cloud, cloud_t, Ti);
    return cloud_t;
}

template <typename PointT, typename D>
pcl::PointCloud<PointT> 
transformToEnv(pcl::PointCloud<PointT> &obj, pcl::PointCloud<PointT> &env,
        Eigen::Vector4f obj_centroid, Eigen::Vector4f env_centroid,
        Eigen::Matrix4f obj_config, D x, D y, D z) {
    pcl::PointCloud<PointT> obj_t;
    // Rotate obj about obj_centroid in obj's frame
    Eigen::Matrix4f Ti = obj_config;
    Eigen::Matrix4f eye = Eigen::Matrix4f::Identity();
    translate(eye,-obj_centroid);
    Ti = Ti * eye;
    pcl::transformPointCloud(obj, obj_t, Ti);
    // Find lowest point in rotated obj
    PointT lowestPt = findLowestPoint(obj_t);
    Eigen::Vector4f lowest;
    lowest << lowestPt.x, lowestPt.y, lowestPt.z, 0;
    // Move the lowest point to (x,y,z) in env's frame centered at env_centroid
    Eigen::Vector4f tr;
    tr << x, y, z, 0;
    tr = tr + env_centroid - lowest;
    translate(Ti,tr);
    pcl::transformPointCloud(obj, obj_t, Ti);
    return obj_t;
}

template <typename PointT, typename D>
std::pair< Eigen::Matrix4f, pcl::PointCloud<PointT> > 
transformToEnvWithTi(pcl::PointCloud<PointT> &obj, pcl::PointCloud<PointT> &env,
        Eigen::Vector4f obj_centroid, Eigen::Vector4f env_centroid,
        Eigen::Matrix4f obj_config, D x, D y, D z) {
    pcl::PointCloud<PointT> obj_t;
    // Rotate obj about obj_centroid in obj's frame
    Eigen::Matrix4f Ti = obj_config;
    Eigen::Matrix4f eye = Eigen::Matrix4f::Identity();
    translate(eye,-obj_centroid);
    Ti = Ti * eye;
    pcl::transformPointCloud(obj, obj_t, Ti);
    // Find lowest point in rotated obj
    PointT lowestPt = findLowestPoint(obj_t);
    Eigen::Vector4f lowest;
    lowest << lowestPt.x, lowestPt.y, lowestPt.z, 0;
    // Move the lowest point to (x,y,z) in env's frame centered at env_centroid
    Eigen::Vector4f tr;
    tr << x, y, z, 0;
    tr = tr + env_centroid - lowest;
    translate(Ti,tr);
    pcl::transformPointCloud(obj, obj_t, Ti);
    return make_pair(Ti,obj_t);
}

    template <typename PointT, typename PointU>
void insertCluster(pcl::PointCloud<PointT> &obj, int cluster_id, pcl::PointCloud<PointU> &cluster)
{
    for (int i=0; i<(int)obj.points.size(); i++)
    {
        pcl::PointXYZRGBIndexCluster tmp;
        tmp.cluster_id = cluster_id;
        tmp.x = obj.points[i].x;
        tmp.y = obj.points[i].y;
        tmp.z = obj.points[i].z;
        tmp.index = obj.points[i].index;
        tmp.rgb = obj.points[i].rgb;
        cluster.points.push_back(tmp);
    }
}

    template <typename PointT>
double cross_product_2D(PointT &a, PointT &b)
{
    return a.x*b.y-a.y*b.x;
}

template <typename PointT>
PointT cross_product_3D(PointT &a, PointT &b) {
    PointT c;
    c.x = a.y*b.z - a.z*b.y;
    c.y = a.z*b.x - a.x*b.z;
    c.z = a.x*b.y - a.y*b.x;
    return c;
}

    template <typename PointT>
double triangle_area_3D(PointT &a, PointT &b, PointT &c)
{
    PointT v1, v2; // v1 = a->b,  v2 = a->c
    v1.x = b.x - a.x;
    v1.y = b.y - a.y;
    v1.z = b.z - a.z;
    v2.x = c.x - a.x;
    v2.y = c.y - a.y;
    v2.z = c.z - a.z;
    PointT s = cross_product_3D(v1,v2);
    return sqrt( sqr(s.x) + sqr(s.y) + sqr(s.z) ) / 2.0;
}

    template<typename PointT>
bool insideConvexPoly(Vector4f &pt, pcl::PointCloud<PointT> &conv, pcl::Vertices &V)
{
    for (size_t i=0; i+1<V.vertices.size(); i++)
    {
        PointT v1 = conv.points[V.vertices[i]];
        PointT v2 = conv.points[V.vertices[i+1]];
        v1.x-=pt(0), v1.y-=pt(1);
        v2.x-=pt(0), v2.y-=pt(1);
        if (cross_product_2D(v1, v2)<0)
            return false;
    }
    return true;
}

// This function seg-faults when multi-threaded!
    template<typename PointT>
double getPolyArea(pcl::PointCloud<PointT> &conv, pcl::Vertices &V)
{
    double s = 0; 
    for (int i=2;i<V.vertices.size();i++) {
        s += triangle_area_3D(
                conv.points[V.vertices[0]],
                conv.points[V.vertices[i-1]],
                conv.points[V.vertices[i]]);
    }
    if (getPolyArea2D(conv,V) - s > 1e-1) {
        printf("Area:  %g vs %g\n", s, getPolyArea2D(conv,V));
    }
    return s;
    /*
       Vector3f vt[V.vertices.size()], sum(0,0,0);
       for (size_t i=0; i<V.vertices.size(); i++)
       {
       PointT p = conv.points[V.vertices[i]];
       vt[i]<<p.x, p.y, p.z;
       }
       for (size_t i=1; i+1<V.vertices.size(); i++)
       {
       sum+=(vt[i]-vt[0]).cross(vt[i+1]-vt[0]);
       }
    //printf("2D area: %.3lf vs %.3lf\n", getPolyArea2D(conv, V), sqrt(sum.dot(sum))/2.0);
    return sqrt(sum.dot(sum))/2.0;
     */
}

    template<typename PointT>
double getPolyArea2D(pcl::PointCloud<PointT> &conv, pcl::Vertices &V)
{
    double s = 0;  
    for (size_t i=0; i+1<V.vertices.size(); i++)
        s+=cross_product_2D(conv.points[V.vertices[i]], conv.points[V.vertices[i+1]]);
    return s/2;
}

boost::mutex convex_hull_mutex;
template<typename PointT>
void getConvexHullXY(pcl::PointCloud<PointT> &pts, 
        pcl::PointCloud<pcl::PointXYZ> &hull,
        vector<pcl::Vertices> &polygons) {
    // Input: pts - set of points
    // Outputs:  hull - Set of vertices of convex hull
    //           polygons - Index into hull of the polygon faces of hull

    // Build pcd
    pcl::PointCloud<pcl::PointXYZ> tmp;
    tmp.points.clear();  tmp.points.reserve(pts.points.size());
    for (size_t i=0;i<pts.points.size();i++) {
        pcl::PointXYZ pt;
        pt.x = pts.points[i].x;
        pt.y = pts.points[i].y;
        pt.z = 0;
        tmp.points.push_back(pt);
    }

    // Compute convex hull
    boost::mutex::scoped_lock l(convex_hull_mutex);
    pcl::ConvexHull<pcl::PointXYZ> obj_convex_solver;
    obj_convex_solver.setInputCloud(tmp.makeShared());
    obj_convex_solver.reconstruct(hull, polygons);
    return;
}

template<typename PointT>
void getConvexHullXY(vector<PointT> &pts, 
        pcl::PointCloud<pcl::PointXYZ> &hull,
        vector<pcl::Vertices> &polygons) {
    // Input: pts - set of points
    // Outputs:  hull - Set of vertices of convex hull
    //           polygons - Index into hull of the polygon faces of hull

    // Build pcd
    pcl::PointCloud<pcl::PointXYZ> tmp;
    tmp.points.clear();  tmp.points.reserve(pts.size());
    for (size_t i=0;i<pts.size();i++) {
        pcl::PointXYZ pt;
        pt.x = pts[i].x;
        pt.y = pts[i].y;
        pt.z = 0;
        tmp.points.push_back(pt);
    }

    // Compute convex hull
    boost::mutex::scoped_lock l(convex_hull_mutex);
    pcl::ConvexHull<pcl::PointXYZ> obj_convex_solver;
    obj_convex_solver.setInputCloud(tmp.makeShared());
    obj_convex_solver.reconstruct(hull, polygons);
    return;
}
#endif // TRANSFORMATION_H

