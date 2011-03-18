/* 
 * File:   CombineUtils.h
 * Author: aa755
 *
 * Created on March 16, 2011, 5:02 PM
 */

#ifndef COMBINEUTILS_H
#define	COMBINEUTILS_H
/* registration main:
 * Create
 * - a Qt Application
 * - a ROS Node,
 * - a ROS listener, listening to and processing kinect data
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/foreach.hpp>
//#include <Eigen/Core>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <stdint.h>
#include "pcl/ros/register_point_struct.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include "pcl/io/pcd_io.h"
#include <string>
#include <pcl_ros/io/bag_io.h>
#include "pcl/kdtree/kdtree_flann.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
//#include <Eigen/Core>
#include <tf/transform_listener.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/point_cloud.h"



typedef pcl::PointXYZRGB PointT;
void transformPointCloud(boost::numeric::ublas::matrix<double> &transform, pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out)
{

    boost::numeric::ublas::matrix<double> matIn(4, 1);
    *out = *in;

    for (size_t i = 0; i < in->points.size(); ++i)
    {
        double * matrixPtr = matIn.data().begin();

        matrixPtr[0] = in->points[i].x;
        matrixPtr[1] = in->points[i].y;
        matrixPtr[2] = in->points[i].z;
        matrixPtr[3] = 1;
        boost::numeric::ublas::matrix<double> matOut = prod(transform, matIn);
        matrixPtr = matOut.data().begin();

        out->points[i].x = matrixPtr[0];
        out->points[i].y = matrixPtr[1];
        out->points[i].z = matrixPtr[2];
    }
}


float
sqrG(float y)
{
    return y*y;
}

class VectorG
{
    double v[3];
public:

    VectorG()
    {
    }

    VectorG(double unitX, double unitY, double unitZ)
    {
        v[0] = unitX;
        v[1] = unitY;
        v[2] = unitZ;


    }

    VectorG(PointT p)
    {
        v[0] = p.x;
        v[1] = p.y;
        v[2] = p.z;


    }
    /**
     * compute the distance from the line joining p1 and p2
     */
    double computeDistanceSqrFromLine(VectorG p1,VectorG p2)
    {
        VectorG t1=p1.subtract(*this);

        VectorG t2=p2.subtract(p1);

        return (t1.getNormSqr()*t2.getNormSqr()-sqrG(t1.dotProduct(t2)))/t2.getNormSqr();
    }

    /**
     *true iff it lies in the cylinder of infinite radius defined by p1 and p2
     * the line segment (p1,p2) is the axis of this cylinder
     */
    bool isInsideLineSegment(VectorG p1,VectorG p2)
    {
        VectorG lineSeg=p1.subtract(p2);
        double lengthSqr=lineSeg.getNormSqr();
        VectorG p1Seg=subtract(p1);
        VectorG p2Seg=subtract(p2);

        if(p1Seg.getNormSqr()<=lengthSqr&&p2Seg.getNormSqr()<=lengthSqr)
            return true;
        else
            return false;


    }

    void
    normalize()
    {
        double norm = getNorm();
        for (int i = 0; i < 3; i++)
            v[i] = v[i] / norm;
    }

    PointT getAsPoint()
    {
        PointT p;
        p.x=v[0];
        p.y=v[1];
        p.z=v[2];
        return p;
    }
    
    double
    getNorm()
    {
        return sqrt(getNormSqr());
    }

    double
    getNormSqr()
    {
        return (sqrG(v[0]) + sqrG(v[1]) + sqrG(v[2]));
    }

    double
    dotProduct(VectorG v2g)
    {
        double sum = 0;
        for (int i = 0; i < 3; i++)
            sum = sum + v[i] * v2g.v[i];
        return sum;
    }

    VectorG
    multiply(double scalar)
    {
        VectorG out;
        for (int i = 0; i < 3; i++)
            out.v[i] = scalar*v[i];
        return out;
    }

    VectorG
    subtract(VectorG v2g)
    {
        VectorG out;
        for (int i = 0; i < 3; i++)
            out.v[i] = v[i] - v2g.v[i];
        return out;

    }


    VectorG
    add(VectorG v2g)
    {
        VectorG out;
        for (int i = 0; i < 3; i++)
            out.v[i] = v[i] + v2g.v[i];
        return out;

    }

    float
    eucliedianDistance(VectorG v2g)
    {
        float sum=0;
        for (int i = 0; i < 3; i++)
            sum=sum + sqrG(v[i] - v2g.v[i]);
        return sqrt(sum);

    }

};

boost::numeric::ublas::matrix<double>
transformAsMatrix(const tf::Transform& bt)
{
    boost::numeric::ublas::matrix<double> outMat(4, 4);

    //  double * mat = outMat.Store();

    double mv[12];
    bt.getBasis().getOpenGLSubMatrix(mv);

    btVector3 origin = bt.getOrigin();

    outMat(0, 0) = mv[0];
    outMat(0, 1) = mv[4];
    outMat(0, 2) = mv[8];
    outMat(1, 0) = mv[1];
    outMat(1, 1) = mv[5];
    outMat(1, 2) = mv[9];
    outMat(2, 0) = mv[2];
    outMat(2, 1) = mv[6];
    outMat(2, 2) = mv[10];

    outMat(3, 0) = outMat(3, 1) = outMat(3, 2) = 0;
    outMat(0, 3) = origin.x();
    outMat(1, 3) = origin.y();
    outMat(2, 3) = origin.z();
    outMat(3, 3) = 1;


    return outMat;
}

class TransformG
{
public:
    boost::numeric::ublas::matrix<double> transformMat;

    TransformG(const tf::Transform& bt)
    {
        transformMat = transformAsMatrix(bt);
    }

    float getDistanceFromOrigin(VectorG point)
    {
        return point.eucliedianDistance(getOrigin());
    }
    
    VectorG
    getXUnitVector()
    {
        return getIthColumn(0);
    }

    VectorG
    getYUnitVector()
    {
        return getIthColumn(1);
    }

    VectorG
    getZUnitVector()
    {
        return getIthColumn(2);
    }

    VectorG
    getIthColumn(int i)
    {
        return VectorG(transformMat(0,i), transformMat(1,i), transformMat(2,i));
    }

    VectorG
    getOrigin()
    {
        return VectorG(transformMat(0, 3), transformMat(1, 3), transformMat(2, 3));
    }

    bool isPointVisible(VectorG vPoint)
    {
        VectorG cam2PointRay=vPoint.subtract(getOrigin());
        cam2PointRay.normalize();
        VectorG cam2PointRayUnit=cam2PointRay;
        double xDot=cam2PointRayUnit.dotProduct(getXUnitVector());
        double yDot=cam2PointRayUnit.dotProduct(getYUnitVector());
        double zDot=cam2PointRayUnit.dotProduct(getZUnitVector());
       // std::cerr<<"dots"<<zDot<<","<<xDot<<","<<xDot/zDot<<std::endl;
        if(zDot>0 && fabs(xDot/zDot)<0.51 && fabs(yDot/zDot)<0.4)
            return true;
        else
            return false;
    }

    void filterPeripheryCloud()
    {

    }

    void
    print()
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
                std::cerr << transformMat(i, j) << ",";
            std::cerr << std::endl;
        }
    }
};

btQuaternion
getQuaternion(geometry_msgs::Quaternion q)
{
    return btQuaternion(q.x, q.y, q.z, q.w);
}

btVector3
getVector3(geometry_msgs::Vector3 v)
{
    return btVector3(v.x, v.y, v.z);

}
void appendCamIndex(pcl::PointCloud<PointT>::Ptr in,pcl::PointCloud<pcl::PointXYGRGBCam>::Ptr out,int camIndex)
{
    out->header=in->header;
    out->points.resize(in->size());
    for(unsigned int i=0;i<in->size();i++)
    {
        out->points[i].x=in->points[i].x;
        out->points[i].y=in->points[i].y;
        out->points[i].z=in->points[i].z;
        out->points[i].rgb=in->points[i].rgb;
        out->points[i].cameraIndex=camIndex;
    }
}
void appendCamIndexAndDistance(pcl::PointCloud<PointT>::Ptr in,pcl::PointCloud<pcl::PointXYGRGBCam>::Ptr out,int camIndex,VectorG camOrigin)
{
    out->header=in->header;
    out->points.resize(in->size());
    for(unsigned int i=0;i<in->size();i++)
    {
        out->points[i].x=in->points[i].x;
        out->points[i].y=in->points[i].y;
        out->points[i].z=in->points[i].z;
        out->points[i].rgb=in->points[i].rgb;
        out->points[i].cameraIndex=camIndex;
        VectorG pt(in->points[i]);
        VectorG disp=pt.subtract(camOrigin);
        out->points[i].distance=disp.getNorm();

    }
}

void
applyFilters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inp_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outp)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB > ()), cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB > ());

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;

    sor.setInputCloud(inp_cloud_ptr);
    std::cerr << "initially : " << inp_cloud_ptr->size() << std::endl;

    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    ror.setInputCloud(cloud_filtered);
    std::cerr << "before radius : " << cloud_filtered->size() << std::endl;
    ror.setRadiusSearch(0.01);
    ror.setMinNeighborsInRadius(2);
    ror.filter(*outp);
    std::cerr << "after radius : " << outp->size() << std::endl;
}

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* COMBINEUTILS_H */

