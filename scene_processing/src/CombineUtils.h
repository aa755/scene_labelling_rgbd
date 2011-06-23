#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
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
public:
    double v[3];

    VectorG()
    {
    }

    VectorG(double unitX, double unitY, double unitZ)
    {
        v[0] = unitX;
        v[1] = unitY;
        v[2] = unitZ;


    }

    VectorG(pcl::PointXYZRGBNormal p)
    {
        v[0] = p.x;
        v[1] = p.y;
        v[2] = p.z;


    }
    VectorG(pcl::PointXYZRGB p)
    {
        v[0] = p.x;
        v[1] = p.y;
        v[2] = p.z;


    }
    
    VectorG(pcl::PointXYZRGBCamSL p)
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
    
    VectorG
    normalizeAndReturn() const
    {
        VectorG out;
        double norm = getNorm();
        for (int i = 0; i < 3; i++)
            out.v[i] = v[i] / norm;
        return out;
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
    getNorm() const
    {
        return sqrt(getNormSqr());
    }

    double
    getNormSqr() const
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
    
   Eigen::Vector4f   toEigenFormat()
   {
       Eigen::Vector4f out;
       for (int i=0;i<3;i++)
           out(i)=v[i];
       return out;
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
    
void transformPointCloudInPlaceAndSetOrigin( pcl::PointCloud<PointT> & in)
{

    boost::numeric::ublas::matrix<double> matIn(4, 1);

    for (size_t i = 0; i < in.points.size(); ++i)
    {
        double * matrixPtr = matIn.data().begin();

        matrixPtr[0] = in.points[i].x;
        matrixPtr[1] = in.points[i].y;
        matrixPtr[2] = in.points[i].z;
        matrixPtr[3] = 1;
        boost::numeric::ublas::matrix<double> matOut = prod(transformMat, matIn);
        matrixPtr = matOut.data().begin();

        in.points[i].x = matrixPtr[0];
        in.points[i].y = matrixPtr[1];
        in.points[i].z = matrixPtr[2];
    }
    
    in.sensor_origin_=getOrigin().toEigenFormat(); // does not work, on reading, reader looses these values
}
    
    boost::numeric::ublas::matrix<double> transformMat;

    TransformG(const tf::Transform& bt)
    {
        transformMat = transformAsMatrix(bt);
    }
    
    TransformG postMultiply(TransformG multiplicand)
    {
        TransformG out;
        out.transformMat=boost::numeric::ublas::prod(transformMat, multiplicand.transformMat); 
        return out;
    }
    #define PI (3.141592653589793)
    bool isOverlapSignificant(TransformG other)
    {
        VectorG orig1=getOrigin();
        VectorG orig2=other.getOrigin();
        VectorG disp=orig1.subtract(orig2);
        if(disp.getNormSqr()>0.5) // if the camera moved by more than 40 cm
            return false;
        double angle=34;
        double cosTurn=fabs(getZUnitVector().dotProduct(other.getZUnitVector()));
        std::cerr<<"turn by"<<cosTurn<<std::endl;
        if(cosTurn<cos(angle*PI/180.0)) //if the camera turned by more than "angle"
            return false;
        
        return true;
            
    }
    
    TransformG preMultiply(TransformG multiplicand)
    {
        TransformG out;
        out.transformMat=boost::numeric::ublas::prod( multiplicand.transformMat,transformMat); 
        return out;
    }

    TransformG()
    {
        transformMat=boost::numeric::ublas::matrix<double>(4,4);
    }
    
    
    tf::Transform  getAsRosMsg()
    {
        btTransform out;
        btMatrix3x3 m_basis(transformMat(0,0),transformMat(0,1),transformMat(0,2),
                            transformMat(1,0),transformMat(1,1),transformMat(1,2),
                            transformMat(2,0),transformMat(2,1),transformMat(2,2));

         btVector3 origin(transformMat(0,3),transformMat(1,3),transformMat(2,3));
         out.setBasis(m_basis);
        out.setOrigin(origin);
        return out;
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
        if(cam2PointRay.getNormSqr()>16.0)
            return false;
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

void appendCamIndexAndDistance(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr in,pcl::PointCloud<pcl::PointXYGRGBCam>::Ptr out,int camIndex,VectorG camOrigin)
{
    out->header=in->header;
    out->points.resize(in->size());
    for(unsigned int i=0;i<in->size();i++)
    {
        out->points[i].x=in->points[i].x;
        out->points[i].y=in->points[i].y;
        out->points[i].z=in->points[i].z;
        out->points[i].rgb=in->points[i].rgb;
        out->points[i].normal_x=in->points[i].normal_x;
        out->points[i].normal_y=in->points[i].normal_y;
        out->points[i].normal_z=in->points[i].normal_z;
        out->points[i].cameraIndex=camIndex;
        VectorG pt(in->points[i]);
        VectorG disp=pt.subtract(camOrigin);
        out->points[i].distance=disp.getNorm();

    }
}

void appendCamIndexAndDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,pcl::PointCloud<pcl::PointXYZRGBCamSL>::Ptr out,int camIndex,VectorG camOrigin)
{
   // out->header=in->header;
   
    out->points.resize(in->size());
    for(unsigned int i=0;i<in->size();++i)
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

double cosNormal(pcl::PointXYZRGBNormal p1,pcl::PointXYZRGBNormal p2)
{
//    float ans=c1.squaredError(c2)+sqrG(p1.normal_x-p2.normal_x)+sqrG(p1.normal_y-p2.normal_y)+sqrG(p1.normal_z-p2.normal_z);
    double ans=fabs(p1.normal_x*p2.normal_x+p1.normal_y*p2.normal_y+p1.normal_z*p2.normal_z);
//    float ans=(c1.squaredError(c2));//+sqrG(p1.normal_x-p2.normal_x)+sqrG(p1.normal_y-p2.normal_y)+sqrG(p1.normal_z-p2.normal_z);
    return ans;

}

void appendNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out)
{
   pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n3d_;
     pcl::KdTree<pcl::PointXYZRGB>::Ptr normals_tree_, clusters_tree_;
  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
//  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
pcl::PointCloud<pcl::Normal> cloud_normals;
  // Normal estimation parameters
  n3d_.setKSearch (50);
  n3d_.setSearchMethod (normals_tree_);
   n3d_.setInputCloud (in);
  n3d_.compute (cloud_normals);
 pcl::concatenateFields (*in, cloud_normals, *out);
}

/*
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
*/
#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* COMBINEUTILS_H */

