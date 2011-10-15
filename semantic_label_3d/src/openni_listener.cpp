#include "includes/CovarianceMatrix.h"
#include "float.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"
#include "includes/genericUtils.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/intensity_spin.h"
#include "pcl/features/normal_3d.h"
//#include "descriptors_3d/all_descriptors.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "moveRobot.h"
//#include <point_cloud_mapping/kdtree/kdtree_ann.h>
#include <vector>
#include "sensor_msgs/point_cloud_conversion.h"
#include "includes/color.cpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
//#include <point_cloud_mapping/geometry/nearest.h>
#include <pcl_ros/io/bag_io.h>
#include "HOG.cpp"
typedef pcl::PointXYZRGBCamSL PointT;
#include "includes/CombineUtils.h"
#include<boost/numeric/ublas/matrix.hpp>
#include<boost/numeric/ublas/io.hpp>
#include<boost/dynamic_bitset.hpp>
//#include<boost/numeric/bindings/traits/ublas_matrix.hpp>
//#include<boost/numeric/bindings/lapack/gels.hpp>
//#include <boost/numeric/bindings/traits/ublas_vector2.hpp>
//namespace ublas = boost::numeric::ublas;
//namespace lapack= boost::numeric::bindings::lapack;
#include "pcl_visualization/pcl_visualizer.h"
typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
#include "time.h"
typedef pcl::KdTree<PointT> KdTree;
typedef pcl::KdTree<PointT>::Ptr KdTreePtr;
using namespace boost;
#include "includes/segmentAndLabel.h"
#include "openni_listener.h"
#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
using namespace std;
using namespace boost;
using namespace octomap;
#include "wallDistance.h"
ros::Publisher pub;
//#include <Eig>
//typedef pcl::PointXYGRGBCam PointT;
TransformG globalTransform;

typedef pcl::KdTree<PointT> KdTree;
typedef pcl::KdTree<PointT>::Ptr KdTreePtr;
bool UseVolFeats = false;
bool BinFeatures = true;
static const string nodeBinFile = "binStumpsN.txt";
static const string edgeBinFile = "binStumpsE.txt";
string environment;

map<int, int> invLabelMap;
map<int, int> labelMap;
pcl::PCDWriter writer;
#define NUM_CLASSES 17

using namespace Eigen;
Matrix<float, Dynamic, Dynamic> *nodeWeights;
Matrix<float, Dynamic, Dynamic> *edgeWeights[NUM_CLASSES];
vector<int> nodeFeatIndices;
vector<int> edgeFeatIndices;


class SpectralProfile {
    vector<float> eigenValues; // sorted in ascending order
public:
    pcl::PointCloud<PointT>::Ptr cloudPtr;
    HOGFeaturesOfBlock avgHOGFeatsOfSegment;
    float avgH;
    float avgS;
    float avgV;
    int count;

    geometry_msgs::Point32 centroid;
    Eigen::Vector3d normal;

    void setEigValues(Eigen::Vector3d eigenValues_) {
        eigenValues.clear();
        //Assuming the values are sorted
        assert(eigenValues_(0) <= eigenValues_(1));
        assert(eigenValues_(1) <= eigenValues_(2));

        for (int i = 0; i < 3; i++)
            eigenValues.push_back(eigenValues_(i));
        //  std::sort (eigenValues.begin (),eigenValues.end ()); // sorted in ascending order
    }

    float getDescendingLambda(int index) const {
        return eigenValues[2 - index];
    }

    void addCentroid(const SpectralProfile & other)
    {        
        centroid.x+=other.centroid.x;
        centroid.y+=other.centroid.y;
        centroid.z+=other.centroid.z;
        count++; // will be used for computing average
    }
    
    void transformCentroid(TransformG & trans)
    {        
        trans.transformPointInPlace(centroid);
    }
    
    void setCentroid(const SpectralProfile & other)
    {
        centroid=other.centroid;
        count=1;
    }

    void setAvgCentroid()
    {
        centroid.x/=count;
        centroid.y/=count;
        centroid.z/=count;
    }
    
    pcl::PointXYZ getCentroid() {
        pcl::PointXYZ ret;
        ret.x = centroid.x;
        ret.y = centroid.y;
        ret.z = centroid.z;
        return ret;
    }

    PointT getCentroidSL() {
        PointT ret;
        ret.x = centroid.x;
        ret.y = centroid.y;
        ret.z = centroid.z;
        return ret;
    }
    
    float getScatter() const {
        return getDescendingLambda(0);
    }

    float getLinearNess() const {
        return (getDescendingLambda(0) - getDescendingLambda(1));
    }

    float getPlanarNess() const {
        return (getDescendingLambda(1) - getDescendingLambda(2));
    }

    float getNormalZComponent() const {
        return normal[2];
    }

    float getAngleWithVerticalInRadians() const {
        return acos(getNormalZComponent());
    }

    float getHorzDistanceBwCentroids(const SpectralProfile & other) const {
        return sqrt(pow(centroid.x - other.centroid.x, 2) + pow(centroid.y - other.centroid.y, 2));
    }

    float getDistanceSqrBwCentroids(const SpectralProfile & other) const {
        return pow(centroid.x - other.centroid.x, 2) + pow(centroid.y - other.centroid.y, 2) + pow(centroid.z - other.centroid.z, 2);
    }

    float getVertDispCentroids(const SpectralProfile & other) {
        return (centroid.z - other.centroid.z);
    }

    float getHDiffAbs(const SpectralProfile & other) {
        return fabs(avgH - other.avgH);
    }

    float getSDiff(const SpectralProfile & other) {
        return (avgS - other.avgS);
    }

    float getVDiff(const SpectralProfile & other) {
        return (avgV - other.avgV);
    }

    float getAngleDiffInRadians(const SpectralProfile & other) {
        return (getAngleWithVerticalInRadians() - other.getAngleWithVerticalInRadians());
    }

    float getNormalDotProduct(const SpectralProfile & other) {
        return fabs(normal(0) * other.normal(0) + normal(1) * other.normal(1) + normal(2) * other.normal(2));
    }

    float getInnerness(const SpectralProfile & other) {
        float r1 = sqrt(centroid.x * centroid.x + centroid.y * centroid.y);
        float r2 = sqrt(other.centroid.x * other.centroid.x + other.centroid.y * other.centroid.y);
        return r1 - r2;
    }

    float pushHogDiffFeats(const SpectralProfile & other, vector<float> & feats) {
        avgHOGFeatsOfSegment.pushBackAllDiffFeats(other.avgHOGFeatsOfSegment, feats);
    }

    float getCoplanarity(const SpectralProfile & other) {
        float dotproduct = getNormalDotProduct(other);
        if (fabs(dotproduct) > 0.9) // if the segments are coplanar return the displacement between centroids in the direction of the normal
        {
            float distance = (centroid.x - other.centroid.x) * normal[0] + (centroid.y - other.centroid.y) * normal[1] + (centroid.z - other.centroid.z) * normal[2];
            if (distance == 0 || fabs(distance) < (1 / 1000)) {
                return 1000;
            }
            return fabs(1 / distance);
        } else // else return -1
            return -1;
    }

    int getConvexity(const SpectralProfile & other, float mindistance) {
        VectorG centroid1(centroid.x, centroid.y, centroid.z);
        VectorG centroid2(other.centroid.x, other.centroid.y, other.centroid.z);

        VectorG c1c2 = centroid2.subtract(centroid1);
        VectorG c2c1 = centroid1.subtract(centroid2);
        VectorG normal1(normal[0], normal[1], normal[2]);
        VectorG normal2(other.normal[0], other.normal[1], other.normal[2]);
        if (mindistance < 0.04 && ((normal1.dotProduct(c1c2) <= 0 && normal2.dotProduct(c2c1) <= 0) || fabs(normal1.dotProduct(normal2)) > 0.95)) // refer local convexity criterion paper
        {
            return 1;
        }
        // else return 0
        return 0;
    }

};


// global variables related to moving the robot and finding the lables
MoveRobot * robot;
#define MAX_TURNS 2
#define MAX_TRYS 20
int turnCount = 0;
vector<int> labelsToFind; // list of classes to find
boost::dynamic_bitset<> labelsFound(NUM_CLASSES); // if the class label is found or not
boost::dynamic_bitset<> labelsToFindBitset(NUM_CLASSES);
vector<pcl::PointCloud<pcl::PointXYZRGBCamSL> > cloudVector;
std::vector<std::map<int, int> > segIndex2LabelVector;
vector<vector<SpectralProfile> >  spectralProfilesVector;
vector<vector<pcl::PointCloud<PointT> > > segment_cloudsVector;
vector<int > sceneNumVector;
bool all_done = false;
std::ofstream labelsFoundFile;
double currentAngle = 0;
vector<double> rotations;
vector<double> translations;
int objCount = 0;
vector<int> labelsToLookFor; 
vector< vector<pcl::PointXYZI> > locations;
vector<pcl::PointXYZI> maximas(NUM_CLASSES);
bool originalScan = true;
boost::dynamic_bitset<> labelsAlreadyLookedFor(NUM_CLASSES);
boost::dynamic_bitset<> maximaChanged(NUM_CLASSES);
bool foundAny = false;
bool translationState = false;
map<int, double> sceneToAngleMap;
#define SPAN 30

vector<int> maximaFrames(NUM_CLASSES,0);


class BinStumps {
public:
    static const int NUM_BINS = 10;
    double binStumps[NUM_BINS];

    BinStumps(string line) {
        char_separator<char> sep("\t");
        tokenizer<char_separator<char> > tokens(line, sep);
        int count = 0;

        BOOST_FOREACH(string t, tokens) {
            binStumps[count] = (lexical_cast<double>(t.data()));
            //    cout<<t<<":"<<count <<endl;
            count++;
        }
        assert(count == NUM_BINS);
    }

    void writeBinnedValues(double value, std::ofstream & file, int featIndex) {
        int binv, bindex;
        for (int i = 0; i < NUM_BINS; i++) {
            binv = 0;
            if (value <= binStumps[i])
                binv = 1;
            bindex = featIndex * NUM_BINS + i + 1;
            file << " " << bindex << ":" << binv;
        }
    }

    void storeBinnedValues(double value, Matrix<float, Dynamic, 1 > & mat, int featIndex) {
        int binv, bindex;
        for (int i = 0; i < NUM_BINS; i++) {
            binv = 0;
            if (value <= binStumps[i])
                binv = 1;
            bindex = featIndex * NUM_BINS + i;
            mat(bindex) = binv;
        }
    }

    void print() {
        for (int i = 0; i < NUM_BINS; i++)
            cout << "," << binStumps[i];
        cout << endl;
    }
};

void createTrees(const std::vector<pcl::PointCloud<PointT> > &segment_clouds, std::vector< pcl::KdTreeFLANN<PointT>::Ptr > &trees) {

    for (size_t i = 0; i < segment_clouds.size(); i++) {
        pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (segment_clouds[i]));
        pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>); //= boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
        //initTree (0, tree);
        tree->setInputCloud(cloud_ptr);
        trees.push_back(tree);
    }
}

pair<float, int> getSmallestDistance(PointT centroid, pcl::KdTreeFLANN<PointT>::Ptr tree) {
    float min_distance = FLT_MAX;
    int min_index = 0;


    std::vector<int> nn_indices;
    nn_indices.resize(2);
    std::vector<float> nn_distances;
    nn_distances.resize(2);
    //float tolerance = 0.3;

    tree->nearestKSearch(centroid, 2, nn_indices, nn_distances);

    for (size_t j = 0; j < nn_indices.size(); ++j) // nn_indices[0] should be sq_idx
    {
        if (min_distance > nn_distances[j]) {
            min_distance = nn_distances[j];
            min_index = nn_indices[j];
            //   cout << "changing min_distance to "  << min_distance<< endl;

        }
    }

    return make_pair(sqrt(min_distance), min_index);
}

int findNeighbors(PointT centroid, const std::vector<pcl::PointCloud<PointT> > &segment_clouds, const std::vector< pcl::KdTreeFLANN<PointT>::Ptr > &trees, vector <int> &neighbor_map) {
    neighbor_map.clear();
    float tolerance = 0.6;
    map< int, float > distance_matrix;
    // get distance matrix
    for (size_t i = 0; i < segment_clouds.size(); i++) {

        pair<float, int> dist_pair = getSmallestDistance(centroid, trees[i]);
        distance_matrix[i] = dist_pair.first;

    }
    // get neighbour map
    int num_neighbors = 0;
    for (map< int, float >::iterator it = distance_matrix.begin(); it != distance_matrix.end(); it++) {
        if ((*it).second < tolerance) {
            neighbor_map.push_back((*it).first);
            num_neighbors++;
        }
    }
    return num_neighbors;
}



vector<BinStumps> nodeFeatStumps;
vector<BinStumps> edgeFeatStumps;

void readStumpValues(vector<BinStumps> & featBins, const string & file) {
    //    char lineBuf[1000]; // assuming a line is less than 
    string line;
    ifstream myfile(file.data());

    if (!myfile.is_open()) {
        cerr << "cound not find the file:" << file << " which stores the ranges for binning the features .. you should run this program from the folder scene_processing(do roscd scene_processing), or put the binning info files: binStumpsN.txt(for node features) and binStumpsE.txt(for edge features) in current folder and rerun ... exiting with assertion failure" << endl;
    }
    assert(myfile.is_open());

    while (myfile.good()) {
        getline(myfile, line);
        //        cout << line << endl;
        if (line.length() > 0)
            featBins.push_back(BinStumps(line));
    }
    myfile.close();

}

void readWeightVectors() {
    nodeFeatIndices.push_back(15);
    nodeFeatIndices.push_back(51);
    edgeFeatIndices.push_back(5);
    edgeFeatIndices.push_back(6);
    edgeFeatIndices.push_back(7);
    edgeFeatIndices.push_back(10);
    nodeWeights = new Matrix<float, Dynamic, Dynamic > (NUM_CLASSES, 10 * nodeFeatIndices.size());
    readCSV("weights/node_weights.csv", NUM_CLASSES, 10 * nodeFeatIndices.size(), ",", *nodeWeights);
    //      readCSV("weights/node_weights.csv",nodeWeights->rows(),10*nodeFeatIndices.size()," ",*nodeWeights);
    //    char lineBuf[1000]; // assuming a line is less than 
    for (size_t i = 0; i < NUM_CLASSES; i++) {
        edgeWeights[i] = new Matrix<float, Dynamic, Dynamic > (NUM_CLASSES, 10 * edgeFeatIndices.size());
        readCSV("weights/edge_weights_" + boost::lexical_cast<std::string > (i) + ".csv", NUM_CLASSES, 10 * edgeFeatIndices.size(), ",", *edgeWeights[i]);
    }

}

void readInvLabelMap(map<int, int> & invLabelMap, const string & file) {
    //    char lineBuf[1000]; // assuming a line is less than 
    string line;
    ifstream myfile(file.data());

    if (!myfile.is_open()) {
        cerr << "cound not find the file:" << file << " which stores the labelmap .. you should run this program from the folder scene_processing(do roscd scene_processing), or put the missing file in current folder and rerun ... exiting with assertion failure" << endl;
    }
    assert(myfile.is_open());

    int origLabel, svmLabel;
    while (myfile.good()) {

        myfile >> origLabel >> svmLabel;
        //        getline(myfile, line);
        //        cout << line << endl;
        invLabelMap[svmLabel] = origLabel;
        labelMap[origLabel]=svmLabel;
    }
    myfile.close();
}

void readAllStumpValues() {
    readStumpValues(nodeFeatStumps, environment + "/" + nodeBinFile);
    readStumpValues(edgeFeatStumps, environment + "/" + edgeBinFile);
}

using namespace pcl;

class OriginalFrameInfo {
    HOG hogDescriptors;
    TransformG cameraTrans;
    bool cameraTransSet;

public:
    pcl::PointCloud<pcl::PointXYZRGBCamSL>::ConstPtr RGBDSlamFrame; // required to get 2D pixel positions

    void saveImage(int segmentId, int label, vector<Point2DAbhishek>points) {
        CvSize size;
        size.height = 480;
        size.width = 640;
        IplImage * image = cvCreateImage(size, IPL_DEPTH_32F, 3);

        pcl::PointXYZRGBCamSL tmp;
        for (int x = 0; x < size.width; x++)
            for (int y = 0; y < size.height; y++) {
                int index = x + y * size.width;
                tmp = RGBDSlamFrame->points[index];
                ColorRGB tmpColor(tmp.rgb);
                CV_IMAGE_ELEM(image, float, y, 3 * x) = tmpColor.b;
                CV_IMAGE_ELEM(image, float, y, 3 * x + 1) = tmpColor.g;
                CV_IMAGE_ELEM(image, float, y, 3 * x + 2) = tmpColor.r;
            }

        ColorRGB tmpColor(0.0, 1.0, 0.0);
        for (int i = 0; i < points.size(); i++) {
            int x = points[i].x;
            int y = points[i].y;

            CV_IMAGE_ELEM(image, float, y, 3 * x) = tmpColor.b;
            CV_IMAGE_ELEM(image, float, y, 3 * x + 1) = tmpColor.g;
            CV_IMAGE_ELEM(image, float, y, 3 * x + 2) = tmpColor.r;

        }

        char filename[30];
        sprintf(filename, "s%d_l%d.png", segmentId, label);
        HOG::saveFloatImage(filename, image);
        cvReleaseImage(&image);


    }

    OriginalFrameInfo(pcl::PointCloud<pcl::PointXYZRGBCamSL>::ConstPtr RGBDSlamFrame_) {
        cameraTransSet = false;
        RGBDSlamFrame = RGBDSlamFrame_;
        CvSize size;
        size.height = 480;
        size.width = 640;
        cout << "RGBslam size:" << RGBDSlamFrame->size() << endl;
        if (RGBDSlamFrame->size() == 0)
            return; // can be 0 for dummy pcds of manually transformed

        assert(RGBDSlamFrame->size() == size.width * size.height); // can be 0 for dummy pcds of manually transformed

        IplImage * image = cvCreateImage(size, IPL_DEPTH_32F, 3);

        pcl::PointXYZRGBCamSL tmp;

        for (int x = 0; x < size.width; x++)
            for (int y = 0; y < size.height; y++) {
                int index = x + y * size.width;
                tmp = RGBDSlamFrame->points[index];
                ColorRGB tmpColor(tmp.rgb);
                CV_IMAGE_ELEM(image, float, y, 3 * x) = tmpColor.b;
                CV_IMAGE_ELEM(image, float, y, 3 * x + 1) = tmpColor.g;
                CV_IMAGE_ELEM(image, float, y, 3 * x + 2) = tmpColor.r;
            }

        hogDescriptors.computeHog(image);

        cvReleaseImage(&image);
    }

    static Point2DAbhishek getPixelFromIndex(int index) {
        //assuming size is 640*480;
        int width = 640;
        Point2DAbhishek ret;
        ret.y = index / width;
        ret.x = index % width;
        assert(index == ret.x + ret.y * width);
        return ret;
    }

    static void findHog(vector<size_t> & pointIndices, pcl::PointCloud<PointT> &incloud, HOGFeaturesOfBlock &hogSegment, OriginalFrameInfo* targetFrame) {
        static int rejectCout = 0;
        assert(targetFrame->RGBDSlamFrame->size() > 0);
        assert(targetFrame->cameraTransSet);

        vector<Point2DAbhishek> pointsInImageLyingOnSegment;
        for (size_t i = 0; i < pointIndices.size(); i++) {
            pointsInImageLyingOnSegment.push_back(getPixelFromIndex(pointIndices[i]));
        }

        assert(pointsInImageLyingOnSegment.size() > 0);
        targetFrame->hogDescriptors.getFeatValForPixels(pointsInImageLyingOnSegment, hogSegment);
        // targetFrame->saveImage (incloud.points[pointIndices[1]].segment,incloud.points[pointIndices[1]].label,pointsInImageLyingOnSegment);

    }

    void
    setCameraTrans(TransformG cameraTrans) {
        this->cameraTrans = cameraTrans;
        cameraTransSet = true;
    }

    void
    applyPostGlobalTrans(TransformG globalTrans) {
        //post => premultiply coz point is towards right;
        if (cameraTransSet)
            cameraTrans = cameraTrans.preMultiply(globalTrans);
    }

    TransformG
    getCameraTrans() const {
        assert(cameraTransSet);
        return cameraTrans;
    }

    void
    setCameraTransSet(bool cameraTransSet) {
        this->cameraTransSet = cameraTransSet;
    }

    bool
    isCameraTransSet() const {
        return cameraTransSet;
    }

    bool
    isEmpty() const {
        return RGBDSlamFrame->size() == 0;
    }

};


OriginalFrameInfo * originalFrame;

class BinningInfo {
    float max;
    float min;
    int numBins;
    float binSize;
public:

    BinningInfo(float min_, float max_, int numBins_) {
        max = max_;
        min = min_;
        numBins = numBins_;
        assert(max > min);
        binSize = (max - min) / numBins;

    }

    int
    getBinIndex(float value) {
        assert(value >= min);
        assert(value <= max);

        int bin = ((value - min) / binSize);

        assert(bin <= numBins);

        if (bin == numBins) {
            bin = numBins - 1;
        }

        return bin;

    }

    float
    GetBinSize() const {
        return binSize;
    }

    int
    GetNumBins() const {
        return numBins;
    }

    float
    GetMin() const {
        return min;
    }

    float
    GetMax() const {
        return max;
    }
};


pcl::PointCloud<PointT> cloudUntransformed;
pcl::PointCloud<PointT> cloudUntransformedUnfiltered;

bool addNodeHeader = true;
;
bool addEdgeHeader = true;

vector<std::string> nodeFeatNames;
vector<std::string> edgeFeatNames;

void addToNodeHeader(std::string featName, size_t numTimes = 1) {
    //char featNameBuf[50];
    // assert(featName.length ()<40);
    if (addNodeHeader) {
        for (size_t i = 0; i < numTimes; i++) {
            //       sprintf (featNameBuf,"%s%d",featName,i);
            nodeFeatNames.push_back(featName + boost::lexical_cast<std::string > (i));
        }
    }
}

inline void addToEdgeHeader(std::string featName, size_t numTimes = 1) {
    //char featNameBuf[50];
    //assert(featName.length ()<40);
    if (addEdgeHeader) {
        for (size_t i = 0; i < numTimes; i++) {
            //sprintf (featNameBuf,"%s%d",featName,i);
            edgeFeatNames.push_back(featName + boost::lexical_cast<std::string > (i));
        }
    }
}

void buildOctoMap(const pcl::PointCloud<PointT> &cloud, OcTreeROS & tree) {



    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (cloud));
    pcl::PointCloud<PointT>::Ptr cloud_cam(new pcl::PointCloud<PointT > ());



    // convert to  pointXYZ format
    sensor_msgs::PointCloud2 cloud_blob;
    pcl::toROSMsg(*cloud_ptr, cloud_blob);
    pcl::PointCloud<pcl::PointXYZ> xyzcloud;
    pcl::fromROSMsg(cloud_blob, xyzcloud);
    // find the camera co-ordinate
    VectorG cam_coordinates = originalFrame->getCameraTrans().getOrigin();
    pcl::PointXYZ origin(cam_coordinates.v[0], cam_coordinates.v[1], cam_coordinates.v[2]);
    // insert to the tree
    tree.insertScan(xyzcloud, origin, -1, true);
}

void apply_segment_filter(pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment) {
    //ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
    //    outcloud.points = incloud.points;
    outcloud.points.resize(incloud.points.size());

    int j = -1;
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].segment == segment) {
            j++;
            outcloud.points[j].x = incloud.points[i].x;
            outcloud.points[j].y = incloud.points[i].y;
            outcloud.points[j].z = incloud.points[i].z;
            outcloud.points[j].rgb = incloud.points[i].rgb;
            outcloud.points[j].segment = incloud.points[i].segment;
            outcloud.points[j].label = incloud.points[i].label;
            outcloud.points[j].cameraIndex = incloud.points[i].cameraIndex;
            outcloud.points[j].distance = incloud.points[i].distance;

            //     std::cerr<<segment_cloud.points[j].label<<",";
        }
    }

    // cout<<j << ","<<segment<<endl;
    if (j >= 0)
        outcloud.points.resize(j + 1);
    else
        outcloud.points.clear();
}

int MIN_SEG_SIZE = 300;

/** it also discards unlabeled segments
 */
void apply_segment_filter_and_compute_HOG(pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment, SpectralProfile & feats) {
    //ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
    //    outcloud.points = incloud.points;
    outcloud.points.resize(incloud.points.size());




    vector<size_t> indices;
    int j = -1;
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].segment == segment) {
            j++;
            outcloud.points[j].x = incloud.points[i].x;
            outcloud.points[j].y = incloud.points[i].y;
            outcloud.points[j].z = incloud.points[i].z;
            outcloud.points[j].rgb = incloud.points[i].rgb;
            outcloud.points[j].segment = incloud.points[i].segment;
            outcloud.points[j].label = incloud.points[i].label;
            outcloud.points[j].cameraIndex = incloud.points[i].cameraIndex;
            outcloud.points[j].distance = incloud.points[i].distance;
            //  cerr<<incloud.points[i].cameraIndex<<","<<numFrames<<endl;
            indices.push_back(i);

            //     std::cerr<<segment_cloud.points[j].label<<",";
        }
    }

    // cout<<j << ","<<segment<<endl;
    if (j >= MIN_SEG_SIZE) {
        outcloud.points.resize(j + 1);
        OriginalFrameInfo::findHog(indices, incloud, feats.avgHOGFeatsOfSegment, originalFrame);
    } else {
        outcloud.points.clear();
        return;
    }

}

void apply_segment_filter(pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment, SpectralProfile & feats) {
    //ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
    //    outcloud.points = incloud.points;
    outcloud.points.resize(incloud.points.size());




    vector<size_t> indices;
    int j = -1;
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].segment == segment) {
            j++;
            outcloud.points[j].x = incloud.points[i].x;
            outcloud.points[j].y = incloud.points[i].y;
            outcloud.points[j].z = incloud.points[i].z;
            outcloud.points[j].rgb = incloud.points[i].rgb;
            outcloud.points[j].segment = incloud.points[i].segment;
            outcloud.points[j].label = incloud.points[i].label;
            outcloud.points[j].cameraIndex = incloud.points[i].cameraIndex;
            outcloud.points[j].distance = incloud.points[i].distance;
            //  cerr<<incloud.points[i].cameraIndex<<","<<numFrames<<endl;
            indices.push_back(i);

            //     std::cerr<<segment_cloud.points[j].label<<",";
        }
    }

    // cout<<j << ","<<segment<<endl;
    if (j >= MIN_SEG_SIZE) {
        outcloud.points.resize(j + 1);
       // OriginalFrameInfo::findHog(indices, incloud, feats.avgHOGFeatsOfSegment, originalFrame);
    } else {
        outcloud.points.clear();
        return;
    }

}

void apply_notsegment_filter(const pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment) {
    //ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
    //    outcloud.points = incloud.points;
    outcloud.points.resize(incloud.points.size());

    int j = -1;
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].segment != segment) {
            j++;
            outcloud.points[j].x = incloud.points[i].x;
            outcloud.points[j].y = incloud.points[i].y;
            outcloud.points[j].z = incloud.points[i].z;
            outcloud.points[j].rgb = incloud.points[i].rgb;
            outcloud.points[j].segment = incloud.points[i].segment;
            outcloud.points[j].label = incloud.points[i].label;
            outcloud.points[j].cameraIndex = incloud.points[i].cameraIndex;
            outcloud.points[j].distance = incloud.points[i].distance;

            //     std::cerr<<segment_cloud.points[j].label<<",";
        }
    }
    // cout<<j << ","<<segment<<endl;
    assert(j >= 0);
    outcloud.points.resize(j + 1);
}

void apply_camera_filter(const pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int camera) {
    //ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
    //    outcloud.points = incloud.points;
    outcloud.points.resize(incloud.points.size());

    int j = -1;
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].cameraIndex == camera) {
            j++;
            outcloud.points[j].x = incloud.points[i].x;
            outcloud.points[j].y = incloud.points[i].y;
            outcloud.points[j].z = incloud.points[i].z;
            outcloud.points[j].rgb = incloud.points[i].rgb;
            outcloud.points[j].segment = incloud.points[i].segment;
            outcloud.points[j].label = incloud.points[i].label;
            outcloud.points[j].cameraIndex = incloud.points[i].cameraIndex;
            outcloud.points[j].distance = incloud.points[i].distance;

            //     std::cerr<<segment_cloud.points[j].label<<",";
        }
    }
    // cout<<j << ","<<segment<<endl;
    assert(j >= 0);
    outcloud.points.resize(j + 1);
}

float getDistanceToBoundary(const pcl::PointCloud<PointT> &cloud1, const pcl::PointCloud<PointT> &cloud2) {
    float max_distance = 0;
    for (size_t i = 0; i < cloud1.points.size(); ++i) {
        float pdist = 0;
        for (size_t j = 0; j < cloud2.points.size(); ++j) {
            float point_dist = pow((cloud1.points[i].x - cloud2.points[j].x), 2) + pow((cloud1.points[i].y - cloud2.points[j].y), 2) + pow((cloud1.points[i].z - cloud2.points[j].z), 2);
            float distance = (pow(cloud2.points[j].distance, 2) - pow(cloud1.points[i].distance, 2) - (point_dist)) / (2 * cloud1.points[i].distance);
            if (pdist < distance) pdist = distance;
            if (max_distance < distance) max_distance = distance;
            // cout << distance << " " << pdist<< " " << max_distance<< endl;
        }
        // cloud1.points[i].distance = pdist;
    }
    return max_distance;
}

pair<float, int> getSmallestDistance(const pcl::PointCloud<PointT> &cloud1, const pcl::PointCloud<PointT> &cloud2) {
    float min_distance = FLT_MAX;
    int min_index = 0;
    pcl::PointCloud<PointT>::Ptr small_cloud;
    pcl::PointCloud<PointT>::Ptr big_cloud;
    if (cloud1.points.size() > cloud2.points.size()) {
        pcl::PointCloud<PointT>::Ptr cloud_ptr1(new pcl::PointCloud<PointT > (cloud1));
        pcl::PointCloud<PointT>::Ptr cloud_ptr2(new pcl::PointCloud<PointT > (cloud2));
        small_cloud = cloud_ptr2;
        big_cloud = cloud_ptr1;
    } else {
        pcl::PointCloud<PointT>::Ptr cloud_ptr1(new pcl::PointCloud<PointT > (cloud1));
        pcl::PointCloud<PointT>::Ptr cloud_ptr2(new pcl::PointCloud<PointT > (cloud2));
        small_cloud = cloud_ptr1;
        big_cloud = cloud_ptr2;
    }

    pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>); //= boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
    //initTree (0, tree);
    tree->setInputCloud(big_cloud); // ,indicesp);
    std::vector<int> nn_indices;
    nn_indices.resize(2);
    std::vector<float> nn_distances;
    nn_distances.resize(2);
    //float tolerance = 0.3;

    for (size_t i = 0; i < small_cloud->points.size(); ++i) {

        //if (!tree->radiusSearch ((*small_cloud).points[i], tolerance, nn_indices, nn_distances))
        tree->nearestKSearch(small_cloud->points[i], 2, nn_indices, nn_distances);

        for (size_t j = 0; j < nn_indices.size(); ++j) // nn_indices[0] should be sq_idx
        {

            //float distance = pow(cloud1.points[i].x - cloud2.points[nn_indices[j]].x,2) + pow(cloud1.points[i].y - cloud2.points[ nn_indices[j]].y,2) + pow(cloud1.points[i].z - cloud2.points[ nn_indices[j]].z,2);
            //cout<< "i,j = " << i << "," << j<< " dist = " <<distance << endl;
            //float a = nn_distances[j];
            //cout<< nn_distances[j] << " "<< a << endl;
            if (min_distance > nn_distances[j]) {
                min_distance = nn_distances[j];
                min_index = nn_indices[j];
                //   cout << "changing min_distance to "  << min_distance<< endl;

            }
        }
    }

    /*
      for (size_t i = 0; i < cloud1.points.size (); ++i)
      { 
       for (size_t j = 0; j < cloud2.points.size (); ++j)
       {
          float distance = pow(cloud1.points[i].x - cloud2.points[j].x,2) + pow(cloud1.points[i].y - cloud2.points[j].y,2) + pow(cloud1.points[i].z - cloud2.points[j].z,2);
         // cerr<< "i,j = " << i << "," << j<< " dist = " <<distance << endl;
          if (min_distance > distance) min_distance = distance;
       }
      }
     */
    return make_pair(sqrt(min_distance), min_index);
}

/**
 * 
 * @param segment_clouds 
 * @param distance_matrix : it will be populated by this method
 * @param neighbor_map : it will be populated by this method(in adjacency list format)
 * @return : number of edges
 */
int get_neighbors(const std::vector<pcl::PointCloud<PointT> > &segment_clouds, map< pair <int, int>, float > &distance_matrix, map <int, vector <int> > &neighbor_map) {
    float tolerance = 0.6;
    // get distance matrix
    for (size_t i = 0; i < segment_clouds.size(); i++) {
        for (size_t j = i + 1; j < segment_clouds.size(); j++) {
            pair<float, int> dist_pair = getSmallestDistance(segment_clouds[i], segment_clouds[j]);
            distance_matrix[make_pair(segment_clouds[i].points[1].segment, segment_clouds[j].points[1].segment)] = dist_pair.first;
            distance_matrix[make_pair(segment_clouds[j].points[1].segment, segment_clouds[i].points[1].segment)] = dist_pair.first;
        }
        //     std::cerr<< "size of segment " << i << " : " << segment_clouds[i].points.size() << "\t and label is: " << segment_clouds[i].points[1].label <<"\n";
    }
    // get neighbour map
    int num_neighbors = 0;
    for (map< pair <int, int>, float >::iterator it = distance_matrix.begin(); it != distance_matrix.end(); it++) {
        if ((*it).second < tolerance) {
            neighbor_map[(*it).first.first].push_back((*it).first.second);
            num_neighbors++;
        }
        //     cout << (*it).first.first << "," << (*it).first.second <<" => " << (*it).second << endl;
    }
    /*
    // printing the neighbor_map
        for ( map< int, vector <int> >::iterator it=neighbor_map.begin() ; it != neighbor_map.end(); it++ )
        {
          cout << (*it).first << " => ";
          for (vector<int>::iterator it2 = (*it).second.begin(); it2 != (*it).second.end(); it2++)
            cout << "," << (*it2) ;
          cout << endl;
        }*/
    return num_neighbors;
}

void getSegmentDistanceToBoundary(const pcl::PointCloud<PointT> &cloud, map<int, float> &segment_boundary_distance) {
    pcl::PointCloud<PointT>::Ptr cloud_rest(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<PointT>::Ptr cloud_cam(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (cloud));

    int cnt = 0;
    // find all the camera indices// find the max segment number

    map<int, int> camera_indices;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        camera_indices[(int) cloud.points[i].cameraIndex] = 1;
    }
    // for every camera index .. apply filter anf get the point cloud
    for (map<int, int>::iterator it = camera_indices.begin(); it != camera_indices.end(); it++) {
        int ci = (*it).first;
        apply_camera_filter(*cloud_ptr, *cloud_cam, ci);

        // find the segment list
        map<int, int> segments;
        for (size_t i = 0; i < cloud_cam->points.size(); ++i) {
            //  if( cloud_cam->points[i].label != 0)
            segments[(int) cloud_cam->points[i].segment] = 1;
        }
        for (map<int, int>::iterator it2 = segments.begin(); it2 != segments.end(); it2++) {
            cnt++;
            int segnum = (*it2).first;
            apply_segment_filter(*cloud_cam, *cloud_seg, segnum);
            apply_notsegment_filter(*cloud_cam, *cloud_rest, segnum);
            float bdist = getDistanceToBoundary(*cloud_seg, *cloud_rest);

            map<int, float>::iterator segit = segment_boundary_distance.find(segnum);
            if (segit == segment_boundary_distance.end() || bdist > segment_boundary_distance[segnum])
                segment_boundary_distance[segnum] = bdist;
            // if(cnt == 1)  outcloud = *cloud_seg;
            // else outcloud += *cloud_seg;
        }

    }

    //for(map<int,float>::iterator it = )
}

void getMinMax(const pcl::PointCloud<PointT> &cloud, const pcl::PointIndices &indices, Eigen::Vector4f &min_p, Eigen::Vector4f &max_p) {
    min_p.setConstant(FLT_MAX);
    max_p.setConstant(-FLT_MAX);
    min_p[3] = max_p[3] = 0;

    for (size_t i = 0; i < indices.indices.size(); ++i) {
        if (cloud.points[indices.indices[i]].x < min_p[0]) min_p[0] = cloud.points[indices.indices[i]].x;
        if (cloud.points[indices.indices[i]].y < min_p[1]) min_p[1] = cloud.points[indices.indices[i]].y;
        if (cloud.points[indices.indices[i]].z < min_p[2]) min_p[2] = cloud.points[indices.indices[i]].z;

        if (cloud.points[indices.indices[i]].x > max_p[0]) max_p[0] = cloud.points[indices.indices[i]].x;
        if (cloud.points[indices.indices[i]].y > max_p[1]) max_p[1] = cloud.points[indices.indices[i]].y;
        if (cloud.points[indices.indices[i]].z > max_p[2]) max_p[2] = cloud.points[indices.indices[i]].z;
    }
}

void getMinMax(const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &min_p, Eigen::Vector4f &max_p) {
    min_p.setConstant(FLT_MAX);
    max_p.setConstant(-FLT_MAX);
    min_p[3] = max_p[3] = 0;

    for (size_t i = 0; i < cloud.points.size(); ++i) {
        if (cloud.points[i].x < min_p[0]) min_p[0] = cloud.points[i].x;
        if (cloud.points[i].y < min_p[1]) min_p[1] = cloud.points[i].y;
        if (cloud.points[i].z < min_p[2]) min_p[2] = cloud.points[i].z;

        if (cloud.points[i].x > max_p[0]) max_p[0] = cloud.points[i].x;
        if (cloud.points[i].y > max_p[1]) max_p[1] = cloud.points[i].y;
        if (cloud.points[i].z > max_p[2]) max_p[2] = cloud.points[i].z;
    }
}

void getCentroid(const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &centroid) {
    centroid[0] = 0;
    centroid[1] = 0;
    centroid[2] = 0;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        //assert(cloud.points[i].z>=0);
        centroid[0] += cloud.points[i].x;
        centroid[1] += cloud.points[i].y;
        centroid[2] += cloud.points[i].z;
        //assert(centroid[2]>=0);
    }
    centroid[0] = centroid[0] / (cloud.points.size() - 1);
    centroid[1] = centroid[1] / (cloud.points.size() - 1);
    centroid[2] = centroid[2] / (cloud.points.size() - 1);
}

void getSpectralProfile(const pcl::PointCloud<PointT> &cloud, SpectralProfile &spectralProfile) {
    Eigen::Matrix3d eigen_vectors;
    Eigen::Vector3d eigen_values;
    sensor_msgs::PointCloud2 cloudMsg2;
    pcl::toROSMsg(cloud, cloudMsg2);
    sensor_msgs::PointCloud cloudMsg;
    sensor_msgs::convertPointCloud2ToPointCloud(cloudMsg2, cloudMsg);
    //    cloud_geometry::nearest::computePatchEigenNormalized(cloudMsg, eigen_vectors, eigen_values, spectralProfile.centroid);

    Eigen::Matrix3d covariance_matrix;
    computeCovarianceMatrix (cloudMsg, covariance_matrix, spectralProfile.centroid);
    for (unsigned int i = 0 ; i < 3 ; i++)
      {
        for (unsigned int j = 0 ; j < 3 ; j++)
	  {
	    covariance_matrix(i, j) /= static_cast<double> (cloudMsg.points.size ());
	  }
      }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
    eigen_values = ei_symm.eigenvalues ();
    eigen_vectors = ei_symm.eigenvectors ();    

    spectralProfile.setEigValues(eigen_values);
    float minEigV = FLT_MAX;

    for (int i = 0; i < 3; i++) {

        //      cout<<"eig value:"<<eigen_values(i)<<endl;
        if (minEigV > eigen_values(i)) {
            minEigV = eigen_values(i);
            //    cout<<"min eig value:"<<minEigV<<endl;
            spectralProfile.normal = eigen_vectors.col(i);
            // check the angle with line joining the centroid to origin
            VectorG centroid(spectralProfile.centroid.x, spectralProfile.centroid.y, spectralProfile.centroid.z);
            VectorG camera = originalFrame->getCameraTrans().getOrigin();
            VectorG cent2cam = camera.subtract(centroid);
            VectorG normal(spectralProfile.normal[0], spectralProfile.normal[1], spectralProfile.normal[2]);
            if (normal.dotProduct(cent2cam) < 0) {
                // flip the sign of the normal
                spectralProfile.normal[0] = -spectralProfile.normal[0];
                spectralProfile.normal[1] = -spectralProfile.normal[1];
                spectralProfile.normal[2] = -spectralProfile.normal[2];
            }
        }
    }
    assert(minEigV == spectralProfile.getDescendingLambda(2));
}
void getSpectralProfileCent(const pcl::PointCloud<PointT> &cloud, SpectralProfile &spectralProfile) {
    assert(1==2); // last line needs to be fixed
    Eigen::Matrix3d eigen_vectors;
    Eigen::Vector3d eigen_values;
    sensor_msgs::PointCloud2 cloudMsg2;
    pcl::toROSMsg(cloud, cloudMsg2);
    sensor_msgs::PointCloud cloudMsg;
    sensor_msgs::convertPointCloud2ToPointCloud(cloudMsg2, cloudMsg);
//    cloud_geometry::nearest::computePatchEigenNormalized(cloudMsg, eigen_vectors, eigen_values, spectralProfile.centroid);
}

void get_feature_average(vector<vector<float> > &descriptor_results, vector<float> &avg_feats) {

    vector<vector<float> >::iterator it = descriptor_results.begin();
    while (it->size() == 0) it++;
    avg_feats.resize(it->size());

    int count = 0;
    for (vector<vector<float> >::iterator it = descriptor_results.begin(); it < descriptor_results.end(); it++) {
        if (it->size() > 0) {
            count++;
        }
        vector<float>::iterator i = avg_feats.begin();
        for (vector<float>::iterator it2 = it->begin(); it2 < it->end(); it2++, i++) {
            *i = *i + *it2;
        }

    }
    // std::cerr << "average features" << std::endl;
    int c = 0;
    for (vector<float>::iterator i = avg_feats.begin(); i < avg_feats.end(); i++) {
        c++;
        *i = *i / count;
        //     std::cerr << c << " : " << *i << ",\t";
    }
    std::cerr << std::endl;
}

void get_feature_histogram(vector<vector<float> > &descriptor_results, vector< vector<float> > &result, vector<BinningInfo> binningInfos) {

    // num_bin = 5;
    vector<vector<float> >::iterator it = descriptor_results.begin();
    int numFeats = it->size();
    result.resize(numFeats);
    // set size of result vector

    vector<BinningInfo>::iterator binningInfo = binningInfos.begin();
    for (vector<vector<float> >::iterator ires = result.begin(); ires < result.end(); ires++, binningInfo++) {
        ires->resize(binningInfo->GetNumBins());
    }
    /*
        while (it->size() == 0) it++;
        max.resize(it->size(), -FLT_MAX);
        min.resize(it->size(), FLT_MAX);
        int count = 0;
        // find the bin size by finding the max and min of feature value
        for (vector<vector<float> >::iterator it = descriptor_results.begin(); it < descriptor_results.end(); it++) {
            vector<float>::iterator imax = max.begin();
            vector<float>::iterator imin = min.begin();
            int c = 0;
            for (vector<float>::iterator it2 = it->begin(); it2 < it->end(); it2++, imax++, imin++) {
                c++;
                //   std::cerr << c << " : " << *it2 << ",\t";
                if (*imax < *it2) {
     *imax = *it2;
                }
                if (*imin > *it2) {
     *imin = *it2;
                }
            }
            //  std::cerr << std::endl;
        }

     */
    // fill the histogram

    for (vector<vector<float> >::iterator it_point = descriptor_results.begin(); it_point < descriptor_results.end(); it_point++) { // iterate over points
        vector<BinningInfo>::iterator binningInfo = binningInfos.begin();

        vector<vector<float> >::iterator ires = result.begin();

        assert(numFeats == it_point->size()); //missing features NOT allowed for now.

        for (vector<float>::iterator it_feature = it_point->begin(); it_feature < it_point->end(); it_feature++, binningInfo++, ires++) { // iterate over features of the point

            int bin = binningInfo->getBinIndex(*it_feature);

            //   ROS_INFO("%f %d %d",bin_size,bin,(*ires).size());

            (*ires)[bin] += 1;
        }
    }

    // normalize and print histogram
    //   std::cerr << "historam \n";

    int numPoints = descriptor_results.size();

    int c1 = 0, c2 = 0;
    for (vector< vector<float> >::iterator i = result.begin(); i < result.end(); i++) {
        c1++;
        //     std::cerr << "histogram for feature:" << c1 << "\n";
        for (vector<float>::iterator i2 = i->begin(); i2 < i->end(); i2++) {
            c2++;
            *i2 = *i2 / numPoints;
            assert(*i2 <= 1.0);
            //  std::cerr << c2 << " : " << *i2 << ",\t";
        }
        //      std::cerr << std::endl;
    }
    //  std::cerr << std::endl;

}


// concat feats (vector-wise) to features vector

void concat_feats(vector<float> &features, vector<vector<float> > &feats) {
    for (vector<vector<float> >::iterator it = feats.begin(); it < feats.end(); it++) {

        //       vector<float>::iterator i = features.end();
        for (vector<float>::iterator it2 = it->begin(); it2 < it->end(); it2++) {
            features.push_back(*it2);
        }
    }
}

// concat feats vector to features vector

void concat_feats(vector<float> &features, vector<float> &feats) {
    for (vector<float>::iterator it = feats.begin(); it < feats.end(); it++) {

        features.push_back(*it);

    }
}

void get_color_features(const pcl::PointCloud<PointT> &cloud, vector<float> &features, SpectralProfile & spectralProfileOfSegment) {
    int num_bin_H = 6;
    int num_bin_S = 2;
    int num_bin_V = 2;
    // histogram and average of hue and intensity

    vector<vector<float> > hist_features;
    vector<float> avg_features;
    vector < vector <float> > color_features(cloud.points.size());
    vector <vector <float> >::iterator it = color_features.begin();
    for (size_t i = 0; i < cloud.points.size(); ++i, it++) {
        ColorRGB c(cloud.points[i].rgb);
        (*it).push_back(c.H);
        (*it).push_back(c.S);
        (*it).push_back(c.V);
    }

    vector<BinningInfo> binnigInfos;
    binnigInfos.push_back(BinningInfo(0, 360, num_bin_H));
    binnigInfos.push_back(BinningInfo(0, 1, num_bin_S));
    binnigInfos.push_back(BinningInfo(0, 1, num_bin_V));
    get_feature_histogram(color_features, hist_features, binnigInfos);
    get_feature_average(color_features, avg_features);

    spectralProfileOfSegment.avgH = avg_features[0];
    spectralProfileOfSegment.avgS = avg_features[1];
    spectralProfileOfSegment.avgV = avg_features[2];

    concat_feats(features, hist_features);
    addToNodeHeader("H_hist", num_bin_H);
    addToNodeHeader("S_hist", num_bin_S);
    addToNodeHeader("V_hist", num_bin_V);

    concat_feats(features, avg_features);
    addToNodeHeader("HAvg");
    addToNodeHeader("SAvg");
    addToNodeHeader("VAvg");
}

void get_global_features(const pcl::PointCloud<PointT> &cloud, vector<float> &features, SpectralProfile & spectralProfileOfSegment) {

    Eigen::Vector4f min_p;
    Eigen::Vector4f max_p;



    // get bounding box features
    getSpectralProfile(cloud, spectralProfileOfSegment);

    getMinMax(cloud, min_p, max_p);
    float xExtent = max_p[0] - min_p[0];
    float yExtent = max_p[1] - min_p[1];
    float horizontalExtent = sqrt(xExtent * xExtent + yExtent * yExtent);
    float zExtent = max_p[2] - min_p[2];

    features.push_back(horizontalExtent);
    addToNodeHeader("horizontalExtent");

    features.push_back(zExtent);
    addToNodeHeader("zExtent");

    features.push_back(spectralProfileOfSegment.centroid.z);
    addToNodeHeader("centroid_z");


    features.push_back(spectralProfileOfSegment.getNormalZComponent());
    addToNodeHeader("normal_z");

    //spectral saliency features
    features.push_back((spectralProfileOfSegment.getLinearNess()));
    addToNodeHeader("linearness");
    features.push_back((spectralProfileOfSegment.getPlanarNess()));
    addToNodeHeader("planarness");
    features.push_back((spectralProfileOfSegment.getScatter()));
    addToNodeHeader("scatter");
    spectralProfileOfSegment.avgHOGFeatsOfSegment.pushBackAllFeats(features);
    addToNodeHeader("HOG", 31);




}

float get_occupancy_feature(const pcl::PointCloud<PointT> &cloud1, const pcl::PointCloud<PointT> &cloud2, OcTreeROS & tree) {
    //
    OcTreeROS::NodeType* treeNode;
    int occCount = 0;
    int totalCount = 0;
    int unknownCount = 0;
    for (int i = 0; i < 100; i++) {
        int p1Index = rand() % cloud1.points.size();
        int p2Index = rand() % cloud2.points.size();
        VectorG p1(cloud1.points[p1Index]);
        VectorG p2(cloud2.points[p2Index]);
        double distance = (p2.subtract(p1)).getNorm();
        int count = 0;
        for (double r = 0.05; r <= distance - 0.05; r += 0.05) {
            count++;
            VectorG point = p1.add((p2.subtract(p1).normalizeAndReturn()).multiply(r));
            pcl::PointXYZ pt(point.v[0], point.v[1], point.v[2]);
            treeNode = tree.search(pt);
            if (treeNode) {
                if (treeNode->getOccupancy() > 0.5) {
                    occCount++;
                }
                //cout << "Occupancy of node at ("<< pt.x << "," << pt.y<< "," << pt.z << ") = " << treeNode->getOccupancy() << " \n";
            } else {
                unknownCount++;
                //cout << "ERROR: OcTreeNode not found (NULL)\n";
            }

        }
        if (count == 0) {
            VectorG point = p1.add(p2.subtract(p1).multiply(0.5));
            pcl::PointXYZ pt(point.v[0], point.v[1], point.v[2]);
            treeNode = tree.search(pt);
            if (treeNode) {
                if (treeNode->getOccupancy() > 0.5) {
                    occCount++;
                }
                //cout << "Occupancy of node at ("<< pt.x << "," << pt.y<< "," << pt.z << ") = " << treeNode->getOccupancy() << " \n";
            } else {
                unknownCount++;
                //cout << "ERROR: OcTreeNode not found (NULL)\n";
            }
            count++;
        }
        totalCount += count;
    }
    cout << "seg1:" << cloud1.points[1].segment << " label1: " << cloud1.points[1].label << " seg2:" << cloud2.points[1].segment << " label2: " << cloud2.points[1].label << endl;
    cout << "total:" << totalCount << " unknown:" << unknownCount << " occupied:" << occCount << endl;
    return (float) unknownCount / (float) totalCount;
}

/*void get_shape_features(const pcl::PointCloud<PointT> &cloud, vector<float> &features, int num_bin ) {


    sensor_msgs::PointCloud cloud_blob2;
    sensor_msgs::PointCloud2 cloud_tmp;

    SpectralAnalysis sa(0.05);
    SpinImageNormal spin_image(0.025, 0.025, 5, 4, false, sa);
    ShapeSpectral shape_spectral(sa);
    OrientationNormal o_normal(0, 0, 1, sa);
    OrientationTangent o_tangent(0, 0, 1, sa);
    Position position;
    BoundingBoxSpectral bbox_spectral(1.0, sa);

    // histogram feats
    vector<Descriptor3D*> descriptors_3d;
    descriptors_3d.push_back(&shape_spectral);
    descriptors_3d.push_back(&o_normal);
    //descriptors_3d.push_back(&o_tangent);
    //descriptors_3d.push_back(&position);
    //descriptors_3d.push_back(&bbox_spectral);
    
    pcl::toROSMsg(cloud, cloud_tmp);
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_tmp, cloud_blob2);
    cloud_kdtree::KdTreeANN pt_cloud_kdtree(cloud_blob2);
    vector<const geometry_msgs::Point32*> interest_pts;
    if (cloud.points.size() < 2000) {
        interest_pts.resize(cloud_blob2.points.size());
        for (size_t i = 0; i < cloud_blob2.points.size(); i++) {
            interest_pts[i] = &(cloud_blob2.points[i]);
        }
    } else {
        interest_pts.resize(1000);
        map<int, int> in;
        int count = 0;
        while (count < 1000) {
            int a = rand() % cloud_blob2.points.size();
            if (in.find(a) == in.end()) {
                in[a] = 1;
                interest_pts[count] = &(cloud_blob2.points[a]);
            }
            count++;
        }

    }


    //     vector<vector<float> >  descriptor_results;
    //     spin_image.compute(cloud_blob2, pt_cloud_kdtree, interest_pts, descriptor_results);
    unsigned int nbr_descriptors = descriptors_3d.size();
    vector<vector<vector<float> > > all_descriptor_results(nbr_descriptors);

  //  vector<vector<vector<float> > > hist_feats(nbr_descriptors);
    vector<vector<float> > avg_feats1(nbr_descriptors);
    for (unsigned int i = 0; i < nbr_descriptors; i++) {
        descriptors_3d[i]->compute(cloud_blob2, pt_cloud_kdtree, interest_pts, all_descriptor_results[i]);
        
       // get_feature_histogram(all_descriptor_results[i], hist_feats[i], num_bin);
      //  concat_feats(features, hist_feats[i]);
        get_feature_average(all_descriptor_results[i], avg_feats1[i]);
        concat_feats(features, avg_feats1[i]);
    }

/*

    // average feats
    vector<Descriptor3D*> descriptors_3d_avg;
    descriptors_3d_avg.push_back(&spin_image);

    unsigned int nbr_descriptors_avg = descriptors_3d_avg.size();
    vector<vector<vector<float> > > all_avg_descriptor_results(nbr_descriptors);
    vector<vector<float> > avg_feats(nbr_descriptors_avg);
    for (unsigned int i = 0; i < nbr_descriptors_avg; i++) {
        std::cerr << "avg featnum: " << i << "\n";
        descriptors_3d_avg[i]->compute(cloud_blob2, pt_cloud_kdtree, interest_pts, all_avg_descriptor_results[i]);
        get_feature_average(all_avg_descriptor_results[i], avg_feats[i]);
        concat_feats(features, avg_feats[i]);
    }

}
 */
/*void get_avg_normals(vector<pcl::PointCloud<PointT> > &segment_clouds, vector<pcl::Normal> &normalsOut )
{
 
        pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT> ());
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setSearchMethod (tree);
        ne.setKSearch (50);
    for (size_t i = 0; i< segment_clouds.size(); i++)
    {
 
        pcl::Normal avgNormal;
                pcl::PointCloud<PointT>::Ptr cloudptr (new pcl::PointCloud<PointT> (segment_clouds[i]));
                ne.setInputCloud (cloudptr);
                ne.compute (*cloud_normals);
                for (size_t i = 0; i < (*cloud_normals).points.size(); ++i)
                {
                        avgNormal.normal[0] += (*cloud_normals).points[i].normal[0];
                avgNormal.normal[1] += (*cloud_normals).points[i].normal[1];
                        avgNormal.normal[2] += (*cloud_normals).points[i].normal[2]; 
                }
                avgNormal.normal[0] = avgNormal.normal[0]/(*cloud_normals).points.size();
        avgNormal.normal[1] = avgNormal.normal[1]/(*cloud_normals).points.size();
                avgNormal.normal[2] = avgNormal.normal[2]/(*cloud_normals).points.size();
                normalsOut.push_back(avgNormal);
        }
} */
int NUM_ASSOCIATIVE_FEATS = 4 + 1;

void get_pair_features(int segment_id, vector<int> &neighbor_list,
        map< pair <int, int>, float > &distance_matrix,
        std::map<int, int> &segment_num_index_map,
        vector<SpectralProfile> & spectralProfiles,
        map < int, vector<float> > &edge_features,
        OcTreeROS & tree) {

    SpectralProfile segment1Spectral = spectralProfiles[segment_num_index_map[segment_id]];

    for (vector<int>::iterator it = neighbor_list.begin(); it != neighbor_list.end(); it++) {

        int seg2_id = *it;
        SpectralProfile segment2Spectral = spectralProfiles[segment_num_index_map[seg2_id]];





        //here goes the associative features:
        //   segment1Spectral.pushHogDiffFeats (segment2Spectral,edge_features[seg2_id]); addToEdgeHeader ("HOGDiff",31);

        edge_features[seg2_id].push_back(segment1Spectral.getHDiffAbs(segment2Spectral));
        addToEdgeHeader("HDiffAbs");

        edge_features[seg2_id].push_back(segment1Spectral.getSDiff(segment2Spectral));
        addToEdgeHeader("SDiff");

        edge_features[seg2_id].push_back(segment1Spectral.getVDiff(segment2Spectral));
        addToEdgeHeader("Viff");

        edge_features[seg2_id].push_back(segment1Spectral.getCoplanarity(segment2Spectral));
        addToEdgeHeader("Coplanarity");

        edge_features[seg2_id].push_back(segment1Spectral.getConvexity(segment2Spectral, distance_matrix[make_pair(segment_id, seg2_id)]));
        addToEdgeHeader("convexity");

        assert(edge_features[seg2_id].size() == NUM_ASSOCIATIVE_FEATS);

        //here goes the non-associative features


        edge_features[seg2_id].push_back(segment1Spectral.getHorzDistanceBwCentroids(segment2Spectral));
        addToEdgeHeader("centroid_horz_diff");
        // difference in z coordinates of the centroids
        edge_features[seg2_id].push_back(segment1Spectral.getVertDispCentroids(segment2Spectral));
        addToEdgeHeader("centroid_z_diff");
        //cerr << "edge feature for edge (" << seg1_id << "," << seg2_id << ")  = " << centroid_z_diff << endl;

        // distance between closest points
        edge_features[seg2_id].push_back(distance_matrix[make_pair(segment_id, seg2_id)]);
        addToEdgeHeader("dist_closest");

        // difference of angles with vertical
        edge_features[seg2_id].push_back(segment1Spectral.getAngleDiffInRadians(segment2Spectral));
        addToEdgeHeader("AngleDiff");

        // dot product of normals
        edge_features[seg2_id].push_back(segment1Spectral.getNormalDotProduct(segment2Spectral));
        addToEdgeHeader("NormalDotProduct");


        edge_features[seg2_id].push_back(segment1Spectral.getInnerness(segment2Spectral));
        addToEdgeHeader("Innerness");

        // occupancy fraction feature
        if (UseVolFeats) {
            edge_features[seg2_id].push_back(get_occupancy_feature(*(segment1Spectral.cloudPtr), *(segment2Spectral.cloudPtr), tree));
            addToEdgeHeader("Occupancy");
        }

        // this line should be in the end
        addEdgeHeader = false;
    }

}

void parseAndApplyLabels(std::ifstream & file, pcl::PointCloud<pcl::PointXYZRGBCamSL> & cloud, std::vector<pcl::PointCloud<PointT> > & segment_clouds, map<int, int> & segIndex2label) {
    string tokens[3];
    char_separator<char> sep1(" ");
    char_separator<char> sep2(":");
    string line;
    getline(file, line);
    int count;
    tokenizer<char_separator<char> > tokens1(line, sep1);
    map<int, int> segId2label;
    foundAny = false;
    BOOST_FOREACH(string t, tokens1) {
        count = 0;
        tokenizer<char_separator<char> > tokens2(t, sep2);

        BOOST_FOREACH(string t2, tokens2) {
            assert(count < 3);
            tokens[count] = t2;
            count++;
        }
        int segmentIndex = (lexical_cast<int>(tokens[0])) - 1;
        int segmentId = segment_clouds[segmentIndex].points[1].segment;
        int label = lexical_cast<int>(tokens[1]);
        segId2label[segmentId] = label;
        segIndex2label[segmentIndex] = label;
        if (!labelsFound.test(label-1)){
            labelsFound.set(label-1, true);
            foundAny = true;
            cout << "Found label:"<< label-1  <<" in scene" <<  sceneNumVector.back() << "  at :" << currentAngle << endl; 
        }
        

    }

    for (int i = 0; i < cloud.size(); i++) {
        cloud.points[i].label = invLabelMap[segId2label[cloud.points[i].segment]];
    }
}

void saveOriginalImages(const pcl::PointCloud<pcl::PointXYZRGBCamSL> &cloud, pcl::PointXYZ max, pcl::PointXYZ min, pcl::PointXYZ steps, int scene_num) {
    int numBins[3];
    for (int i = 0; i < 3; i++) {
        numBins[i] = int((max.data[i] - min.data[i]) / steps.data[i]) + 1;
        cout << i << " " << max.data[i] << " " << min.data[i] << " " << steps.data[i] << " " << numBins[i] << endl;
    }
    CvSize size;

    size.height = numBins[1];
    size.width = numBins[0];
    IplImage * topImageOriginal = cvCreateImage(size, IPL_DEPTH_32F, 3);
    cout << "size of image " << numBins[0] << "x" << numBins[1] << endl;

    size.height = numBins[2];
    size.width = numBins[1];
    IplImage * frontImageOriginal = cvCreateImage(size, IPL_DEPTH_32F, 3);

    int indexMatrix[numBins[1]][numBins[0]];
    for (size_t i = 0; i < numBins[1]; i++)
        for (size_t j = 0; j < numBins[0]; j++) {
            indexMatrix[i][j] = -1;
        }

    int frontIndexMatrix[numBins[2]][numBins[1]];
    for (size_t i = 0; i < numBins[2]; i++)
        for (size_t j = 0; j < numBins[1]; j++) {
            frontIndexMatrix[i][j] = -1;
        }

    for (size_t i = 0; i < cloud.points.size(); i++) {
        if (isnan(cloud.points[i].x) || isnan(cloud.points[i].y) || isnan(cloud.points[i].z)) {
            continue;
        }
        int xbin = int((cloud.points[i].x - min.data[0]) / steps.data[0]);
        int ybin = int((cloud.points[i].y - min.data[1]) / steps.data[1]);
        int zbin = int((cloud.points[i].z - min.data[2]) / steps.data[2]);
        // cout << "x:" << xbin << " y:" << ybin <<endl;
        if (indexMatrix[ybin][xbin] != -1) {
            //   cout << "indexMatrix[xbin][ybin]= " << indexMatrix[xbin][ybin] << endl;
            if (cloud.points[indexMatrix[ybin][xbin]].z < cloud.points[i].z) {
                indexMatrix[ybin][xbin] = i;
            }
        } else {
            indexMatrix[ybin][xbin] = i;
        }
        if (frontIndexMatrix[zbin][ybin] != -1) {
            //   cout << "indexMatrix[xbin][ybin]= " << indexMatrix[xbin][ybin] << endl;
            if (cloud.points[frontIndexMatrix[zbin][ybin]].x > cloud.points[i].x) {
                frontIndexMatrix[zbin][ybin] = i;
            }
        } else {
            frontIndexMatrix[zbin][ybin] = i;
        }


    }
    ColorRGB tmpColor;
    for (size_t i = 0; i < numBins[0]; i++)
        for (size_t j = 0; j < numBins[1]; j++) {
            int x = numBins[1] - j - 1;
            int y = i;
            if (indexMatrix[x][y] != -1) {
                tmpColor.assignColor(cloud.points[indexMatrix[x][y]].rgb);
            } else {
                tmpColor.assignColor(0, 0, 0);
            }
            CV_IMAGE_ELEM(topImageOriginal, float, j, 3 * i) = tmpColor.b;
            CV_IMAGE_ELEM(topImageOriginal, float, j, 3 * i + 1) = tmpColor.g;
            CV_IMAGE_ELEM(topImageOriginal, float, j, 3 * i + 2) = tmpColor.r;

        }
    char filename[30];
    sprintf(filename, "top0riginal.png");
    HOG::saveFloatImage(filename, topImageOriginal);

    for (size_t i = 0; i < numBins[1]; i++)
        for (size_t j = 0; j < numBins[2]; j++) {
            int x = numBins[2] - 1 - j;
            int y = i;
            if (frontIndexMatrix[x][y] != -1) {
                tmpColor.assignColor(cloud.points[frontIndexMatrix[x][y]].rgb);
            } else {
                tmpColor.assignColor(0, 0, 0);
            }
            CV_IMAGE_ELEM(frontImageOriginal, float, j, 3 * i) = tmpColor.b;
            CV_IMAGE_ELEM(frontImageOriginal, float, j, 3 * i + 1) = tmpColor.g;
            CV_IMAGE_ELEM(frontImageOriginal, float, j, 3 * i + 2) = tmpColor.r;

        }
    // char filename[30];
    sprintf(filename, "front0riginal%d.png",scene_num);
    HOG::saveFloatImage(filename, frontImageOriginal);

}

int getBinIndex(double value, double min, double step) {
    return (value - min) / step;
}

int getBinIndex(pcl::PointXYZ value, pcl::PointXYZ min, pcl::PointXYZ step, int index) {
    return (value.data[index] - min.data[index]) / step.data[index];
}
void lookForClassInOriginalFrameOrthographic(vector<int> & classes, pcl::PointCloud<pcl::PointXYZRGBCamSL> & cloud, vector<SpectralProfile> & spectralProfiles, map<int, int> & segIndex2label, const std::vector<pcl::PointCloud<PointT> > &segment_clouds, int scene_num, vector<pcl::PointXYZI> & maximas)
{
    pcl::PointXYZ steps(0.01, 0.01, 0.01);
    std::vector< pcl::KdTreeFLANN<PointT>::Ptr > trees;

    maximas.resize(classes.size());
    createTrees(segment_clouds, trees);
    /* find bounding box and discretize
     */
    pcl::PointXYZ max;
    pcl::PointXYZ min;
    TransformG invTrans=globalTransform.inverse();
    pcl::PointCloud<pcl::PointXYZRGBCamSL>  cloudOriginal=cloud;
    invTrans.transformPointCloudInPlaceAndSetOrigin(cloudOriginal);
    
    for (int i = 0; i < 3; i++)
    {
        max.data[i] = FLT_MIN;
        min.data[i] = FLT_MAX;
    }
    for (size_t i = 0; i < cloud.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (max.data[j] < cloudOriginal.points[i].data[j])
                max.data[j] = cloudOriginal.points[i].data[j];

            if (min.data[j] > cloudOriginal.points[i].data[j])
                min.data[j] = cloudOriginal.points[i].data[j];
        }
    }
    saveOriginalImages(cloudOriginal, max, min, steps, scene_num);
    int numBins[3];
    for (int i = 0; i < 3; i++)
    {
        numBins[i] = (int) ((max.data[i] - min.data[i]) / steps.data[i]) + 1;
    }

    Matrix<float, Dynamic, 1 > nodeFeatsB(nodeFeatIndices.size()*10);
    Matrix<float, Dynamic, 1 > edgeFeatsB(edgeFeatIndices.size()*10);
    vector<float> nodeFeats(nodeFeatIndices.size(), 0.0);
    vector<float> edgeFeats(edgeFeatIndices.size(), 0.0);
    cout << "max:" << max << endl;
    cout << "min:" << min << endl;
    double maxDist[360];
    getMaxRanges(maxDist, cloud);
    double cost;
    double maxCost[classes.size()];
    double minCost[classes.size()];
    SpectralProfile maxS[classes.size()];
    Matrix<float, Dynamic, Dynamic> heatMapTop[classes.size()];
    Matrix<float, Dynamic, Dynamic> heatMapFront[classes.size()];

    for (int oclass = 0; oclass < classes.size(); oclass++)
    {
        minCost[oclass] = DBL_MAX;
        maxCost[oclass] = -DBL_MAX;
        heatMapTop[oclass].setConstant(numBins[1], numBins[0], -FLT_MAX);
        heatMapFront[oclass].setConstant(numBins[2], numBins[1], -FLT_MAX);
    }




    int countx = 0;
    int county = 0;
    int countz = 0;
    int k;
    float x;
    float y;
    float z;
    for (x = min.x, countx = 0; x < max.x; countx++, x += steps.x)
        for (y = min.y, county = 0; y < max.y; county++, y += steps.y)
            for (z = min.z, countz = 0; z < max.z; countz++, z += steps.z)
            {
                vector<int> neighbors;
                    SpectralProfile target;
                    target.centroid.x = x;
                    target.centroid.y = y;
                    target.centroid.z = z;
                    
                    target.transformCentroid(globalTransform);
                    
                PointT centroid=target.getCentroidSL();
                double dist = getWallDistanceCent(maxDist, centroid);
                
                    nodeFeats.at(0) = target.centroid.z;
                    nodeFeats.at(1) = dist;
                    
                if (dist < 0)
                    continue;
                findNeighbors(centroid, segment_clouds, trees, neighbors);
                
                for (int oclass = 0; oclass < classes.size(); oclass++)
                {
                    k = classes[oclass];
                    cost = 0.0;
                    //compute feats
                    //                nodeFeats.at(1)=0;//dummy for now


                    for (size_t i = 0; i < nodeFeatIndices.size(); i++)
                    {
                        nodeFeatStumps[nodeFeatIndices.at(i)].storeBinnedValues(nodeFeats[i], nodeFeatsB, i);
                    }

                    cost += nodeFeatsB.dot(nodeWeights->row(k));

                    for (size_t i = 0; i < neighbors.size(); i++)
                    {
                        int nbrIndex = neighbors.at(i);
                        int nbrLabel = segIndex2label[nbrIndex] - 1;
                        
                        if(nbrLabel<0) // this neighbor was not labeled by the classifier(probably it is using sum<1)
                            continue;
                        
                        //cerr << nbrIndex<<","<<nbrLabel << ",nl - k," << k << endl;
                        //assert(nbrLabel != k);
                        edgeFeats.at(0) = target.getHorzDistanceBwCentroids(spectralProfiles.at(nbrIndex));
                        edgeFeats.at(1) = target.getVertDispCentroids(spectralProfiles.at(nbrIndex));
                        edgeFeats.at(2) = target.getDistanceSqrBwCentroids(spectralProfiles.at(nbrIndex));
                        edgeFeats.at(3) = target.getInnerness(spectralProfiles.at(nbrIndex));
                        //  cout<<"edge feats "<<edgeFeats[0]<<","<<edgeFeats[1]<<","<<edgeFeats[2]<<endl;

                        for (size_t j = 0; j < edgeFeatIndices.size(); j++)
                        {
                            edgeFeatStumps[edgeFeatIndices.at(j)].storeBinnedValues(edgeFeats[j], edgeFeatsB, j);
                        }
                        cost += edgeFeatsB.dot(edgeWeights[k]->row(nbrLabel));

                        edgeFeats.at(0) = spectralProfiles.at(nbrIndex).getHorzDistanceBwCentroids(target);
                        edgeFeats.at(1) = spectralProfiles.at(nbrIndex).getVertDispCentroids(target);
                        edgeFeats.at(2) = spectralProfiles.at(nbrIndex).getDistanceSqrBwCentroids(target);
                        edgeFeats.at(3) = spectralProfiles.at(nbrIndex).getInnerness(target);

                        for (size_t j = 0; j < edgeFeatIndices.size(); j++)
                        {
                            edgeFeatStumps[edgeFeatIndices.at(j)].storeBinnedValues(edgeFeats[j], edgeFeatsB, j);
                        }

                        assert(nbrLabel >= 0);
                        assert(nbrLabel < NUM_CLASSES);
                        assert(k >= 0);
                        assert(k < NUM_CLASSES);
                        cost += edgeFeatsB.dot(edgeWeights[nbrLabel]->row(k));
                    }

                    //     cout<<x<<","<<y<<","<<z<<","<<dist<<","<<cost<<endl;
                    if (maxCost[oclass] < cost)
                    {
                        maxCost[oclass] = cost;
                        maxS[oclass].setCentroid( target);
                      //  maximas[oclass].x = x;
                      //  maximas[oclass].y = y;
                      //  maximas[oclass].z = z;
                      //  maximas[oclass].intensity = cost;
                        //   cout<<"nodeFeats \n"<<nodeFeatsB<<endl;                    
                    } 
                    else if(maxCost[oclass] == cost)
                    {
                        maxS[oclass].addCentroid(target);                        
                    }

                    if (minCost[oclass] > cost)
                    {
                        minCost[oclass] = cost;
                    }

                    if (heatMapTop[oclass](numBins[1] - 1 - county, countx) < cost)
                    {
                        heatMapTop[oclass](numBins[1] - 1 - county, countx) = cost;
                    }

                    if (heatMapFront[oclass](numBins[2] - 1 - countz, county) < cost)
                    {
                        heatMapFront[oclass](numBins[2] - 1 - countz, county) = cost;
                    }
                }

                // all feats can be computed using SpectralProfile
                // wall distance will take time
                //use octomap to filter out occluded regions

            }

    for (size_t oclass = 0; oclass < classes.size(); oclass++)
    {
        //   replace<float>(heatMapTop[oclass], -FLT_MAX, minCost);
        //    replace<float>(heatMapFront[oclass], -FLT_MAX, minCost);
        maxS[oclass].setAvgCentroid();
        maxS[oclass].transformCentroid(invTrans); // come back to original
        
        maximas[oclass].x = maxS[oclass].centroid.x;
        maximas[oclass].y = maxS[oclass].centroid.y;
        maximas[oclass].z = maxS[oclass].centroid.z;
        maximas[oclass].intensity = maxCost[oclass];
        
        writeHeatMap<float>((lexical_cast<string > (classes[oclass]) + "_topHeat" + lexical_cast<string > (scene_num) + ".png").data(), heatMapTop[oclass], maxCost[oclass], minCost[oclass], numBins[1] - 1 - getBinIndex(maxS[oclass].getCentroid(), min, steps, 1), getBinIndex(maxS[oclass].getCentroid(), min, steps, 0));
        writeHeatMap<float>((lexical_cast<string > (classes[oclass]) + "frontHeat" + lexical_cast<string > (scene_num) + ".png").data(), heatMapFront[oclass], maxCost[oclass], minCost[oclass], numBins[2] - 1 - getBinIndex(maxS[oclass].getCentroid(), min, steps, 2), getBinIndex(maxS[oclass].getCentroid(), min, steps, 1));
        //cout << "optimal point" << maxS.centroid.x << "," << maxS.centroid.y << "," << maxS.centroid.z << " with cost:" << minCost << endl;
    }
    exit(1);


}

void lookForClassInOriginalFrameProjective(vector<int> & classes, pcl::PointCloud<pcl::PointXYZRGBCamSL> & cloud, vector<SpectralProfile> & spectralProfiles, map<int, int> & segIndex2label, const std::vector<pcl::PointCloud<PointT> > &segment_clouds, int scene_num, vector<pcl::PointXYZI> & maximas)
{
    pcl::PointXYZ steps(0.01, 0.01, 0.01);
    std::vector< pcl::KdTreeFLANN<PointT>::Ptr > trees;

    maximas.resize(classes.size());
    createTrees(segment_clouds, trees);
    /* find bounding box and discretize
     */
    pcl::PointXYZ max;
    pcl::PointXYZ min;
    TransformG invTrans=globalTransform.inverse();
    pcl::PointCloud<pcl::PointXYZRGBCamSL>  cloudOriginal=cloud;
    invTrans.transformPointCloudInPlaceAndSetOrigin(cloudOriginal);
    
    for (int i = 0; i < 3; i++)
    {
        max.data[i] = FLT_MIN;
        min.data[i] = FLT_MAX;
    }
    for (size_t i = 0; i < cloud.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (max.data[j] < cloudOriginal.points[i].data[j])
                max.data[j] = cloudOriginal.points[i].data[j];

            if (min.data[j] > cloudOriginal.points[i].data[j])
                min.data[j] = cloudOriginal.points[i].data[j];
        }
    }
    
    saveOriginalImages(cloudOriginal, max, min, steps, scene_num);
    int numBins[3];
    for (int i = 0; i < 3; i++)
    {
        numBins[i] = (int) ((max.data[i] - min.data[i]) / steps.data[i]) + 1;
    }

    Matrix<float, Dynamic, 1 > nodeFeatsB(nodeFeatIndices.size()*10);
    Matrix<float, Dynamic, 1 > edgeFeatsB(edgeFeatIndices.size()*10);
    vector<float> nodeFeats(nodeFeatIndices.size(), 0.0);
    vector<float> edgeFeats(edgeFeatIndices.size(), 0.0);
    cout << "max:" << max << endl;
    cout << "min:" << min << endl;
    double maxDist[360];
    getMaxRanges(maxDist, cloud);
    double cost;
    double maxCost[classes.size()];
    double minCost[classes.size()];
    SpectralProfile maxS[classes.size()];
    Matrix<float, Dynamic, Dynamic> heatMapTop[classes.size()];
    //Matrix<float, Dynamic, Dynamic> heatMapFront[classes.size()];

    int imageWidth=640;
    int imageHeight=480;
    for (int oclass = 0; oclass < classes.size(); oclass++)
    {
        minCost[oclass] = DBL_MAX;
        maxCost[oclass] = -DBL_MAX;
        heatMapTop[oclass].setConstant(imageHeight, imageWidth, -FLT_MAX);
       // heatMapFront[oclass].setConstant(numBins[2], numBins[1], -FLT_MAX);
    }

    int countx = 0;
    int county = 0;
    int countz = 0;
    int k;
    float x;
    float y;
    float z;
                    int imageX;
                    int imageY;
    for (x = min.x, countx = 0; x < max.x; countx++, x += steps.x)
        for (y = min.y, county = 0; y < max.y; county++, y += steps.y)
            for (z = min.z, countz = 0; z < max.z; countz++, z += steps.z)
            {
                vector<int> neighbors;
                    SpectralProfile target;
                    target.centroid.x = x;
                    target.centroid.y = y;
                    target.centroid.z = z;
                    
                    target.transformCentroid(globalTransform);
                    
                PointT centroid=target.getCentroidSL();
                double dist = getWallDistanceCent(maxDist, centroid);
                
                    nodeFeats.at(0) = target.centroid.z;
                    nodeFeats.at(1) = dist;
                    
                if (dist < 0)
                    continue;
                findNeighbors(centroid, segment_clouds, trees, neighbors);
                
                for (int oclass = 0; oclass < classes.size(); oclass++)
                {
                    k = classes[oclass];
                    cost = 0.0;
                    //compute feats
                    //                nodeFeats.at(1)=0;//dummy for now


                    for (size_t i = 0; i < nodeFeatIndices.size(); i++)
                    {
                        nodeFeatStumps[nodeFeatIndices.at(i)].storeBinnedValues(nodeFeats[i], nodeFeatsB, i);
                    }

                    cost += nodeFeatsB.dot(nodeWeights->row(k));

                    for (size_t i = 0; i < neighbors.size(); i++)
                    {
                        int nbrIndex = neighbors.at(i);
                        int nbrLabel = segIndex2label[nbrIndex] - 1;
                        
                        if(nbrLabel<0) // this neighbor was not labeled by the classifier(probably it is using sum<1)
                            continue;
                        
                        //cerr << nbrIndex<<","<<nbrLabel << ",nl - k," << k << endl;
                        //assert(nbrLabel != k);
                        edgeFeats.at(0) = target.getHorzDistanceBwCentroids(spectralProfiles.at(nbrIndex));
                        edgeFeats.at(1) = target.getVertDispCentroids(spectralProfiles.at(nbrIndex));
                        edgeFeats.at(2) = target.getDistanceSqrBwCentroids(spectralProfiles.at(nbrIndex));
                        edgeFeats.at(3) = target.getInnerness(spectralProfiles.at(nbrIndex));
                        //  cout<<"edge feats "<<edgeFeats[0]<<","<<edgeFeats[1]<<","<<edgeFeats[2]<<endl;

                        for (size_t j = 0; j < edgeFeatIndices.size(); j++)
                        {
                            edgeFeatStumps[edgeFeatIndices.at(j)].storeBinnedValues(edgeFeats[j], edgeFeatsB, j);
                        }
                        cost += edgeFeatsB.dot(edgeWeights[k]->row(nbrLabel));

                        edgeFeats.at(0) = spectralProfiles.at(nbrIndex).getHorzDistanceBwCentroids(target);
                        edgeFeats.at(1) = spectralProfiles.at(nbrIndex).getVertDispCentroids(target);
                        edgeFeats.at(2) = spectralProfiles.at(nbrIndex).getDistanceSqrBwCentroids(target);
                        edgeFeats.at(3) = spectralProfiles.at(nbrIndex).getInnerness(target);

                        for (size_t j = 0; j < edgeFeatIndices.size(); j++)
                        {
                            edgeFeatStumps[edgeFeatIndices.at(j)].storeBinnedValues(edgeFeats[j], edgeFeatsB, j);
                        }

                        assert(nbrLabel >= 0);
                        assert(nbrLabel < NUM_CLASSES);
                        assert(k >= 0);
                        assert(k < NUM_CLASSES);
                        cost += edgeFeatsB.dot(edgeWeights[nbrLabel]->row(k));
                    }

                    //     cout<<x<<","<<y<<","<<z<<","<<dist<<","<<cost<<endl;
                    if (maxCost[oclass] < cost)
                    {
                        maxCost[oclass] = cost;
                        maxS[oclass].setCentroid( target);
                      //  maximas[oclass].x = x;
                      //  maximas[oclass].y = y;
                      //  maximas[oclass].z = z;
                      //  maximas[oclass].intensity = cost;
                      //  cout<<"nodeFeats \n"<<nodeFeatsB<<endl;                    
                    } 
                    else if(maxCost[oclass] == cost)
                    {
                        maxS[oclass].addCentroid(target);                        
                    }

                    if (minCost[oclass] > cost)
                    {
                        minCost[oclass] = cost;
                    }
                    
                        imageX=(640*x/z/1.1147 + 640*0.5)/2.0  ;
                        imageY=(480*0.5 -460*y/z/0.8336)/2.0  ; 
                        
                        assert(imageX>=0);
                        assert(imageY>=0);
                        assert(imageX<imageWidth);
                        assert(imageY>imageHeight);

                    if (heatMapTop[oclass](imageY, imageX) < cost)
                    {
                        heatMapTop[oclass](imageY, imageX) = cost;
                    }

                }

                // all feats can be computed using SpectralProfile
                // wall distance will take time
                //use octomap to filter out occluded regions

            }

    for (size_t oclass = 0; oclass < classes.size(); oclass++)
    {
        //   replace<float>(heatMapTop[oclass], -FLT_MAX, minCost);
        //    replace<float>(heatMapFront[oclass], -FLT_MAX, minCost);
        maxS[oclass].setAvgCentroid();
        maxS[oclass].transformCentroid(invTrans); // come back to original
                        imageX=(640*maxS[oclass].centroid.x/maxS[oclass].centroid.z/1.1147 + 640*0.5)/2.0  ;
                        imageY=(480*0.5 -460*maxS[oclass].centroid.y/maxS[oclass].centroid.z/0.8336)/2.0  ; 
        
        maximas[oclass].x = maxS[oclass].centroid.x;
        maximas[oclass].y = maxS[oclass].centroid.y;
        maximas[oclass].z = maxS[oclass].centroid.z;
        maximas[oclass].intensity = maxCost[oclass];
        
        writeHeatMap<float>((lexical_cast<string > (classes[oclass]) + "_topHeat" + lexical_cast<string > (scene_num) + ".png").data(), heatMapTop[oclass], maxCost[oclass], minCost[oclass],imageY, imageX);
      //  writeHeatMap<float>((lexical_cast<string > (classes[oclass]) + "frontHeat" + lexical_cast<string > (scene_num) + ".png").data(), heatMapFront[oclass], maxCost[oclass], minCost[oclass], numBins[2] - 1 - getBinIndex(maxS[oclass].getCentroid(), min, steps, 2), getBinIndex(maxS[oclass].getCentroid(), min, steps, 1));
        //cout << "optimal point" << maxS.centroid.x << "," << maxS.centroid.y << "," << maxS.centroid.z << " with cost:" << minCost << endl;
    }
    exit(1);


}

void lookForClass(vector<int> & classes, pcl::PointCloud<pcl::PointXYZRGBCamSL> & cloud, vector<SpectralProfile> & spectralProfiles, map<int, int> & segIndex2label, const std::vector<pcl::PointCloud<PointT> > &segment_clouds, int scene_num, vector<pcl::PointXYZI> & maximas)
{
    pcl::PointXYZ steps(0.005, 0.005, 0.005);
    std::vector< pcl::KdTreeFLANN<PointT>::Ptr > trees;

    maximas.resize(classes.size());
    createTrees(segment_clouds, trees);
    /* find bounding box and discretize
     */
    pcl::PointXYZ max;
    pcl::PointXYZ min;
    for (int i = 0; i < 3; i++)
    {
        max.data[i] = FLT_MIN;
        min.data[i] = FLT_MAX;
    }
    for (size_t i = 0; i < cloud.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (max.data[j] < cloud.points[i].data[j])
                max.data[j] = cloud.points[i].data[j];

            if (min.data[j] > cloud.points[i].data[j])
                min.data[j] = cloud.points[i].data[j];
        }
    }
    saveOriginalImages(cloud, max, min, steps, scene_num);
    int numBins[3];
    for (int i = 0; i < 3; i++)
    {
        numBins[i] = (int) ((max.data[i] - min.data[i]) / steps.data[i]) + 1;
    }

    Matrix<float, Dynamic, 1 > nodeFeatsB(nodeFeatIndices.size()*10);
    Matrix<float, Dynamic, 1 > edgeFeatsB(edgeFeatIndices.size()*10);
    vector<float> nodeFeats(nodeFeatIndices.size(), 0.0);
    vector<float> edgeFeats(edgeFeatIndices.size(), 0.0);
    cout << "max:" << max << endl;
    cout << "min:" << min << endl;
    double maxDist[360];
    getMaxRanges(maxDist, cloud);
    double cost;
    double maxCost[classes.size()];
    double minCost[classes.size()];
    SpectralProfile maxS[classes.size()];
    Matrix<float, Dynamic, Dynamic> heatMapTop[classes.size()];
    Matrix<float, Dynamic, Dynamic> heatMapFront[classes.size()];

    for (int oclass = 0; oclass < classes.size(); oclass++)
    {
        minCost[oclass] = DBL_MAX;
        maxCost[oclass] = -DBL_MAX;
        heatMapTop[oclass].setConstant(numBins[1], numBins[0], -FLT_MAX);
        heatMapFront[oclass].setConstant(numBins[2], numBins[1], -FLT_MAX);
    }




    int countx = 0;
    int county = 0;
    int countz = 0;
    int k;
    float x;
    float y;
    float z;
    for (x = min.x, countx = 0; x < max.x; countx++, x += steps.x)
        for (y = min.y, county = 0; y < max.y; county++, y += steps.y)
            for (z = min.z, countz = 0; z < max.z; countz++, z += steps.z)
            {
                vector<int> neighbors;
                PointT centroid;
                centroid.x = x;
                centroid.y = y;
                centroid.z = z;
                double dist = getWallDistanceCent(maxDist, centroid);
                if (dist < 0)
                    continue;
                findNeighbors(centroid, segment_clouds, trees, neighbors);
                
                for (int oclass = 0; oclass < classes.size(); oclass++)
                {
                    k = classes[oclass];
                    cost = 0.0;
                    //compute feats
                    nodeFeats.at(0) = z;
                    nodeFeats.at(1) = dist;
                    //                nodeFeats.at(1)=0;//dummy for now

                    SpectralProfile target;
                    target.centroid.x = x;
                    target.centroid.y = y;
                    target.centroid.z = z;

                    for (size_t i = 0; i < nodeFeatIndices.size(); i++)
                    {
                        nodeFeatStumps[nodeFeatIndices.at(i)].storeBinnedValues(nodeFeats[i], nodeFeatsB, i);
                    }

                    cost += nodeFeatsB.dot(nodeWeights->row(k));

                    for (size_t i = 0; i < neighbors.size(); i++)
                    {
                        int nbrIndex = neighbors.at(i);
                        int nbrLabel = segIndex2label[nbrIndex] - 1;
                        
                        if(nbrLabel<0) // this neighbor was not labeled by the classifier(probably it is using sum<1)
                            continue;
                        
                        //cerr << nbrIndex<<","<<nbrLabel << ",nl - k," << k << endl;
                        //assert(nbrLabel != k);
                        edgeFeats.at(0) = target.getHorzDistanceBwCentroids(spectralProfiles.at(nbrIndex));
                        edgeFeats.at(1) = target.getVertDispCentroids(spectralProfiles.at(nbrIndex));
                        edgeFeats.at(2) = target.getDistanceSqrBwCentroids(spectralProfiles.at(nbrIndex));
                        edgeFeats.at(3) = target.getInnerness(spectralProfiles.at(nbrIndex));
                        //  cout<<"edge feats "<<edgeFeats[0]<<","<<edgeFeats[1]<<","<<edgeFeats[2]<<endl;

                        for (size_t j = 0; j < edgeFeatIndices.size(); j++)
                        {
                            edgeFeatStumps[edgeFeatIndices.at(j)].storeBinnedValues(edgeFeats[j], edgeFeatsB, j);
                        }
                        cost += edgeFeatsB.dot(edgeWeights[k]->row(nbrLabel));

                        edgeFeats.at(0) = spectralProfiles.at(nbrIndex).getHorzDistanceBwCentroids(target);
                        edgeFeats.at(1) = spectralProfiles.at(nbrIndex).getVertDispCentroids(target);
                        edgeFeats.at(2) = spectralProfiles.at(nbrIndex).getDistanceSqrBwCentroids(target);
                        edgeFeats.at(3) = spectralProfiles.at(nbrIndex).getInnerness(target);

                        for (size_t j = 0; j < edgeFeatIndices.size(); j++)
                        {
                            edgeFeatStumps[edgeFeatIndices.at(j)].storeBinnedValues(edgeFeats[j], edgeFeatsB, j);
                        }

                        assert(nbrLabel >= 0);
                        assert(nbrLabel < NUM_CLASSES);
                        assert(k >= 0);
                        assert(k < NUM_CLASSES);
                        cost += edgeFeatsB.dot(edgeWeights[nbrLabel]->row(k));
                    }

                    //     cout<<x<<","<<y<<","<<z<<","<<dist<<","<<cost<<endl;
                    if (maxCost[oclass] < cost)
                    {
                        maxCost[oclass] = cost;
                        maxS[oclass].setCentroid( target);
                      //  maximas[oclass].x = x;
                      //  maximas[oclass].y = y;
                      //  maximas[oclass].z = z;
                      //  maximas[oclass].intensity = cost;
                        //   cout<<"nodeFeats \n"<<nodeFeatsB<<endl;                    
                    } 
                    else if(maxCost[oclass] == cost)
                    {
                        maxS[oclass].addCentroid(target);                        
                    }

                    if (minCost[oclass] > cost)
                    {
                        minCost[oclass] = cost;
                    }

                    if (heatMapTop[oclass](numBins[1] - 1 - county, countx) < cost)
                    {
                        heatMapTop[oclass](numBins[1] - 1 - county, countx) = cost;
                    }

                    if (heatMapFront[oclass](numBins[2] - 1 - countz, county) < cost)
                    {
                        heatMapFront[oclass](numBins[2] - 1 - countz, county) = cost;
                    }
                }

                // all feats can be computed using SpectralProfile
                // wall distance will take time
                //use octomap to filter out occluded regions

            }

    for (size_t oclass = 0; oclass < classes.size(); oclass++)
    {
        //   replace<float>(heatMapTop[oclass], -FLT_MAX, minCost);
        //    replace<float>(heatMapFront[oclass], -FLT_MAX, minCost);
        maxS[oclass].setAvgCentroid();
        
        maximas[oclass].x = maxS[oclass].centroid.x;
        maximas[oclass].y = maxS[oclass].centroid.y;
        maximas[oclass].z = maxS[oclass].centroid.z;
        maximas[oclass].intensity = maxCost[oclass];
        
        writeHeatMap<float>((lexical_cast<string > (classes[oclass]) + "_topHeat" + lexical_cast<string > (scene_num) + ".png").data(), heatMapTop[oclass], maxCost[oclass], minCost[oclass], numBins[1] - 1 - getBinIndex(maxS[oclass].getCentroid(), min, steps, 1), getBinIndex(maxS[oclass].getCentroid(), min, steps, 0));
        writeHeatMap<float>((lexical_cast<string > (classes[oclass]) + "frontHeat" + lexical_cast<string > (scene_num) + ".png").data(), heatMapFront[oclass], maxCost[oclass], minCost[oclass], numBins[2] - 1 - getBinIndex(maxS[oclass].getCentroid(), min, steps, 2), getBinIndex(maxS[oclass].getCentroid(), min, steps, 1));
        //cout << "optimal point" << maxS.centroid.x << "," << maxS.centroid.y << "," << maxS.centroid.z << " with cost:" << minCost << endl;
    }



}

int counts[640 * 480];

int write_feats(TransformG transG, pcl::PointCloud<pcl::PointXYZRGBCamSL>::Ptr & cloud_ptr, int scene_num) {
    std::ofstream featfile;
    pcl::PointCloud<pcl::PointXYZRGBCamSL> & cloud = *cloud_ptr;
    originalFrame = new OriginalFrameInfo(cloud_ptr);

    //labelfile.open("data_labels.txt",ios::app);
    string featfilename = "data_scene_labelling_full_" + lexical_cast<string > (scene_num);
    featfile.open(featfilename.data());

    // read the pcd file


    // convert to templated message type

    originalFrame->setCameraTrans(transG);



    OcTreeROS tree(0.01);
    if (UseVolFeats) {
        OcTreeROS::NodeType* treeNode;
        buildOctoMap(cloud, tree);
    }



    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT > ());
    //pcl::PointCloud<PointXYZI>::Ptr cloud_seg (new pcl::PointCloud<PointXYZI> ());
    std::vector<pcl::PointCloud<PointT> > segment_clouds;
    std::map<int, int> segment_num_index_map;
    pcl::PointIndices::Ptr segment_indices(new pcl::PointIndices());

    // get segments

    // find the max segment number

    int max_segment_num = 0;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        counts[cloud.points[i].segment]++;
        if (max_segment_num < cloud.points[i].segment) {
            max_segment_num = (int) cloud.points[i].segment;
        }
    }

    ExtractIndices<PointT> extract;

    int index_ = 0;
    vector<SpectralProfile> spectralProfiles;
    std::cerr << "max_seg num:" << max_segment_num << "," << cloud.points.size() << endl;
    for (int seg = 1; seg <= max_segment_num; seg++) {
         if(counts[seg]<=MIN_SEG_SIZE)
           continue;
        //vector<float> features;
        //int label;
        //segment_indices->indices.clear();
        //int sizec = 0;
        //for (size_t i = 0; i < cloud.points.size(); ++i) {
        //assert(cloud.points[i].z>=0);
        //  if (cloud.points[i].segment == seg) {
        //    sizec++;
        //  segment_indices->indices.push_back(i);
        // label = cloud.points[i].label;
        // }
        // }
        // if(label!=0) cout << "segment: "<< seg << " label: " << label  << " size: " << sizec << endl;
        //extract.setInputCloud(cloud_ptr);
        //extract.setIndices(segment_indices);
        //extract.setNegative(false);
        //extract.filter(*cloud_seg);
        SpectralProfile temp;
        apply_segment_filter_and_compute_HOG(*cloud_ptr, *cloud_seg, seg, temp);

        //if (label!=0) cout << "segment: "<< seg << " label: " << label << " size: " << cloud_seg->points.size() << endl;
        if (!cloud_seg->points.empty() && cloud_seg->points.size() > MIN_SEG_SIZE) {
            //std::cout << seg << ". Cloud size after extracting : " << cloud_seg->points.size() << std::endl;
            segment_clouds.push_back(*cloud_seg);
            pcl::PointCloud<PointT>::Ptr tempPtr(new pcl::PointCloud<PointT > (segment_clouds[segment_clouds.size() - 1]));
            temp.cloudPtr = tempPtr;

            spectralProfiles.push_back(temp);
            segment_num_index_map[cloud_seg->points[1].segment] = index_;
            index_++;
        }
    }
    map< pair <int, int>, float > distance_matrix;
    map <int, vector <int> > neighbor_map;
    cerr << "computing neighbores" << endl;
    clock_t start_time = clock();
    int num_edges = get_neighbors(segment_clouds, distance_matrix, neighbor_map);
    clock_t elapsed = clock() - start_time;
    cerr << "computing neighbores" << elapsed / ((double) CLOCKS_PER_SEC) << endl;

    //    cerr<<"done adding wall distance features"<<endl;


    // for each segment compute node featuers
    int num_bin_shape = 3;
    map < int, vector<float> > features;
    bool isFirstFrame = addNodeHeader;
    for (size_t i = 0; i < segment_clouds.size(); i++) {
        //   vector<float> features;
        int seg_id = segment_clouds[i].points[1].segment;
        // get color features
        //cout << "computing color features" << std::endl;
        get_color_features(segment_clouds[i], features[seg_id], spectralProfiles[i]);

        // get shape features - now in global features
        //   get_shape_features(segment_clouds[i], features[seg_id], num_bin_shape);

        // get bounding box and centroid point features
        get_global_features(segment_clouds[i], features[seg_id], spectralProfiles[i]);
        addNodeHeader = false;

    }
    cerr << "adding wall distance features" << endl;
    start_time = clock();
    add_distance_features(cloud, features, segment_clouds);
    if (isFirstFrame) nodeFeatNames.push_back("distance_from_wall0");
    elapsed = clock() - start_time;
    cerr << "time for computing wall" << elapsed / ((double) CLOCKS_PER_SEC) << endl;

    cerr << "done adding wall distance features" << endl;

    //  vector<pcl::Normal> cloud_normals;
    // get_avg_normals(segment_clouds,cloud_normals);
    // print the node features
    //   assert(nodeFeatNames.size ()<100); // some error in setting flag can cause trouble
    //for(size_t i=0;i<nodeFeatNames.size ();i++)
    //featfile<<"#"<<nodeFeatNames[i]<<endl;
    int totatAssocFeats = NUM_ASSOCIATIVE_FEATS;
    if (BinFeatures)
        totatAssocFeats = NUM_ASSOCIATIVE_FEATS * BinStumps::NUM_BINS;
    featfile << segment_clouds.size() << " " << num_edges << " " << 17/*should not matter ... it is read from modelfile*/ << " " << totatAssocFeats << endl;

    for (map< int, vector<float> >::iterator it = features.begin(); it != features.end(); it++) {
        assert(nodeFeatNames.size() == (*it).second.size());
        //cerr << (*it).first << ":\t";
        featfile << segment_clouds[segment_num_index_map[(*it).first]].points[1].label << " " << (*it).first;
        int featIndex = 0;
        for (vector<float>::iterator it2 = (*it).second.begin(); it2 != (*it).second.end(); it2++) {
            if (BinFeatures) {
                nodeFeatStumps[featIndex].writeBinnedValues(*it2, featfile, featIndex);
            } else {
                featfile << " " << featIndex << ":" << *it2;
            }
            featIndex++;
        }
        //cerr << endl;
        featfile << "\n";
    }
    cerr << "\n";
    map < int, vector <float> > edge_features;
    // find pairwise features
    // assert(edgeFeatNames.size ()<100); // some error in setting flag can cause trouble
    //   efeatfile<<"#"<<NUM_ASSOCIATIVE_FEATS<<endl;
    int edgecount = 0;
    for (map< int, vector<int> >::iterator it = neighbor_map.begin(); it != neighbor_map.end(); it++) {

        edge_features.clear();
        get_pair_features((*it).first, (*it).second, distance_matrix, segment_num_index_map, spectralProfiles, edge_features, tree);
        edgecount++;
        //   if(edgecount==1)
        //   {
        //       for(size_t i=0;i<edgeFeatNames.size ();i++)
        //           featfile<<"#"<<edgeFeatNames[i]<<endl;
        //}

        // print pair-wise features
        for (map< int, vector<float> >::iterator it2 = edge_features.begin(); it2 != edge_features.end(); it2++) {
            //  cerr << "edge: ("<< (*it).first << "," << (*it2).first << "):\t";
            featfile << segment_clouds[segment_num_index_map[(*it).first]].points[1].label << " " << segment_clouds[segment_num_index_map[(*it2).first]].points[1].label << " " << (*it).first << " " << (*it2).first;
            assert(edgeFeatNames.size() == (*it2).second.size());

            int featIndex = 0;
            for (vector<float>::iterator it3 = (*it2).second.begin(); it3 != (*it2).second.end(); it3++) {
                //    cerr << *it3 << "\t";
                if (BinFeatures) {
                    edgeFeatStumps[featIndex].writeBinnedValues(*it3, featfile, featIndex);
                } else {
                    featfile << " " << featIndex << ":" << *it3;
                }
                featIndex++;

                //                efeatfile << "\t" <<*it3 ;
            }
            //cerr << endl;
            featfile << endl;
        }

    }


    //    cout << "DONE!!\n";
    // write features to file
    /*    for (vector<float>::iterator it = features.begin(); it < features.end(); it++) {
          outfile << *it << "\t";
        }
        outfile << "\n";
     */
    //labelfile.close();
    featfile.close();
    //    std::ofstream  featfile;
    featfile.open(("temp." + featfilename).data());
    featfile << featfilename;
    featfile.close();
    string command = "../svm-python-v204/svm_python_classify --m svmstruct_mrf --l micro --lm nonassoc --cm sumLE1.IP --omf ../svm-python-v204/" + environment + "_objectMap.txt temp." + featfilename + " ../svm-python-v204/" + environment + "Model pred." + featfilename + " > out." + featfilename;
    system(command.data());

    std::ifstream predLabels;
    predLabels.open(("pred." + featfilename).data()); // open the file containing predictions
    map<int, int> segIndex2Label;
    
/*    vector<int> classes;
    classes.push_back(7);
    classes.push_back(13);
    vector<PointXYZI> maximas;
    lookForClass(classes, cloud, spectralProfiles, segIndex2Label, segment_clouds, scene_num, maximas);
 */ 
    cloudVector.push_back(cloud);
    spectralProfilesVector.push_back(spectralProfiles);
    segment_cloudsVector.push_back(segment_clouds);
    sceneNumVector.push_back(scene_num);
    parseAndApplyLabels(predLabels, cloud, segment_clouds, segIndex2Label);
    segIndex2LabelVector.push_back(segIndex2Label);
    predLabels.close();
    writer.write<pcl::PointXYZRGBCamSL > (featfilename + ".pcd", cloud, true);
    sensor_msgs::PointCloud2 cloudMsg;
    toROSMsg(cloud, cloudMsg);
    pub.publish(cloudMsg);


    //   efeatfile.close();



}

/*
OpenNIListener::OpenNIListener( ros::NodeHandle nh,  const char* visual_topic, 
                               const char* depth_topic, const char* info_topic, 
                               const char* cloud_topic, const char* filename , unsigned int step ) 
{
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
 // cloud_sub_=nh.subscribe(cloud_topic,2,cameraCallback);
//  ROS_INFO_STREAM("OpenNIListener listening to " << visual_topic << ", " << depth_topic \
                   << ", " << info_topic << " and " << cloud_topic << "\n"); 

 // bag_.open(filename, rosbag::bagmode::Write);

//  matcher_ = new cv::BruteForceMatcher<cv::L2<float> >() ;
/*  pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2> (
            "/rgbdslam/input/points", 10);
  pub_info_ = nh.advertise<sensor_msgs::CameraInfo> (
            "/rgbdslam/input/camera_info", 10);
  pub_visual_ = nh.advertise<sensor_msgs::Image> (
           "/rgbdslam/input/image_mono", 10);
  pub_depth_ = nh.advertise<sensor_msgs::Image> (
           "/rgbdslam/input/depth_image", 10);
//

}
 */

void readLabelList(const string & file) {
    string line;
    ifstream myfile(file.data());

    if (!myfile.is_open()) {
        cerr << "cound not find the file:" << file << " which stores the ranges for binning the features .. you should run this program from the folder scene_processing(do roscd scene_processing), or put the binning info files: binStumpsN.txt(for node features) and binStumpsE.txt(for edge features) in current folder and rerun ... exiting with assertion failure" << endl;
    }
    assert(myfile.is_open());

    while (myfile.good()) {
        getline(myfile, line);
        //        cout << line << endl;
        if (line.length() > 0) {
            char_separator<char> sep(",");
            tokenizer<char_separator<char> > tokens(line, sep);
            

            BOOST_FOREACH(string t, tokens) {
                labelsToFind.push_back(lexical_cast<int>(t.data()));
                labelsToFindBitset.set(lexical_cast<int>(t.data()));
            }
        }

    }
    myfile.close();
    //for ( vector<int>::iterator it=labelList.begin() ; it < labelList.end(); it++ )
    //  cout << " " << *it;
    //cout << endl;

}

int step = 1;

void processPointCloud(/*const sensor_msgs::ImageConstPtr& visual_img_msg, 
                                     const sensor_msgs::ImageConstPtr& depth_img_msg,   
                                     const sensor_msgs::CameraInfoConstPtr& cam_info,*/
        const sensor_msgs::PointCloud2ConstPtr& point_cloud) {

    static int callback_counter_ = 0;
    callback_counter_++;
    ROS_INFO("Received frame from kinect");
    if (++callback_counter_ % step == 0) {
        ROS_INFO("accepted it");

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGBCamSL>::Ptr cloud_seg_ptr(new pcl::PointCloud<pcl::PointXYZRGBCamSL > ());
        pcl::fromROSMsg(*point_cloud, cloud);
        convertType(cloud, *cloud_seg_ptr, VectorG(0,0,0), 0);
        assert(cloud_seg_ptr->size() == 640 * 480);
        segmentInPlace(*cloud_seg_ptr);
        globalTransform.transformPointCloudInPlaceAndSetOrigin(*cloud_seg_ptr);
        write_feats(globalTransform, cloud_seg_ptr, callback_counter_);
        sceneToAngleMap[callback_counter_] = currentAngle;

    } else
        ROS_INFO("rejected it");
    
}

void printLabelsToLookFor()
{
  cout << "Labels to look for: " ;
  for (size_t i = 0; i < labelsToLookFor.size(); i++)
  {
     cout << labelsToLookFor.at(i) << ", ";
  }
  cout << "\n";
}

void printLabelsFound(int turnCount){

    labelsFoundFile << turnCount ; 
    // print out the list of labels found
    cout << "Labels Found:\n";
    for (boost::dynamic_bitset<>::size_type i = 0; i < labelsToFindBitset.size(); i++)
    {
        if(labelsToFindBitset.test(i) ){
            std::cout << i << ":" << labelsFound[i] << "," ;
            labelsFoundFile << "\t"<< i << ":" << labelsFound[i]    ;
        }
    }
    std::cout << std::endl;
    labelsFoundFile << endl;
}

void getMovement(bool lookFor){
    objCount = 0;
    labelsToLookFor.clear();
      // for all the classes not found run look for class in each of the predicted frames
    for(boost::dynamic_bitset<>::size_type k = 0; k < labelsFound.size(); k++){
        
        if(!labelsFound.test(k)){
			labelsToLookFor.push_back(k);
        }
    }
    if (lookFor) {
        locations.clear();
        int cloudCount = 0;
        while (cloudVector.size() > 0) {

            vector<pcl::PointXYZI> frame_maximas;
            lookForClass(labelsToLookFor, cloudVector.at(0), spectralProfilesVector.at(0), segIndex2LabelVector.at(0), segment_cloudsVector.at(0), sceneNumVector.at(cloudCount), frame_maximas);
            locations.push_back(frame_maximas);
            // remove the point clouds in which maximas are found
            cloudVector.erase(cloudVector.begin());
            spectralProfilesVector.erase(spectralProfilesVector.begin());
            segIndex2LabelVector.erase(segIndex2LabelVector.begin());
            segment_cloudsVector.erase(segment_cloudsVector.begin());

            cloudCount++;
        }

        // find the maximum for each class and the corresponding frames     


        for (int lcount = 0; lcount < labelsToLookFor.size(); lcount++) {

            int label = labelsToLookFor.at(lcount);
            for (int i = 0; i < locations.size(); i++) {
                if (locations.at(i).at(lcount).intensity > maximas.at(label).intensity) {
                    maximas.at(label).x = locations.at(i).at(lcount).x;
                    maximas.at(label).y = locations.at(i).at(lcount).y;
                    maximas.at(label).z = locations.at(i).at(lcount).z;
                    maximas.at(label).intensity = locations.at(i).at(lcount).intensity;
                    maximaFrames.at(label) = sceneNumVector.at(i);
                    maximaChanged.set(label, true);
                    cout << "Maxima for label: " << label << " changed to scene num: " << sceneNumVector.at(i) << endl;
                }
            }
        }
        sceneNumVector.clear();

        // remove any classes already looked for and maxima location didnt change
        for (int lcount = 0; lcount < labelsToLookFor.size(); lcount++) {
            if (!maximaChanged.test(labelsToLookFor.at(lcount)) && labelsAlreadyLookedFor.test(labelsToLookFor.at(lcount))) {
                labelsToLookFor.erase(labelsToLookFor.begin() + lcount);
                lcount--;
            }
        }
    }
    maximaChanged.reset(); 
    rotations.clear();
    translations.clear(); 
    rotations.resize(labelsToLookFor.size(),0);
    translations.resize(labelsToLookFor.size(),0);
    
    for(int lcount =0; lcount< labelsToLookFor.size(); lcount++)
    {
        int label = labelsToLookFor.at(lcount);
        double angle =  sceneToAngleMap[maximaFrames.at(label)]; // this angle is wrt to the initial point
        double theta= (double)(atan(maximas.at(label).y/maximas.at(label).x)*180/PI);
        cout << "the angle within the frame is: " << theta << endl;
        angle = angle+theta;
        rotations.at(lcount) =  angle;
        
    }
    cout << "Rotations calculated:" << endl;
    // printing the labels to look for, frame numbers and rotation angles
    for(int lcount =0; lcount< labelsToLookFor.size(); lcount++)
    {
        cout << lcount << ". label: "  << labelsToLookFor.at(lcount) << " frame: " << maximaFrames.at(labelsToLookFor.at(lcount)) << " rotation: " << rotations[lcount] << endl;
    }
    
}

void robotMovementControl(const sensor_msgs::PointCloud2ConstPtr& point_cloud){
        double forwardDistance=0.5;
    if(all_done){ exit(0); }
    
    if(turnCount < MAX_TURNS && labelsFound.count() < NUM_CLASSES ) // if more labels are to be found and the turn count is less than the max
    {
        ROS_INFO("processing %d cloud.. \n",turnCount+1);
        processPointCloud (point_cloud);
         
        // turn robot and increase the count
        if(turnCount < MAX_TURNS-1){ 
        robot->turnLeft(40,2);
        currentAngle += 40;
        cout << "current Angle now is "  << currentAngle<< endl;
        }
        turnCount++;
        printLabelsFound(turnCount);
        // if all classes found then return
        if (labelsFound.count()== NUM_CLASSES) {
            all_done = true;
            return;
        }
        
        
        return;
    }
    // if all initial turns are done, and the there are some more labels to find
    if ( turnCount == MAX_TURNS && labelsFound.count() < NUM_CLASSES) {
        //look for the remaining labels and get the corresponding rotation and translation motions
       
        getMovement(true);
        printLabelsToLookFor();
     
        originalScan = false;
        // do not process the current cloud but move the robot to the correct position
        double angle = rotations[0] - currentAngle;
        robot->turnLeft(angle, 0);
        //robot->moveForward(1.0,2);
        currentAngle = rotations[0];
        cout << "current Angle now is "  << currentAngle<< endl;
        cout << "Looking for object " << labelsToLookFor.at(objCount)<< endl;
        labelsAlreadyLookedFor.set(labelsToLookFor.at(objCount),true);
        objCount++;
        //robot->moveForward(translations[0], 2);
        
        rotations.erase(rotations.begin());
        translations.erase(translations.begin());
        turnCount++;
        return;
    }
    
    if ( !translationState && labelsFound.count() < NUM_CLASSES   && objCount <= labelsToLookFor.size() ){
        ROS_INFO("processing %d cloud.. \n",turnCount+1);
        processPointCloud (point_cloud);
        printLabelsFound(turnCount);
        printLabelsToLookFor();
        cout << "foundAny : " << foundAny << " objCount: " << objCount << "size of labelsToLookFor: " << labelsToLookFor.size() << endl;
        // call get movement only if new labels are found 
        if(foundAny && objCount != labelsToLookFor.size() ){
		   cout << "calling getMovement  when found any "  <<  endl;
           getMovement(true);
           printLabelsToLookFor();
        }
        if(objCount == labelsToLookFor.size())
        {
            cout << "calling getMovement when objCount == labelsToLookFor.size()" << endl;
		    getMovement(false);
            printLabelsToLookFor();
            translationState = true;
            cout<< "switching to moving forward mode"<<endl;
        }
        
        // if there are still movements left, move the robot else all_done
        if(!rotations.empty()){
           double angle = rotations[0] - currentAngle;
        //robot->moveForward(-1.0,0);
           robot->turnLeft(angle,0);

           if(translationState ){
               robot->moveForward(forwardDistance,2);  
           }
           
        //robot->moveForward(1.0,2);
           currentAngle = rotations[0];
           cout << "current angle now is " << currentAngle << endl;
           cout << "Looking for object " << labelsToLookFor.at(objCount) << " at " << objCount << endl;
           labelsAlreadyLookedFor.set(labelsToLookFor.at(objCount),true);
           objCount++;
          // robot->moveForward(translations[0],2);
           rotations.erase(rotations.begin());
           translations.erase(translations.begin());
           turnCount++;

        }
        return;
                
    }
    if (translationState && turnCount < MAX_TRYS && labelsFound.count() < NUM_CLASSES && objCount <= labelsToLookFor.size()){
        ROS_INFO("processing %d cloud.. \n",turnCount+1);
        processPointCloud (point_cloud);
        printLabelsToLookFor();
        // do not update the movement now
        //if(foundAny){
        //   getMovement();
        //}

        printLabelsFound(turnCount);
        // if there are still movements left, move the robot else all_done
        if(!rotations.empty()){
           double angle = rotations[0] - currentAngle;
        robot->moveForward(-forwardDistance,0);
           robot->turnLeft(angle,0);
        robot->moveForward(+forwardDistance,2);
           currentAngle = rotations[0];
           cout << "current angle now is " << currentAngle << endl;
           cout << "Looking for object " << labelsToLookFor.at(objCount)<< endl;
           labelsAlreadyLookedFor.set(labelsToLookFor.at(objCount),true);
           objCount++;
          // robot->moveForward(translations[0],2);
           rotations.erase(rotations.begin());
           translations.erase(translations.begin());
           turnCount++;

        }else{all_done = true;}
                
    } else{ 
        all_done = true;
    }
    
    
}

void robotMovementControlSingleObject(const sensor_msgs::PointCloud2ConstPtr& point_cloud){
    double angle = 0;
    
    // first frame just pick a random direction and move to it
    if(turnCount== 0){
       srand ( time(NULL) );

        angle = rand() % SPAN - SPAN/2;
        robot->turnLeft(angle,2);
        currentAngle = angle;
        cout << "current Angle now is "  << currentAngle<< endl;
        turnCount ++;
        return;
    }
     // second frame onwards, 
    else if (turnCount < MAX_TRYS && labelsFound.count() < NUM_CLASSES){
        // process the cloud
        ROS_INFO("processing %d cloud.. \n",turnCount+1);
        processPointCloud (point_cloud);
        
        // look for the class
        getMovement(true);
        // if maxima changed the look-for/rotations list will be non-empty
        if(labelsToLookFor.size() != 0){
             printLabelsToLookFor();
             double angle = rotations[0] - currentAngle;
             robot->turnLeft(angle, 0);
             currentAngle = rotations[0];
             cout << "current Angle now is "  << currentAngle<< endl;
             cout << "Looking for object " << labelsToLookFor.at(objCount)<< endl;
             labelsAlreadyLookedFor.set(labelsToLookFor.at(objCount),true);
             objCount++;
             rotations.erase(rotations.begin());
             

        }else {// else pic a random angle and move
            angle = rand() % SPAN - SPAN/2;
            robot->turnLeft(angle-currentAngle,2);
            currentAngle = angle;
            cout << "current Angle now is "  << currentAngle<< endl;
        }  
        // increase the count
        turnCount++;
        return;
        
    }else {exit (0);}
    
      
}

void robotMovementControlRandom(const sensor_msgs::PointCloud2ConstPtr& point_cloud){
    double angle = 0;
    
    // first frame just pick a random direction and move to it
    if(turnCount== 0){
       srand ( time(NULL) );

        angle = rand() % SPAN - SPAN/2;
        robot->turnLeft(angle,2);
        currentAngle = angle;
        cout << "current Angle now is "  << currentAngle<< endl;
        turnCount ++;
        return;
    }
     // second frame onwards, 
    else if (turnCount < MAX_TRYS && labelsFound.count() < NUM_CLASSES){
        // process the cloud
        ROS_INFO("processing %d cloud.. \n",turnCount+1);
        processPointCloud (point_cloud);
        
            angle = rand() % SPAN - SPAN/2;
            robot->turnLeft(angle-currentAngle,2);
            currentAngle = angle;
            cout << "current Angle now is "  << currentAngle<< endl;
        turnCount++;
        return;
        
    }else {exit (0);}
    
      
}

Vector3d getCenter(pcl::PointCloud<PointT> & cloud)
{
    Vector3d max;
    Vector3d min;
        for (size_t i = 0; i < cloud.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (max(j) < cloud.points[i].data[j])
                max(j) = cloud.points[i].data[j];

            if (min(j) > cloud.points[i].data[j])
                min(j) = cloud.points[i].data[j];
        }
    }
    return (max+min)/2;
}

void evaluate(string inp, string out, int oclass)
{
        pcl::PointCloud<PointT> outc;
        pcl::io::loadPCDFile<PointT>(out, outc);
    std::vector<pcl::PointCloud<PointT> > segment_clouds;
        pcl::PointCloud<PointT> cloud;
        pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile<PointT>(inp, cloud);
    map<int, int> segIndex2Label;
    int max_segment_num;
    for (size_t i = 0; i < cloud.points.size(); i++) {
        counts[cloud.points[i].segment]++;
        if (max_segment_num < cloud.points[i].segment) {
            max_segment_num = (int) cloud.points[i].segment;
        }
    }


    int index_ = 0;
    vector<SpectralProfile> spectralProfiles;
    std::cerr << "max_seg num:" << max_segment_num << "," << cloud.points.size() << endl;
    for (int seg = 1; seg <= max_segment_num; seg++) {
         if(counts[seg]<=MIN_SEG_SIZE)
           continue;
        SpectralProfile temp;
        apply_segment_filter(cloud, *cloud_seg, seg, temp);
        getSpectralProfileCent(*cloud_seg,temp);

        //if (label!=0) cout << "segment: "<< seg << " label: " << label << " size: " << cloud_seg->points.size() << endl;
        if (!cloud_seg->points.empty() && cloud_seg->points.size() > MIN_SEG_SIZE) {
            segIndex2Label[index_]=labelMap[cloud_seg->points[1].label];
            //std::cout << seg << ". Cloud size after extracting : " << cloud_seg->points.size() << std::endl;
            segment_clouds.push_back(*cloud_seg);
            pcl::PointCloud<PointT>::Ptr tempPtr(new pcl::PointCloud<PointT > (segment_clouds[segment_clouds.size() - 1]));
            temp.cloudPtr = tempPtr;

            spectralProfiles.push_back(temp);
           // segment_num_index_map[cloud_seg->points[1].seugment] = index_;
            index_++;
        }
    }
    
    vector<int> classes;
    classes.push_back(oclass);
    vector<PointXYZI> maximas;
    lookForClass(classes, cloud, spectralProfiles, segIndex2Label, segment_clouds, 0, maximas);
    cout<<"heatMax"<<maximas.at(0)<<endl;
    
    Vector3d predL(maximas.at(0).x,maximas.at(0).y,maximas.at(0).z);
    Vector3d sump(0,0,0);
    //    Vector3d predL=getCenter(outc);
    int count=0;
    int targetLabel=invLabelMap[oclass+1];
    double minDist=DBL_MAX;
    cout<<"keyboard:"<<targetLabel<<endl;
    for (size_t i = 0; i < outc.points.size(); i++) {
        if(outc.points[i].label==targetLabel)
        {
                Vector3d point(outc.points[i].x,outc.points[i].y,outc.points[i].z);
                sump+=point;
                count++;
                double dist=(predL-point).norm();
                if(minDist>dist)
                    minDist=dist;
        }
    }
    
    cout<<count<<" points had label keyboard"<<endl;
    Vector3d centroid=sump/count;
    cout<<"minDist"<<minDist<<endl;
    cout<<"centDist"<<(predL-centroid).norm()<<endl;
  
    
}
int main(int argc, char** argv) {
    readWeightVectors();
    ros::init(argc, argv, "hi");
    //  unsigned int step = 10;
    environment = "office";
    if (argc > 1) environment = argv[1];
    cout << "using evv= " << environment << endl;
    ros::NodeHandle n;
    robot = new MoveRobot(n);


    string labelsFoundFilename = environment+ "labels_found.txt";
    labelsFoundFile.open(labelsFoundFilename.data());
    // initialize the maximas
    for(size_t i = 0; i < maximas.size(); i++)
    {
        maximas.at(i).intensity = -FLT_MAX;
    }
    

    //Instantiate the kinect image listener
    if (BinFeatures) {
        readAllStumpValues();
    }

    readLabelList(environment + "_to_find.txt");
    labelsFound = labelsToFindBitset;
    labelsFound.flip();
    //for (boost::dynamic_bitset<>::size_type i = 0; i < labelsFound.size(); ++i)
      //  std::cout << labelsFound[i];

    readInvLabelMap(invLabelMap, "../svm-python-v204/" + environment + "_labelmap.txt");
    globalTransform = readTranform("globalTransform.bag");
    pub = n.advertise<sensor_msgs::PointCloud2 > ("/scene_labler/labeled_cloud", 10);
    //    std_msgs::String str;
    //    str.data = "hello world";
    ros::Subscriber cloud_sub_ = n.subscribe("/camera/rgb/points", 1, processPointCloud);

    ros::spin();
  

}


