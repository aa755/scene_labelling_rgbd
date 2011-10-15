#include "includes/CovarianceMatrix.h"
#include"feat_utils.h"

bool UseVolFeats=false;
vector<OriginalFrameInfo*> originalFrames;

    pcl::PointCloud<PointT> cloudUntransformed;
    pcl::PointCloud<PointT> cloudUntransformedUnfiltered;

    bool addNodeHeader=true;;
    bool addEdgeHeader=true;
    
    vector<std::string> nodeFeatNames;
    vector<std::string> edgeFeatNames;

    void addToNodeHeader(std::string featName,size_t numTimes=1)
    {
      //char featNameBuf[50];
     // assert(featName.length ()<40);
      if(addNodeHeader)
        {
          for(size_t i=0;i<numTimes;i++)
            {
       //       sprintf (featNameBuf,"%s%d",featName,i);
                nodeFeatNames.push_back (featName+boost::lexical_cast<std::string>(i));
            }
        }
    }
    
    inline void addToEdgeHeader(std::string featName,size_t numTimes=1)
    {
      //char featNameBuf[50];
      //assert(featName.length ()<40);
      if(addEdgeHeader)
        {
          for(size_t i=0;i<numTimes;i++)
            {
              //sprintf (featNameBuf,"%s%d",featName,i);
                edgeFeatNames.push_back (featName+boost::lexical_cast<std::string>(i));
            }
        }
    }
    
    

void apply_segment_filter(pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment) {
    //ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
//    outcloud.points = incloud.points;
    outcloud.points.resize ( incloud.points.size() );

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
    if(j>=0)
        outcloud.points.resize ( j+1 );
    else
       outcloud.points.clear ();
}

/** it also discards unlabeled segments
 */
void apply_segment_filter_and_compute_HOG(pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment,SpectralProfile & feats) {
    //ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
//    outcloud.points = incloud.points;
    outcloud.points.resize ( incloud.points.size() );

    assert(originalFrames.size ()>0);
    int numFrames=originalFrames.size ();
    
    int sceneIndexCounts[numFrames];
    
    for(size_t f=0;f<numFrames;f++)
      sceneIndexCounts[f]=0;
    
    vector<size_t> indices;
    int j = -1;
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].segment == segment && incloud.points[i].label>0) {
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
          assert(incloud.points[i].cameraIndex<numFrames);
          sceneIndexCounts[incloud.points[i].cameraIndex]+=1;
          indices.push_back (i);

            //     std::cerr<<segment_cloud.points[j].label<<",";
        }
    }
    
   // cout<<j << ","<<segment<<endl;
    if(j>=0)
        outcloud.points.resize ( j+1 );
    else
      {
        outcloud.points.clear ();
        return;
      }
    int max=-1;
    size_t maxIndex;
    
    for(size_t f=0;f<numFrames;f++)
      if(max<sceneIndexCounts[f])
        {
           max=sceneIndexCounts[f];
           maxIndex=f;
        }
    assert(max>=0);
    cout<<"segment index:"<<segment<<"with "<<indices.size ()<<" points"<<" main frame "<<maxIndex<<" with numMax="<<max<<endl;
    OriginalFrameInfo::findHog (maxIndex, indices, incloud, feats.avgHOGFeatsOfSegment,originalFrames,cloudUntransformed);
    
}

void apply_notsegment_filter(const pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment) {
    //ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
//    outcloud.points = incloud.points;
    outcloud.points.resize ( incloud.points.size() );

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
    assert(j>=0);
    outcloud.points.resize ( j+1 );
}

void apply_camera_filter(const pcl::PointCloud<PointT> &incloud,  pcl::PointCloud<PointT> &outcloud, int camera) {
    //ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
//    outcloud.points = incloud.points;
    outcloud.points.resize ( incloud.points.size() );

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
    assert(j>=0);
    outcloud.points.resize ( j+1 );
}


float getDistanceToBoundary( const pcl::PointCloud<PointT> &cloud1, const pcl::PointCloud<PointT> &cloud2)
{
    float max_distance = 0;
    for(size_t i =0; i< cloud1.points.size(); ++i)
    {
        float pdist = 0;
        for(size_t j =0; j< cloud2.points.size(); ++j){
            float point_dist = pow((cloud1.points[i].x - cloud2.points[j].x),2) + pow((cloud1.points[i].y - cloud2.points[j].y),2) + pow((cloud1.points[i].z - cloud2.points[j].z),2);
            float distance = (pow(cloud2.points[j].distance,2 ) - pow(cloud1.points[i].distance,2) - ( point_dist ))/(2*cloud1.points[i].distance);
            if (pdist < distance) pdist = distance;
            if (max_distance < distance) max_distance = distance;
           // cout << distance << " " << pdist<< " " << max_distance<< endl;
        }
       // cloud1.points[i].distance = pdist;
    }
    return max_distance;
}



pair<float,int>  getSmallestDistance (const pcl::PointCloud<PointT> &cloud1,const pcl::PointCloud<PointT> &cloud2)
{
  float min_distance = FLT_MAX;
  int min_index = 0;
  pcl::PointCloud<PointT>::Ptr small_cloud;
  pcl::PointCloud<PointT>::Ptr big_cloud;
  if (cloud1.points.size() > cloud2.points.size()){
    pcl::PointCloud<PointT>::Ptr cloud_ptr1(new pcl::PointCloud<PointT > (cloud1));
    pcl::PointCloud<PointT>::Ptr cloud_ptr2(new pcl::PointCloud<PointT > (cloud2));
    small_cloud = cloud_ptr2;
    big_cloud = cloud_ptr1;
  }else {
    pcl::PointCloud<PointT>::Ptr cloud_ptr1(new pcl::PointCloud<PointT > (cloud1));
    pcl::PointCloud<PointT>::Ptr cloud_ptr2(new pcl::PointCloud<PointT > (cloud2));
    small_cloud = cloud_ptr1;
    big_cloud = cloud_ptr2;
  }

  pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);//= boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
  //initTree (0, tree);
  tree->setInputCloud (big_cloud );// ,indicesp);
  std::vector<int> nn_indices;
  nn_indices.resize(2);
  std::vector<float> nn_distances;
  nn_distances.resize(2);
  //float tolerance = 0.3;
  
  for (size_t i = 0; i < small_cloud->points.size (); ++i)
  {
  
	//if (!tree->radiusSearch ((*small_cloud).points[i], tolerance, nn_indices, nn_distances))
	tree->nearestKSearch (small_cloud->points[i], 2 , nn_indices, nn_distances);
  	
	for (size_t j = 0; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
      	{
			
      		//float distance = pow(cloud1.points[i].x - cloud2.points[nn_indices[j]].x,2) + pow(cloud1.points[i].y - cloud2.points[ nn_indices[j]].y,2) + pow(cloud1.points[i].z - cloud2.points[ nn_indices[j]].z,2);
     		 //cout<< "i,j = " << i << "," << j<< " dist = " <<distance << endl;
                 //float a = nn_distances[j];
                 //cout<< nn_distances[j] << " "<< a << endl;
      		 if (min_distance > nn_distances[j])
                 {
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
  return make_pair(sqrt(min_distance),min_index) ;
}


void get_neighbors ( const std::vector<pcl::PointCloud<PointT> > &segment_clouds, map< pair <int,int> , float > &distance_matrix, map <int , vector <int> > &neighbor_map )
{
   float tolerance =0.15;
// get distance matrix
    for (size_t i = 0; i< segment_clouds.size(); i++)
    {
      for (size_t j = i+1; j < segment_clouds.size() ; j++)
      {   
         pair<float,int> dist_pair = getSmallestDistance(segment_clouds[i],segment_clouds[j]);
         distance_matrix[make_pair(segment_clouds[i].points[1].segment,segment_clouds[j].points[1].segment)] = dist_pair.first;
         distance_matrix[make_pair(segment_clouds[j].points[1].segment,segment_clouds[i].points[1].segment)] = dist_pair.first;
      }
 //     std::cerr<< "size of segment " << i << " : " << segment_clouds[i].points.size() << "\t and label is: " << segment_clouds[i].points[1].label <<"\n";
    }
// get neighbour map
    for ( map< pair <int,int> , float >::iterator it=distance_matrix.begin() ; it != distance_matrix.end(); it++ )
    {   
      if((*it).second < tolerance)  neighbor_map[(*it).first.first].push_back((*it).first.second);
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
}


void getSegmentDistanceToBoundary( const pcl::PointCloud<PointT> &cloud , map<int,float> &segment_boundary_distance){
    pcl::PointCloud<PointT>::Ptr cloud_rest(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<PointT>::Ptr cloud_cam(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (cloud));

    int cnt =0;
    // find all the camera indices// find the max segment number

    map<int,int> camera_indices;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        camera_indices[(int) cloud.points[i].cameraIndex] = 1;
    }
    // for every camera index .. apply filter anf get the point cloud
    for (map<int,int>::iterator it = camera_indices.begin(); it != camera_indices.end();it++)
    {
        int ci = (*it).first;
        apply_camera_filter(*cloud_ptr,*cloud_cam,ci);

        // find the segment list
        map<int,int> segments;
        for (size_t i = 0; i < cloud_cam->points.size(); ++i) {
            if( cloud_cam->points[i].label != 0)
                segments[(int) cloud_cam->points[i].segment] = 1;
        }
        for (map<int,int>::iterator it2 = segments.begin(); it2 != segments.end();it2++){
            cnt++;
            int segnum = (*it2).first;
            apply_segment_filter(*cloud_cam,*cloud_seg,segnum);
            apply_notsegment_filter(*cloud_cam,*cloud_rest,segnum);
            float bdist = getDistanceToBoundary(*cloud_seg,*cloud_rest);

            map<int , float>::iterator segit = segment_boundary_distance.find(segnum);
            if(segit== segment_boundary_distance.end() || bdist> segment_boundary_distance[segnum]  )
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

void getCentroid(const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &centroid)
{
    centroid[0]=0;
    centroid[1]=0;
    centroid[2]=0;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
			//assert(cloud.points[i].z>=0);
        centroid[0] += cloud.points[i].x;
        centroid[1] += cloud.points[i].y;
        centroid[2] += cloud.points[i].z;
			//assert(centroid[2]>=0);
    }
    centroid[0] = centroid[0]/(cloud.points.size()-1) ;
    centroid[1] = centroid[1]/(cloud.points.size()-1) ;
    centroid[2] = centroid[2]/(cloud.points.size()-1) ;
}

void getSpectralProfile(const pcl::PointCloud<PointT> &cloud, SpectralProfile &spectralProfile)
{
   Eigen::Matrix3d eigen_vectors;
   Eigen::Vector3d eigen_values;
   sensor_msgs::PointCloud2 cloudMsg2;
   pcl::toROSMsg (cloud,cloudMsg2);
   sensor_msgs::PointCloud cloudMsg;
   sensor_msgs::convertPointCloud2ToPointCloud(cloudMsg2,cloudMsg);
   //cloud_geometry::nearest::computePatchEigenNormalized (cloudMsg,eigen_vectors,eigen_values,spectralProfile.centroid);
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

  
  spectralProfile.setEigValues (eigen_values);
  float minEigV=FLT_MAX;
 
  for(int i=0;i<3;i++)
    {
      
          cout<<"eig value:"<<eigen_values(i)<<endl;
      if(minEigV>eigen_values(i))
        {
          minEigV=eigen_values(i);
          cout<<"min eig value:"<<minEigV<<endl;
          spectralProfile.normal=eigen_vectors.col(i);
          // check the angle with line joining the centroid to origin
          VectorG centroid(spectralProfile.centroid.x,spectralProfile.centroid.y,spectralProfile.centroid.z);
          VectorG camera=originalFrames[cloud.points[1].cameraIndex]->getCameraTrans().getOrigin();
          VectorG cent2cam=camera.subtract(centroid);
          VectorG normal(spectralProfile.normal[0],spectralProfile.normal[1],spectralProfile.normal[2]);
          if (normal.dotProduct(cent2cam) < 0)
          {
              // flip the sign of the normal
              spectralProfile.normal[0] = -spectralProfile.normal[0];
              spectralProfile.normal[1] = -spectralProfile.normal[1];
              spectralProfile.normal[2] = -spectralProfile.normal[2];
          }
        }
    }
  assert(minEigV==spectralProfile.getDescendingLambda (2));
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
    int numFeats=it->size();
    result.resize(numFeats);
    // set size of result vector
    
    vector<BinningInfo>::iterator binningInfo = binningInfos.begin();
    for (vector<vector<float> >::iterator ires = result.begin(); ires < result.end(); ires++,binningInfo++)
      {
        ires->resize(binningInfo->GetNumBins ());       
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

        assert(numFeats==it_point->size ());//missing features NOT allowed for now.
        
        for (vector<float>::iterator it_feature = it_point->begin(); it_feature < it_point->end(); it_feature++, binningInfo++, ires++) { // iterate over features of the point

            int bin = binningInfo->getBinIndex (*it_feature);
            
            //   ROS_INFO("%f %d %d",bin_size,bin,(*ires).size());
            
            (*ires)[bin] += 1;
        }
    }

    // normalize and print histogram
 //   std::cerr << "historam \n";
    
    int numPoints=descriptor_results.size ();
    
    int c1 = 0, c2 = 0;
    for (vector< vector<float> >::iterator i = result.begin(); i < result.end(); i++) {
        c1++;
   //     std::cerr << "histogram for feature:" << c1 << "\n";
        for (vector<float>::iterator i2 = i->begin(); i2 < i->end(); i2++) {
            c2++;
            *i2 = *i2 / numPoints;
            assert(*i2<=1.0);
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
 int num_bin_H=6;
 int num_bin_S=2;
 int num_bin_V=2;
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
    binnigInfos.push_back (BinningInfo(0,360,num_bin_H));
    binnigInfos.push_back (BinningInfo(0,1,num_bin_S));
    binnigInfos.push_back (BinningInfo(0,1,num_bin_V));
    get_feature_histogram(color_features, hist_features,binnigInfos);
    get_feature_average(color_features, avg_features);
    
    spectralProfileOfSegment.avgH=avg_features[0];
    spectralProfileOfSegment.avgS=avg_features[1];
    spectralProfileOfSegment.avgV=avg_features[2];

    concat_feats(features, hist_features);addToNodeHeader ("H_hist",num_bin_H);addToNodeHeader ("S_hist",num_bin_S);addToNodeHeader ("V_hist",num_bin_V);
    
    concat_feats(features, avg_features);addToNodeHeader ("HAvg");addToNodeHeader ("SAvg");addToNodeHeader ("VAvg");
}

void get_global_features(const pcl::PointCloud<PointT> &cloud, vector<float> &features, SpectralProfile & spectralProfileOfSegment) {
    
    Eigen::Vector4f min_p;
    Eigen::Vector4f max_p;



    // get bounding box features
    getSpectralProfile (cloud, spectralProfileOfSegment);
    
    getMinMax(cloud, min_p, max_p);
    float xExtent=max_p[0] - min_p[0];
    float yExtent=max_p[1] - min_p[1];
    float horizontalExtent=sqrt(xExtent*xExtent+yExtent*yExtent);
    float zExtent=max_p[2] - min_p[2];
    
    features.push_back(horizontalExtent);addToNodeHeader ("horizontalExtent");
    
    features.push_back(zExtent);addToNodeHeader ("zExtent");

    features.push_back(spectralProfileOfSegment.centroid.z);addToNodeHeader ("centroid_z");
    
    
    features.push_back (spectralProfileOfSegment.getNormalZComponent ());addToNodeHeader ("normal_z");
  
    //spectral saliency features
    features.push_back ((spectralProfileOfSegment.getLinearNess ()));addToNodeHeader ("linearness");
    features.push_back ((spectralProfileOfSegment.getPlanarNess ()));addToNodeHeader ("planarness");
    features.push_back ((spectralProfileOfSegment.getScatter ()));addToNodeHeader ("scatter");
    spectralProfileOfSegment.avgHOGFeatsOfSegment.pushBackAllFeats (features);addToNodeHeader ("HOG",31);
    
    
   
 
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

float get_occupancy_feature(const pcl::PointCloud<PointT> &cloud1, const pcl::PointCloud<PointT> &cloud2,  OcTreeROS & tree){
    //
    OcTreeROS::NodeType* treeNode;
    int occCount = 0;
    int totalCount = 0;
    int unknownCount = 0;
    for (int i =0 ; i <100 ; i++)
    {
        int p1Index =  rand() % cloud1.points.size() ;
        int p2Index =  rand() % cloud2.points.size();
        VectorG p1 (cloud1.points[p1Index]);
        VectorG p2 (cloud2.points[p2Index]);
        double distance = (p2.subtract(p1)).getNorm();
        int count = 0;
        for ( double r = 0.05 ; r<= distance-0.05 ; r+=0.05)
        {
            count ++;
            VectorG point = p1.add( ( p2.subtract(p1).normalizeAndReturn() ).multiply(r));
            pcl::PointXYZ pt (point.v[0],point.v[1],point.v[2]);
            treeNode = tree.search(pt);
            if (treeNode){
                if (treeNode->getOccupancy() > 0.5){occCount++;}
                //cout << "Occupancy of node at ("<< pt.x << "," << pt.y<< "," << pt.z << ") = " << treeNode->getOccupancy() << " \n";
            }
            else{
                unknownCount++;
                //cout << "ERROR: OcTreeNode not found (NULL)\n";
            }
            
        }
        if(count ==0)
        {
            VectorG point = p1.add(p2.subtract(p1).multiply(0.5));
            pcl::PointXYZ pt (point.v[0],point.v[1],point.v[2]);
            treeNode = tree.search(pt);
            if (treeNode){
                if (treeNode->getOccupancy() > 0.5){occCount++;}
                //cout << "Occupancy of node at ("<< pt.x << "," << pt.y<< "," << pt.z << ") = " << treeNode->getOccupancy() << " \n";
            }
            else{
                unknownCount++;
                //cout << "ERROR: OcTreeNode not found (NULL)\n";
            }
            count ++;
        }
        totalCount += count;       
    }
    cout << "seg1:" << cloud1.points[1].segment<< " label1: " << cloud1.points[1].label <<  " seg2:" << cloud2.points[1].segment<< " label2: " << cloud2.points[1].label << endl;
    cout << "total:" << totalCount << " unknown:"  << unknownCount << " occupied:" << occCount  << endl;
    return (float)unknownCount/(float)totalCount;
}

/*

void get_convex_regions (map <int , vector <int> > &neighbor_map, std::map<int,int>  &segment_num_index_map, vector<SpectralProfile> & spectralProfiles){
    
    // for every segment
    for(map<int , vector <int> >::iterator it = neighbor_map.begin(); it != neighbor_map.end(); it++)
    {
                
        segment_id = (*it).first;
        
        SpectralProfile segment1Spectral=spectralProfiles[segment_num_index_map[segment_id]];
    // explore list = neighbour list
    // convexity list = empty
    // if element in neighbour list is convex add it to convexity list and add its neighbours not already explored to explore list
    
    }
    
    for (vector<int>::iterator it = neighbor_list.begin(); it != neighbor_list.end(); it++) {

        int seg2_id = *it;
        SpectralProfile segment2Spectral=spectralProfiles[segment_num_index_map[seg2_id]];
    segment1Spectral.getConvexity (segment2Spectral,distance_matrix[make_pair(segment_id,seg2_id)] )
 
}
**/
int NUM_ASSOCIATIVE_FEATS=4+31+1;
void get_pair_features( int segment_id, vector<int>  &neighbor_list,
                        map< pair <int,int> , float > &distance_matrix,
						std::map<int,int>  &segment_num_index_map,
                        vector<SpectralProfile> & spectralProfiles,
                        map < int, vector<float> > &edge_features,
                        OcTreeROS & tree) {

    SpectralProfile segment1Spectral=spectralProfiles[segment_num_index_map[segment_id]];

    for (vector<int>::iterator it = neighbor_list.begin(); it != neighbor_list.end(); it++) {

        int seg2_id = *it;
        SpectralProfile segment2Spectral=spectralProfiles[segment_num_index_map[seg2_id]];
        
        
        
        
        
        //here goes the associative features:
        segment1Spectral.pushHogDiffFeats (segment2Spectral,edge_features[seg2_id]); addToEdgeHeader ("HOGDiff",31);
        
        edge_features[seg2_id].push_back(segment1Spectral.getHDiffAbs (segment2Spectral)); addToEdgeHeader ("HDiffAbs");
        
        edge_features[seg2_id].push_back(segment1Spectral.getSDiff (segment2Spectral)); addToEdgeHeader ("SDiff");
        
        edge_features[seg2_id].push_back(segment1Spectral.getVDiff (segment2Spectral)); addToEdgeHeader ("Viff");

        edge_features[seg2_id].push_back(segment1Spectral.getCoplanarity (segment2Spectral)); addToEdgeHeader ("Coplanarity");
        

        edge_features[seg2_id].push_back(segment1Spectral.getConvexity (segment2Spectral,distance_matrix[make_pair(segment_id,seg2_id)] ));addToEdgeHeader ("convexity");

        assert(edge_features[seg2_id].size ()==NUM_ASSOCIATIVE_FEATS);
        
        //here goes the non-associative features
        
        
        edge_features[seg2_id].push_back(segment1Spectral.getHorzDistanceBwCentroids (segment2Spectral));addToEdgeHeader ("centroid_horz_diff");
        // difference in z coordinates of the centroids
        edge_features[seg2_id].push_back(segment1Spectral.getVertDispCentroids (segment2Spectral));addToEdgeHeader ("centroid_z_diff");
        //cerr << "edge feature for edge (" << seg1_id << "," << seg2_id << ")  = " << centroid_z_diff << endl;
        
        // distance between closest points
        edge_features[seg2_id].push_back(distance_matrix[make_pair(segment_id,seg2_id)]);addToEdgeHeader ("dist_closest");

        // difference of angles with vertical
        edge_features[seg2_id].push_back(segment1Spectral.getAngleDiffInRadians (segment2Spectral));addToEdgeHeader ("AngleDiff");
		
		// dot product of normals
        edge_features[seg2_id].push_back(segment1Spectral.getNormalDotProduct (segment2Spectral));addToEdgeHeader ("NormalDotProduct");

        
        edge_features[seg2_id].push_back(segment1Spectral.getInnerness (segment2Spectral));addToEdgeHeader ("Innerness");
    
        // occupancy fraction feature
        if(UseVolFeats)
        {
                edge_features[seg2_id].push_back(get_occupancy_feature( *(segment1Spectral.cloudPtr), *(segment2Spectral.cloudPtr), tree ) );addToEdgeHeader ("Occupancy");
        }
        
// this line should be in the end
        addEdgeHeader=false;
    }
    
}

void add_distance_features(const pcl::PointCloud<PointT> &cloud, map< int,vector<float> >&features){
    map<int,float> segment_boundary_distance;
    getSegmentDistanceToBoundary(cloud,segment_boundary_distance);
    for(map<int,float>::iterator it = segment_boundary_distance.begin(); it != segment_boundary_distance.end(); it++ )
    {
        int segid = (*it).first;
        features[segid].push_back((*it).second);
    }
}


void buildOctoMap(const pcl::PointCloud<PointT> &cloud,  OcTreeROS & tree)
{

    
    
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (cloud));
    pcl::PointCloud<PointT>::Ptr cloud_cam(new pcl::PointCloud<PointT > ());

    int cnt =0;
    // find all the camera indices
    map<int,int> camera_indices;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        camera_indices[(int) cloud.points[i].cameraIndex] = 1;
    }
    // for every camera index .. apply filter and get the point cloud
    for (map<int,int>::iterator it = camera_indices.begin(); it != camera_indices.end();it++)
    {
        int ci = (*it).first;
        apply_camera_filter(*cloud_ptr,*cloud_cam,ci);


        // convert to  pointXYZ format
        sensor_msgs::PointCloud2 cloud_blob;
        pcl::toROSMsg(*cloud_cam,cloud_blob);
        pcl::PointCloud<pcl::PointXYZ> xyzcloud;
        pcl::fromROSMsg(cloud_blob, xyzcloud);
        // find the camera co-ordinate
        VectorG cam_coordinates = originalFrames[ci]->getCameraTrans().getOrigin();
        pcl::PointXYZ origin (cam_coordinates.v[0], cam_coordinates.v[1], cam_coordinates.v[2]);
        // insert to the tree
        tree.insertScan(xyzcloud,origin,-1,true);
    }
}

int main(int argc, char** argv) {
  bool SHOW_CAM_POS_IN_VIEWER=false;
    int scene_num = atoi(argv[2]);
    std::string unTransformedPCD=argv[3];
    std::string rgbdslamBag=argv[4];
    sensor_msgs::PointCloud2 cloud_blob;
    pcl::PointCloud<PointT> cloud;
    std::ofstream labelfile, nfeatfile, efeatfile;

    //labelfile.open("data_labels.txt",ios::app);
    nfeatfile.open("data_nodefeats.txt",ios::app);
    efeatfile.open("data_edgefeats.txt",ios::app);

    // read the pcd file

    if (pcl::io::loadPCDFile(argv[1], cloud_blob) == -1) {
        ROS_ERROR("Couldn't read file test_pcd.pcd");
        return (-1);
    }
    ROS_INFO("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int) (cloud_blob.width * cloud_blob.height), pcl::getFieldsList(cloud_blob).c_str());

    
    // convert to templated message type

    pcl::fromROSMsg(cloud_blob, cloud);

    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT > (cloud));
    
          cout<<cloud.size ()<<endl;
    gatherOriginalFrames (unTransformedPCD,rgbdslamBag,originalFrames,cloudUntransformed);
    TransformG globalTransform;
    computeGlobalTransform (cloud,cloudUntransformed,globalTransform);

    for(unsigned int i=0;i<originalFrames.size ();i++)
      originalFrames[i]->applyPostGlobalTrans (globalTransform);
    
    // call buildOctoMap here
        OcTreeROS tree(0.01);
    if(UseVolFeats)
    {
        OcTreeROS::NodeType* treeNode;
        buildOctoMap(cloud,  tree);  
    }
    
    if(SHOW_CAM_POS_IN_VIEWER)
      {
      ColorHandlerPtr color_handler;
  pcl_visualization::PCLVisualizer viewer ("3D Viewer");
          color_handler.reset (new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob));
          viewer.addPointCloud (*cloud_ptr, color_handler, "cloud");
  
    for(unsigned int i=0;i<originalFrames.size ();i++)
      {
        if(originalFrames[i]->isCameraTransSet ())
          {
            VectorG cam=originalFrames[i]->getCameraTrans ().getOrigin ();
                viewer.addCoordinateSystem (1,cam.v[0],cam.v[1],cam.v[2]);
          }
      }
          
          viewer.spin ();
      }
    assert(cloudUntransformed.size()==cloud.size ());

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT > ());
    pcl::PointCloud<PointT>::Ptr cloud_seg(new pcl::PointCloud<PointT > ());
    //pcl::PointCloud<PointXYZI>::Ptr cloud_seg (new pcl::PointCloud<PointXYZI> ());
    std::vector<pcl::PointCloud<PointT> > segment_clouds;
    std::map<int,int>  segment_num_index_map;
    pcl::PointIndices::Ptr segment_indices(new pcl::PointIndices());

    // get segments

    // find the max segment number
    int max_segment_num = 0;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        if (max_segment_num < cloud.points[i].segment) {
            max_segment_num = (int) cloud.points[i].segment;
        }
    }

    ExtractIndices<PointT> extract;

    int index_ = 0;
    vector<SpectralProfile> spectralProfiles;
    for (int seg = 1; seg <= max_segment_num; seg++) {
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
        apply_segment_filter_and_compute_HOG (*cloud_ptr,*cloud_seg,seg,temp);
        
        //if (label!=0) cout << "segment: "<< seg << " label: " << label << " size: " << cloud_seg->points.size() << endl;
        if (!cloud_seg->points.empty () && cloud_seg->points.size() > 10  && cloud_seg->points[1].label != 0) {
         //std::cout << seg << ". Cloud size after extracting : " << cloud_seg->points.size() << std::endl;
			segment_clouds.push_back(*cloud_seg);
                        pcl::PointCloud<PointT>::Ptr tempPtr(new pcl::PointCloud<PointT > (segment_clouds[segment_clouds.size()-1]));
                        temp.cloudPtr=tempPtr;
 
                        spectralProfiles.push_back (temp);
			segment_num_index_map[cloud_seg->points[1].segment] = index_;
			index_ ++; 
        }
    }
    map< pair <int,int> , float > distance_matrix;
    map <int , vector <int> > neighbor_map;
    get_neighbors ( segment_clouds, distance_matrix, neighbor_map );


    // for each segment compute node featuers
    int num_bin_shape = 3;
    map < int , vector<float> > features;
    for (size_t i = 0; i< segment_clouds.size(); i++)
    {
     //   vector<float> features;
        int seg_id = segment_clouds[i].points[1].segment;
        // get color features
        //cout << "computing color features" << std::endl;
        get_color_features(segment_clouds[i], features[seg_id],spectralProfiles[i]);

        // get shape features - now in global features
     //   get_shape_features(segment_clouds[i], features[seg_id], num_bin_shape);

        // get bounding box and centroid point features
        get_global_features(segment_clouds[i], features[seg_id],spectralProfiles[i]);
          addNodeHeader=false;

    }
    add_distance_features(cloud,features);nodeFeatNames.push_back ("distance_from_wall0");
  //  vector<pcl::Normal> cloud_normals;
   // get_avg_normals(segment_clouds,cloud_normals);
    // print the node features
 //   assert(nodeFeatNames.size ()<100); // some error in setting flag can cause trouble
    for(size_t i=0;i<nodeFeatNames.size ();i++)
      nfeatfile<<"#"<<nodeFeatNames[i]<<endl;
    
    for (map< int, vector<float> >::iterator it = features.begin(); it != features.end(); it++ ){
        assert(nodeFeatNames.size ()==(*it).second.size ());
        //cerr << (*it).first << ":\t";
        nfeatfile <<  scene_num << "\t" << (*it).first << "\t" << segment_clouds[segment_num_index_map[(*it).first]].points[1].label << "\t";
        for (vector<float>::iterator it2 = (*it).second.begin(); it2 != (*it).second.end(); it2++) {
           //cerr << *it2 << "\t";
           nfeatfile <<  *it2 << "\t";
        }
        //cerr << endl;
        nfeatfile << "\n";
    }
    cerr << "\n";
    map < int, vector <float> > edge_features;
    // find pairwise features
   // assert(edgeFeatNames.size ()<100); // some error in setting flag can cause trouble
    efeatfile<<"#"<<NUM_ASSOCIATIVE_FEATS<<endl;
    int edgecount=0;
    for ( map< int, vector<int> >::iterator it=neighbor_map.begin() ; it != neighbor_map.end(); it++) {

        edge_features.clear();
        get_pair_features((*it).first, (*it).second, distance_matrix, segment_num_index_map , spectralProfiles, edge_features,tree);
        edgecount++;
        if(edgecount==1)
          {
                for(size_t i=0;i<edgeFeatNames.size ();i++)
                      efeatfile<<"#"<<edgeFeatNames[i]<<endl;

          }
        // print pair-wise features
        for (map< int, vector<float> >::iterator it2 = edge_features.begin(); it2 != edge_features.end(); it2++) {
          //  cerr << "edge: ("<< (*it).first << "," << (*it2).first << "):\t";
            efeatfile << scene_num << "\t" << (*it).first << "\t" << (*it2).first << "\t" << segment_clouds[segment_num_index_map[(*it).first]].points[1].label << "\t" << segment_clouds[segment_num_index_map[(*it2).first]].points[1].label ;
                    assert(edgeFeatNames.size ()==(*it2).second.size ());

            for (vector<float>::iterator it3 = (*it2).second.begin(); it3 != (*it2).second.end(); it3++) {
            //    cerr << *it3 << "\t";
                efeatfile << "\t" <<*it3 ;
            }
            //cerr << endl;
            efeatfile << endl;
        }

    }
  
    
    cout << "DONE!!\n";
    // write features to file
/*    for (vector<float>::iterator it = features.begin(); it < features.end(); it++) {
      outfile << *it << "\t";
    }
    outfile << "\n";
 */
    //labelfile.close();
    nfeatfile.close();
    efeatfile.close();



}


