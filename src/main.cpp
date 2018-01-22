#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>


#include <fstream>

#include <string>
#include <vector>




#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;

void
poseLogGet(string fileIn, vector<vector<float> >& Vec_Dti)
{
    vector<float> temp_line;


    ifstream fileinput;
    fileinput.open(fileIn.data());
    int i=0;
    float tmp;

    while(fileinput>>tmp)
    {
        temp_line.push_back(tmp);
        i++;
        if (i ==6){
            Vec_Dti.push_back (temp_line);
            i=0;
            temp_line.clear();
        }

    }

   

    fileinput.close();


}




void
filterRMOutlier (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered)
{

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.01);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;


}



void
filterSmoothing (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PointCloud<pcl::PointNormal>::Ptr& mls_points)
{


  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);


   pcl::PointCloud<pcl::PointNormal> mls_points_;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.1);

  // Reconstruct
  mls.process (mls_points_);
  *mls_points=mls_points_;

}

float*
generateColorMap(const int numColor)
{
  if (numColor>100){
    std::cerr<<"numColor should less than 100"<<std::endl;
      exit(0);
  }

  float *colorMap = new float[numColor]; 
  int step = floor(255/numColor*2);

  uint8_t r(255), g(0), b(0);
  for (int i = 0;i<numColor;i++)
  {
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

    float rgbF = *reinterpret_cast<float*>(&rgb);
    colorMap[i] = rgbF;

    if (i<(numColor/2)){
      r -= step;
      g += step;
    }
    else{
      r = 0 ;
      g -= step;
      b += step;
    }
    

  }
      return (colorMap);
}

void
shapesVis (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,pcl::PointCloud<pcl::PointNormal>::ConstPtr mls_points)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------

  viewer->setBackgroundColor (0, 0, 0);
  
  viewer->addPointCloud<pcl::PointNormal> (mls_points, "map cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map cloud");
//   viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

}

void
shapesVis (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr mls_points)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------

  viewer->setBackgroundColor (0, 0, 0);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(mls_points);

  viewer->addPointCloud<pcl::PointXYZRGB> (mls_points,rgb, "map cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map cloud");
//   viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

}

void
plotPath ( boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, vector<vector<float> >& Vec_Dti)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());

  

    pcl::PointXYZ  startPoint , endPoint;


  for (int i=0;i<Vec_Dti.size()-1; i++)
  { 
      startPoint.x =  Vec_Dti[i][3];
      startPoint.y =  Vec_Dti[i][4];
      startPoint.z =  Vec_Dti[i][5];

      endPoint.x =  Vec_Dti[i+1][3];
      endPoint.y =  Vec_Dti[i+1][4];
      endPoint.z =  Vec_Dti[i+1][5];

    std::stringstream filename;
    filename << "line"<<i;
    viewer->addLine<pcl::PointXYZ> (startPoint, endPoint,255,0,0, filename.str());


    cloud->push_back(startPoint);

  }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloud, 255, 0, 0);
    
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "path cloud");

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "path cloud");
}


void
mapPlot (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,pcl::PointCloud<pcl::PointNormal>::Ptr& mls_points,vector<vector<float> >& Vec_Dti)
{

    
    shapesVis (viewer,mls_points);
    plotPath(viewer,Vec_Dti);
    


}

void
mapPlot (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& mls_points,vector<vector<float> >& Vec_Dti)
{

    
    shapesVis (viewer,mls_points);
    plotPath(viewer,Vec_Dti);
    


}



int
main(int argc, char* argv[])
{
  
  vector<vector<float> > Vec_Dti;
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points_RGB(new pcl::PointCloud<pcl::PointXYZRGB> ());
  
  int layerNum = 50;

  float* colorMap;
  colorMap = generateColorMap(layerNum);
  




  if (argc ==4){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    // Replace the path below with the path where you saved your file
    pcl::io::loadPCDFile (argv[1], *cloud);


    poseLogGet(argv[2], Vec_Dti);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ> ());
    filterRMOutlier ( cloud,cloud_filtered);
    filterSmoothing(cloud_filtered,mls_points);

    int numPoints = mls_points->points.size();  
      
      float max = mls_points->points[0].y;
      float min = mls_points->points[0].y;
      for(int i = 1; i < numPoints; ++i) 
      {
        if(max < mls_points->points[i].y)
        {
        max = mls_points->points[i].y;
        }
        else
        {
        if(min > mls_points->points[i].y)
        {
          min = mls_points->points[i].y;
        }
        }
      }
    
    float layerStep = (max-min)/layerNum;
    
    for(int i = 0; i < numPoints; ++i) {
      
      int layerID = floor ((mls_points->points[i].y-min)/layerStep);
      if (layerID >=0 && layerID < layerNum){
          
          pcl::PointXYZRGB  points;

          points.x = mls_points->points[i].x;
          points.y = mls_points->points[i].y;
          points.z = mls_points->points[i].z;
          points.rgb = colorMap[layerID];
          mls_points_RGB->push_back(points);

      }

    }



    std::stringstream filename;
    filename<<argv[3]<<"/filterdPointCloud.pcd";
    pcl::io::savePCDFile (filename.str(), *mls_points_RGB);

  }
    
  else{


 
    // Replace the path below with the path where you saved your file
    pcl::io::loadPCDFile (argv[1], *mls_points_RGB);

    poseLogGet(argv[2], Vec_Dti);

  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Map"));

   mapPlot(viewer,mls_points_RGB,Vec_Dti);

    while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}



