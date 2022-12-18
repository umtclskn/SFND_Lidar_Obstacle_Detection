/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool render_scene = true;
    std::vector<Car> cars = initHighway(render_scene, viewer);
    
    // Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr points = lidar->scan();

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> point_processor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_cloud = point_processor.RansacPlane(points,100,0.2);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters = point_processor.Clustering(segmented_cloud.first,1.0,3,30);
    renderPointCloud(viewer,segmented_cloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmented_cloud.second,"planeCloud",Color(0,1,0));

    int cluster_id=0;
    std::vector<Color> colors={Color(1,0,0),Color(0,1,0),Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloud_clusters)
    {
        point_processor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(cluster_id),colors[cluster_id]);
        Box box = point_processor.BoundingBox(cluster);
        renderBox(viewer,box,cluster_id);
        
        ++cluster_id;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,ProcessPointClouds<pcl::PointXYZI>* point_processor, const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud= point_processor->FilterCloud(input_cloud,0.3,Eigen::Vector4f (-10,-6,-2,1),Eigen::Vector4f (30,6.3,1,1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_cloud = point_processor->RansacPlane(filter_cloud,150,0.25);
    
    KdTree* tree = new KdTree;
    for(int i=0;i<segmented_cloud.first->points.size();i++)
        tree->insert(segmented_cloud.first->points[i],i);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters = point_processor->euclideanCluster(segmented_cloud.first, tree, 0.5, 7, 250);
    renderPointCloud(viewer,segmented_cloud.first,"ObstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmented_cloud.second,"PlaneCloud",Color(0,1,0));   

    int cluster_id=0;
    std::vector<Color> colors={Color(1,0,1),Color(0,1,1),Color(1,1,0)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloud_clusters)
    {
        point_processor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(cluster_id),colors[cluster_id]);
        Box box=point_processor->BoundingBox(cluster);
        renderBox(viewer,box,cluster_id);
        ++cluster_id;
    }

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
	ProcessPointClouds<pcl::PointXYZI>* point_processor;
	std::vector<boost::filesystem::path> stream = point_processor->streamPcd("../src/sensors/data/pcd/data_1");
	auto streamIterator = stream.begin();
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
    //simpleHighway(viewer);
	//cityBlock(viewer);
    while (!viewer->wasStopped ())
    {
      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      input_cloud = point_processor->loadPcd((*streamIterator).string());
      cityBlock(viewer, point_processor, input_cloud);

      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();

      viewer->spinOnce ();
    }
}