// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>
#include <vector>
#include"kdtree.h"



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    auto startTime = std::chrono::steady_clock::now();
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    std::vector<int> indices;

    pcl::CropBox<PointT> nearby(true);
    nearby.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    nearby.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1));
    nearby.setInputCloud(cloud_region);
    region.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int j:indices)
        inliers->indices.push_back(j);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr  planeCloud (new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;
    for (int i: inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[i]);

    }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	while(maxIterations--)
	{

		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand() % (cloud->points.size()));
		float x1,x2,y1,y2,z1,z2,z3,x3,y3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		std::vector<float> v1= {x2-x1,y2-y1,z2-z1};
		std::vector<float> v2 = {x3-x1,y3-y1,z3-z1};

		std::vector<float> mul = {(v1[1]*v2[2]-v1[2]*v2[1]),(v1[2]*v2[0]-v1[0]*v2[2]),(v1[0]*v2[1]-v1[1]*v2[0])};

		float a = mul[0];
		float b = mul[1];
		float c = mul[2];
		float d = (-1)*(a*x1 + b*y1 + c*z1 );




		for(int index=0;index< cloud->points.size();index++)
		{
			if(inliers.count(index)>0)
				continue;
			PointT point = cloud->points[index];
			float x = point.x;
			float y = point.y;
			float z = point.z;

			float dist = fabs(a*x+b*y+c*z+d)/sqrt(a*a + b*b + c*c);

			if(dist <= distanceThreshold)
				inliers.insert(index);

		}
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}


	}
    std::unordered_set<int> inliers = inliersResult;
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr  planeCloud (new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstCloud->points.push_back(point);
	}


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    segResult.first = obstCloud;
    segResult.second = planeCloud;
    return segResult;
}

template<typename PointT>
void clusterHelper(int indice,typename pcl::PointCloud<PointT>::Ptr& cloud,std::vector<int>& cluster,std::vector<bool>& processed,KdTree<PointT>*& tree,float& distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(cloud->points[indice],distanceTol);

	for(int id:nearest)
	{
		if(!processed[id])
			clusterHelper(id,cloud,cluster,processed,tree,distanceTol);
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr& cloud,  KdTree<PointT>*& tree, float distanceTol,int minsize,int maxsize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    //std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(cloud->size(),false);
	int i=0;
	while(i < cloud->size())
    {
		if(processed[i])
        {
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(i,cloud,cluster,processed,tree,distanceTol);
        typename pcl::PointCloud<PointT>::Ptr newclus(new typename pcl::PointCloud<PointT>());
        if((cluster.size() >= minsize) && (cluster.size()<=maxsize))
        {
            for(int i:cluster)
            {
                newclus->points.push_back((cloud->points[i]));
            }
            newclus->width = newclus->points.size();
            newclus->height = 1;
            clusters.push_back(newclus);
        }
		i++;

	}

	return clusters;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,int minsize,int maxsize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree<PointT>* tree = new KdTree<PointT>;

    for (int i=0; i<cloud->size(); i++)
    	tree->insert(cloud->points[i],i);

    clusters = euclideanCluster(cloud,tree,clusterTolerance,minsize,maxsize);

    /*std::vector<pcl::PointIndices> cluster_indices;

    for(pcl::PointIndices getIndices:cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>());
        for(int index:getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height =1;
        cloudCluster->is_dense =true;

        clusters.push_back(cloudCluster);
    }*/
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
