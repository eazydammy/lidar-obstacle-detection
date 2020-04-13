// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"
#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}


	void insertHelper(Node** node, uint depth, pcl::PointXYZI point, int id)
	{
		//Tree is empty
		if(*node==NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			
			uint cd = depth % 3;

			switch (cd)
			{
			case 0:
				{
					if (point.x < (*node)->point.x) {
						insertHelper(&((*node)->left), depth + 1, point, id);
					} else {
						insertHelper(&((*node)->right), depth + 1, point, id);
					}
            	} break;

			case 1:
				{
					if (point.y < (*node)->point.y) {
						insertHelper(&((*node)->left), depth + 1, point, id);
					} else {
						insertHelper(&((*node)->right), depth + 1, point, id);
					}
            	} break;

			case 2:
				{
					if (point.z < (*node)->point.z) {
						insertHelper(&((*node)->left), depth + 1, point, id);
					} else {
						insertHelper(&((*node)->right), depth + 1, point, id);
					}
            	} break;
			}
		}	
	}
  
	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id);

	}

	void searchHelper(pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL) {
            float delta_x = node->point.x - target.x;
            float delta_y = node->point.y - target.y;
            float delta_z = node->point.z - target.z;

            if ((delta_x >= -distanceTol && delta_x <= distanceTol) &&
                (delta_y >= -distanceTol && delta_y <= distanceTol) &&
                (delta_z >= -distanceTol && delta_z <= distanceTol)) {
                float distance = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
                if (distance <= distanceTol) {
                    ids.push_back(node->id);
                }
            }

            // check across boundary
			uint cd = depth % 3;
			switch (cd)
			{
			case 0:
				{
					if (delta_x > -distanceTol)
					{
						searchHelper(target, node->left, depth + 1, distanceTol, ids);
					}
                  
					if (delta_x < distanceTol)
					{
						searchHelper(target, node->right, depth + 1, distanceTol, ids);
					}	
				} break;
			
			case 1:
				{
					if (delta_y > -distanceTol)
					{
						searchHelper(target, node->left, depth + 1, distanceTol, ids);
					}
                  
					if (delta_y < distanceTol)
					{
						searchHelper(target, node->right, depth + 1, distanceTol, ids);
					}	
				} break;

			case 2:
				{
					if (delta_z > -distanceTol)
					{
						searchHelper(target, node->left, depth + 1, distanceTol, ids);
					}
                  
					if (delta_z < distanceTol)
					{
						searchHelper(target, node->right, depth + 1, distanceTol, ids);
					}	
				} break;
			}
        }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	

};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CustomRANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    void clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol);
    
	std::vector<typename pcl::PointCloud<PointT>::Ptr> customClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize);
    
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */