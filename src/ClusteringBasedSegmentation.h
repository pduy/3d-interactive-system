#pragma once
#include "MainProgram.h"
#include "ObjectInsertion.h"
#include "GraphBasedSegmentation.h"
#include "PlaneSegmentation.h"
#include "Triangulation.h"

#include<iostream>
#include<Eigen\src\Core\Matrix.h>

using namespace std;
using namespace pcl;
using namespace Eigen;


class ClusteringBasedSegmentation
{
public:
	ClusteringBasedSegmentation(void);
	~ClusteringBasedSegmentation(void);

	static PointCloud<PointXYZRGB>::Ptr RemoveBackgroundPlanes(PointCloud<PointXYZRGB> cloud, vector<PointIndices> plane_list, PointIndices::Ptr &old_indices);
	static void SegmentUsingKMeans(PointCloud<PointXYZRGB>::Ptr cloud, int nloops, vector<PointIndices> &list_subcloud);
	static void FindNeighbours(PointCloud<PointXYZRGB>::Ptr cloud, size_t index_point, float epsilon, PointIndices &eps_neighborhood);
	static void CreateCluster(PointCloud<PointXYZRGB>::Ptr cloud,vector<bool> &status_point, size_t index_point, PointIndices &eps_neighborhood, float epsilon, size_t min_points);
	static void SegmentUsingDBSCAN(PointCloud<PointXYZRGB>::Ptr cloud, float epsilon, size_t min_points, vector<PointIndices> &list_subcloud);
	static void SegmentUsingModifiedDBSCAN(PointCloud<PointXYZRGB>::Ptr cloud, size_t index_point, float epsilon, size_t min_points, PointIndices &list_subcloud);

	//Number of clusters
	static int k;
	//Number of removed planes
	static int n_planes;
	static pcl::KdTreeFLANN<PointXYZRGB> tree;
	static vector<vector<int>> neighbor_map;
	static vector<bool> is_in_neighbor_list;
};

