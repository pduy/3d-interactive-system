#pragma once

#include "ObjectInsertion.h"
#include "ClusteringBasedSegmentation.h"
#include "Triangulation.h"
#include "MainProgram.h"

#include<iostream>
#include<pcl/segmentation/sac_segmentation.h>
#include<Eigen/src/Core/Matrix.h>
#include<time.h>
#include<pcl/common/common.h>

using namespace std;
using namespace pcl;
using namespace Eigen;

class PlaneSegmentation
{
public:
	PlaneSegmentation(void);
	~PlaneSegmentation(void);

	static void GetAllPlanes(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices::Ptr old, float distance_threshold, vector<size_t> &plane_map, static vector<PointIndices> &plane_index_list, vector<ModelCoefficients> &coefficient_list);
	static void GetAllPlanesByNormal(PointCloud<PointXYZ>::Ptr cloud, PointCloud<Normal>::Ptr point_normal, PointIndices::Ptr old, float distance_threshold, vector<size_t> &plane_map, static vector<PointIndices> &plane_index_list, vector<ModelCoefficients> &coefficient_list);
	static void DetermineGroundPlanes(PointCloud<PointXYZRGB>::Ptr cloud);
	static vector<PointIndices> plane_index_list;
	static vector<ModelCoefficients> coefficient_list;
	static vector<size_t> plane_map;
	static vector<int> ground_plane_list;
};

