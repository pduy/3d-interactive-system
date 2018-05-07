#pragma once
#include "ClusteringBasedSegmentation.h"
#include "GraphBasedSegmentation.h"
#include "PlaneSegmentation.h"
#include "Triangulation.h"
#include "MainProgram.h"
#include "SupportingFunctions.h"

using namespace Eigen;
using namespace pcl;

static class ObjectRemoval
{
public:
	ObjectRemoval(void);
	~ObjectRemoval(void);

	static void DetectObjectPlanes(PointCloud<PointXYZRGB>::Ptr cloud, vector<PointIndices> & plane_index_list, vector<size_t> & plane_map, PointIndices &object, vector<size_t> &object_plane_list);
	static void DetectObjectGroundPlanes(PointCloud<PointXYZRGB>::Ptr cloud, vector<PointIndices> & plane_index_list, vector<size_t> & plane_map, vector<ModelCoefficients> &coefficient_list, PointIndices &object, vector<size_t> &ground_plane_list);
	static void RemoveObject(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices object_index);
	static void RemoveObject(PointCloud<PointXYZRGB>::Ptr cloud, vector<ModelCoefficients> &coefficient_list, vector<PointIndices> & plane_index_list, PointIndices &object_index, vector<size_t> &object_plane_list, vector<size_t> &ground_plane_list, vector<PointIndices> &restored_indices, vector<PointCloud<PointXYZRGB>::Ptr> &restored_points);
	static void FuseColorToRestoredSurface(PointCloud<PointXYZRGB>::Ptr cloud, vector<PointIndices> &restored_indices, vector<PointCloud<PointXYZRGB>::Ptr> &restored_points);

	static float SEARCH_RADIUS_;
};

