#pragma once
#include "ObjectInsertion.h"
#include "ClusteringBasedSegmentation.h"
#include "PlaneSegmentation.h"
#include "Triangulation.h"
#include "MainProgram.h"
#include "MyMinCut.h"

#include<iostream>
#include<pcl/segmentation/min_cut_segmentation.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<Eigen\src\Core\Matrix.h>
#include<pcl\kdtree\kdtree_flann.h>

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace visualization;

class GraphBasedSegmentation
{
public:
	GraphBasedSegmentation(void);
	~GraphBasedSegmentation(void);

	static void SetForeground(PointIndices foreground);
	static void SetBackground(PointIndices background);
	static void AddForeground(PointIndices foreground);
	static void AddBackground(PointIndices background);
	static void ClearForeground();
	static void ClearBackground();
	static bool ForegroundIsEmpty();
	static bool BackgroundIsEmpty();
	static PointIndices::Ptr GetEdgeBuffer(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices &object, float size, vector<bool> &is_in_object_buffer);
	static void RefineMinCutSegmentation(PointCloud<PointXYZRGB>::Ptr cloud, vector<PointIndices> &cluster);

	static void MinCutBaseSegmentation(PointCloud<PointXYZRGB>::Ptr xyzcloud, vector<PointIndices> &clusters);
	static void SegmentUsingMyMinCut(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals, vector<PointIndices> &clusters);

	static bool foreground_updated_;
	static bool background_updated_;
protected:
	static PointIndices foreground_points_;
	static PointIndices background_points_;
	
};

