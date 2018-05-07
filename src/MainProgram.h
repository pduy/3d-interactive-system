#pragma once
#include "ObjectInsertion.h"
#include "ObjectRemoval.h"
#include "ClusteringBasedSegmentation.h"
#include "GraphBasedSegmentation.h"
#include "PlaneSegmentation.h"
#include "Triangulation.h"

#include<iostream>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/boost.h>
#include<pcl/point_types.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/features/normal_3d.h>
#include<pcl/common/distances.h>
#include<time.h>
#include<Eigen\src\Core\Matrix.h>
#include<pcl\filters\voxel_grid.h>
#include<pcl\surface\gp3.h>
#include<pcl\filters\statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include<math.h>
	
using namespace std;
using namespace pcl;
using namespace Eigen;

static class MainProgram
{
public:
	MainProgram(void);
	~MainProgram(void);

	static void Initialize();
	//General methods
	static Normal GetNormal(int plane_index);
	static int SearchForObject(size_t index_point, vector<size_t> &map);
	static int SearchForObject(size_t index_point, vector<int> &list);
	static int SearchForObject(size_t index_point, vector<PointIndices> &list);
	static void RemoveOutliers(PointCloud<PointXYZRGB>::Ptr cloud_in, PointCloud<PointXYZRGB>::Ptr &cloud_out, int meanK, float stdDev);
	static void FillColor2Cloud(PointCloud<PointXYZRGB>::Ptr cloud, vector<PointIndices> cluster_list);

	static void pp_callback (const pcl::visualization::PointPickingEvent& event, void* cookie);
	static void kb_callback (const pcl::visualization::KeyboardEvent& event, void* cookie);
	static void mouse_callback (const pcl::visualization::MouseEvent& event, void* cookie);
	static int performSinglePick (vtkRenderWindowInteractor *iren, float &x, float &y, float &z);
	static void Helper();

	//General variables
	static int mode;
	static int model_type;
	static PointIndices::Ptr picked_points;
	static visualization::PCLVisualizer viewer;
	static PolygonMesh::Ptr mesh_model;
	static PointCloud<PointXYZRGB>::Ptr model;
	static PointCloud<PointXYZRGB>::Ptr edited_model;
	static PointCloud<Normal>::Ptr normal_model;
	static vector<PointIndices> clusters;
	static vector<bool> is_in_object;
	static vector<pcl::visualization::Camera> camera;
	static int PointFilter;
	static const int POINT_FILTER_LIMITATION ;
	static int iObject;
};

