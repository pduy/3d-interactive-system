#pragma once
#include "ClusteringBasedSegmentation.h"
#include "PlaneSegmentation.h"
#include "Triangulation.h"
#include "MainProgram.h"

#include<Eigen\src\Core\Matrix.h>

using namespace Eigen;
using namespace pcl;

class ObjectInsertion
{
public:
	ObjectInsertion(void);
	~ObjectInsertion(void);

	static void LoadObjectList(vector<string> file_list);
	static void TriangulateObjects();
	static void TransformPointCloud(PointCloud<PointXYZRGB>::Ptr cloud, Vector3f &normal_object, Vector3f normal_dest, Vector3f point, Vector3f source_point);
	static void TransformPointCloud(PolygonMesh::Ptr mesh, Vector3f &normal_object, Vector3f normal_dest, PointXYZRGB point, PointXYZRGB source_point);
	static void RotatePointCloud(PointCloud<PointXYZRGB>::Ptr cloud, Vector3f &normal_object, float angle, Vector3f point);

	static vector<PointCloud<PointXYZRGB>::Ptr> object_library;
	static vector<PolygonMesh::Ptr> object_meshes;
	static vector<PointCloud<Normal>::Ptr> object_normal_list;
	static vector<Vector3f> object_normals;
	static vector<Vector3f> object_standings;
	static vector<int> n_transformations;
};

