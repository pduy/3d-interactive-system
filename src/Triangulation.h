#pragma once
#include "ObjectInsertion.h"
#include "ClusteringBasedSegmentation.h"
#include "PlaneSegmentation.h"
#include "MainProgram.h"

#include<iostream>
#include<pcl\surface\gp3.h>
#include<pcl/features/normal_3d.h>
#include<Eigen\src\Core\Matrix.h>
#include<vtkDelaunay3D.h>

using namespace std;
using namespace pcl;
using namespace Eigen;

class Triangulation
{
public:
	Triangulation(void);
	~Triangulation(void);

	static PointCloud<Normal>::Ptr EstimateSurfaceNormal(PointCloud<PointXYZRGB>::Ptr cloud);
	static PointCloud<Normal>::Ptr EstimateSurfaceNormalUsingEquations(PointCloud<PointXYZRGB>::Ptr cloud, bool is_plane_list_ready);
	static PolygonMesh::Ptr Triangulate(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals);
	static PolygonMesh::Ptr FullTriangulate(PointCloud<PointXYZRGB>::Ptr cloud);
};

