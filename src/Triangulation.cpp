#include "Triangulation.h"


Triangulation::Triangulation(void)
{
}


Triangulation::~Triangulation(void)
{
}

PointCloud<Normal>::Ptr Triangulation::EstimateSurfaceNormal(PointCloud<PointXYZRGB>::Ptr cloud)
{
	PointCloud<Normal>::Ptr point_normal(new PointCloud<Normal>());
	NormalEstimation<PointXYZRGB, Normal> ne;
	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>());

	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setRadiusSearch(0.04);

	ne.compute(*point_normal);

	return point_normal;
}

PointCloud<Normal>::Ptr Triangulation::EstimateSurfaceNormalUsingEquations(PointCloud<PointXYZRGB>::Ptr cloud, bool is_plane_list_ready)
{
	vector<ModelCoefficients> list;
	vector<size_t> map;
	if(is_plane_list_ready)
		list = PlaneSegmentation::coefficient_list;
	else
	{
		PointIndices::Ptr old;
		vector<PointIndices> plane_list;
		PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>());
		copyPointCloud(*cloud, *temp_cloud);
		PlaneSegmentation::GetAllPlanes(temp_cloud, old, 0.01, map, plane_list, list);
	}

	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());

	Normal normal;
	size_t index_plane;
	size_t size_cloud = cloud->size();

	for(size_t i = 0; i < size_cloud; ++i)
	{
		if(is_plane_list_ready)
			index_plane = PlaneSegmentation::plane_map[i];
		else
			index_plane = map[i];

		normal.normal_x = list[index_plane].values[0];
		normal.normal_y = list[index_plane].values[1];
		normal.normal_z = list[index_plane].values[2];


		normals->push_back(normal);
	}

	return normals;
}

PolygonMesh::Ptr Triangulation::Triangulate(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals)
{
	//Match normals with cloud
	PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals(new PointCloud<PointXYZRGBNormal>());
	concatenateFields(*cloud, *normals, *cloud_with_normals);

	//Create search tree
	search::KdTree<PointXYZRGBNormal>::Ptr tree(new search::KdTree<PointXYZRGBNormal>());
	tree->setInputCloud(cloud_with_normals);

	//Triangulation object
	GreedyProjectionTriangulation<PointXYZRGBNormal> gp3;
	PolygonMesh::Ptr triangles(new PolygonMesh());

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	 // Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree);
	gp3.reconstruct(*triangles);

	return triangles;
}

PolygonMesh::Ptr Triangulation::FullTriangulate(PointCloud<PointXYZRGB>::Ptr cloud)
{
	PointCloud<Normal>::Ptr normals;
	normals = EstimateSurfaceNormal(cloud);
	PolygonMesh::Ptr mesh = Triangulate(cloud, normals);
	return mesh;
}