#include "PlaneSegmentation.h"

vector<PointIndices> PlaneSegmentation::plane_index_list;
vector<ModelCoefficients> PlaneSegmentation::coefficient_list;
vector<size_t> PlaneSegmentation::plane_map;
vector<int> PlaneSegmentation::ground_plane_list;

PlaneSegmentation::PlaneSegmentation(void)
{
}


PlaneSegmentation::~PlaneSegmentation(void)
{
}

void PlaneSegmentation::GetAllPlanes(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices::Ptr old, float distance_threshold, vector<size_t> &plane_map, static vector<PointIndices> &plane_index_list, vector<ModelCoefficients> &coefficient_list)
{
	cout << "cloud size = " << cloud->size();
	
	// cloud size = 0 => empty list
	
	size_t size_cloud = cloud->size();

	if(size_cloud < 4)
	{
		size_t index = plane_index_list.size() - 1;
		for(size_t i = 0; i < size_cloud; ++i)
		{
			plane_index_list[index].indices.push_back(old->indices[i]);
		}
		return;
	}

//	PointCloud<PointXYZINormal>::Ptr cloud_with_normals(new PointCloud<PointXYZINormal>());
//	concatenateFields(*cloud, *normal_cloud, *cloud_with_normals);

	SACSegmentation<PointXYZRGB> seg;
	PointIndices::Ptr inliers (new PointIndices());
	ModelCoefficients::Ptr coefficients(new ModelCoefficients());

	// set parameters for segmentation object
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance_threshold);
	seg.setMaxIterations(400);
	seg.setInputCloud (cloud);
	
	// segment the model to get a plane and coefficients
	seg.segment (*inliers, *coefficients);

	cout << "		Plane size = " << inliers->indices.size() << endl;

	size_t size_inliers = inliers->indices.size();

	// remove the plane out of the cloud and do the same algorithm to the sub-cloud

	for(size_t i = 0; i < size_inliers; ++i)
	{
		int index = inliers->indices[i];
		
		cloud->points[index].x = 0;
		cloud->points[index].y = 0;
		cloud->points[index].z = 0;
	}

	PointCloud<PointXYZRGB>::Ptr cloud_remaining(new PointCloud<PointXYZRGB>());
	PointIndices::Ptr indices_old(new PointIndices());

	for(size_t i = 0; i < size_cloud; ++i)
	{
		if(cloud->points[i].x != 0 && cloud->points[i].y != 0 && cloud->points[i].z != 0)
		{
			cloud_remaining->points.push_back(cloud->points[i]);
			if(old != NULL)
				indices_old->indices.push_back(old->indices[i]);
			else
				indices_old->indices.push_back(i);
		}
	}

	//Get the real indices
	if(old != NULL)
	{
		for(size_t i = 0; i < size_inliers; ++i)
		{
			inliers->indices[i] = old->indices[inliers->indices[i]];
		}
	}

	// get the detected plane
	plane_index_list.push_back(*inliers);
	coefficient_list.push_back(*coefficients);

	// inilialize the map if not exist
	if(plane_map.empty())
		plane_map.resize(size_cloud);

	// mapping plane index to point indices
	size_t current_pos = plane_index_list.size() - 1;

	if(current_pos != 0)
		for(size_t i = 0; i < inliers->indices.size(); ++i)
		{
			plane_map[inliers->indices[i]] = current_pos;
		}

	if(cloud_remaining->size() > 0)
		GetAllPlanes(cloud_remaining, indices_old, distance_threshold, plane_map, plane_index_list, coefficient_list);
}

void PlaneSegmentation::GetAllPlanesByNormal(PointCloud<PointXYZ>::Ptr cloud, PointCloud<Normal>::Ptr point_normal, PointIndices::Ptr old, float distance_threshold, vector<size_t> &plane_map, static vector<PointIndices> &plane_index_list, vector<ModelCoefficients> &coefficient_list)
{	
	// cloud size = 0 => empty list
	
	size_t size_cloud = cloud->size();
	cout << "cloud size = " << size_cloud;

	if(size_cloud < 100)
	{
		size_t index = plane_index_list.size() - 1;
		for(size_t i = 0; i < size_cloud; ++i)
		{
			plane_index_list[index].indices.push_back(old->indices[i]);
		}
		return;
	}

	SACSegmentation<PointXYZ> seg;
	PointIndices::Ptr inliers (new PointIndices());
	ModelCoefficients::Ptr coefficients(new ModelCoefficients());

	// set parameters for segmentation object
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance_threshold);
	seg.setMaxIterations(400);
	seg.setInputCloud (cloud);
	
	// segment the model to get a plane and coefficients
	seg.segment (*inliers, *coefficients);

	size_t size_inliers = inliers->indices.size();
	PointIndices::Ptr new_inliers(new PointIndices());

	cout << "		Plane size before = " << inliers->indices.size() << endl;

	Vector3f normal_plane(coefficients->values[0], coefficients->values[1], coefficients->values[1]);
	//cout << "PlaneN: " <<normal_plane.x()  << "; " << normal_plane.x()  << "; " << normal_plane.z();
	// remove the plane out of the cloud and do the same algorithm to the sub-cloud
	for(size_t i = 0; i < size_inliers; ++i)
	{
		int index = inliers->indices[i];

		Vector3f normal_point(point_normal->points[index].normal_x, point_normal->points[index].normal_y, point_normal->points[index].normal_z);
		//cout << normal_point.x()  << "; " << normal_point.x()  << "; " << normal_point.z()  << endl;
		float cosine = normal_point.dot(normal_plane) / (normal_point.norm() * normal_plane.norm());
		//cin.get();
		if(cosine > 0.06333 || cosine < -0.06333)
		{
			cloud->points[index].x = 0;
			cloud->points[index].y = 0;
			cloud->points[index].z = 0;
			new_inliers->indices.push_back(index);
		}
	}

	cout << "		Plane size = " << new_inliers->indices.size() << endl;
	//cin.get();

	PointCloud<PointXYZ>::Ptr cloud_remaining(new PointCloud<PointXYZ>());
	PointCloud<Normal>::Ptr normal_remaining(new PointCloud<Normal>());
	PointIndices::Ptr indices_old(new PointIndices());

	for(size_t i = 0; i < size_cloud; ++i)
	{
		if(cloud->points[i].x != 0 && cloud->points[i].y != 0 && cloud->points[i].z != 0)
		{
			cloud_remaining->points.push_back(cloud->points[i]);
			normal_remaining->points.push_back(point_normal->points[i]);
			if(old != NULL)
				indices_old->indices.push_back(old->indices[i]);
			else
				indices_old->indices.push_back(i);
		}
	}

	size_inliers = new_inliers->indices.size();
	//Get the real indices
	if(old != NULL)
	{
		for(size_t i = 0; i < size_inliers; ++i)
		{
			new_inliers->indices[i] = old->indices[new_inliers->indices[i]];
		}
	}

	// get the detected plane
	plane_index_list.push_back(*new_inliers);
	coefficient_list.push_back(*coefficients);

	// inilialize the map if not exist
	if(plane_map.empty())
		plane_map.resize(size_cloud);

	// mapping plane index to point indices
	size_t current_pos = plane_index_list.size() - 1;

	if(current_pos != 0)
		for(size_t i = 0; i < size_inliers; ++i)
		{
			plane_map[new_inliers->indices[i]] = current_pos;
		}

	if(cloud_remaining->size() > 0)
		GetAllPlanesByNormal(cloud_remaining, normal_remaining, indices_old, distance_threshold, plane_map, plane_index_list, coefficient_list);
}

void PlaneSegmentation::DetermineGroundPlanes(PointCloud<PointXYZRGB>::Ptr cloud)
{
	int n_left_points, n_right_points;
	for(size_t i = 0; i < coefficient_list.size(); ++i)
	{
		float a = coefficient_list[i].values[0];
		float b = coefficient_list[i].values[1];
		float c = coefficient_list[i].values[2];
		float d = coefficient_list[i].values[3];

		n_left_points = n_right_points = 0;
		for(size_t j = 0; j < cloud->size(); ++j)
		{			
			if(a * cloud->points[j].x + b * cloud->points[j].y + c * cloud->points[j].z + d - 0.03 > 0)
				n_left_points++;
			else if(a * cloud->points[j].x + b * cloud->points[j].y + c * cloud->points[j].z + d + 0.03 < 0)
				n_right_points++;
		}

		float ratio = (float)n_left_points / (float)(n_left_points + n_right_points);
		if(ratio < 0.2 || ratio > 0.8)
		{
			ground_plane_list.push_back(i);
		}			
	}
}
