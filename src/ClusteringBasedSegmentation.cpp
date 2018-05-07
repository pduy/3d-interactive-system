#include "ClusteringBasedSegmentation.h"

int ClusteringBasedSegmentation::k;
int ClusteringBasedSegmentation::n_planes;
pcl::KdTreeFLANN<PointXYZRGB> ClusteringBasedSegmentation::tree;
vector<vector<int>> ClusteringBasedSegmentation::neighbor_map;
vector<bool> ClusteringBasedSegmentation::is_in_neighbor_list;

ClusteringBasedSegmentation::ClusteringBasedSegmentation(void)
{
}

ClusteringBasedSegmentation::~ClusteringBasedSegmentation(void)
{
}

PointCloud<PointXYZRGB>::Ptr ClusteringBasedSegmentation::RemoveBackgroundPlanes(PointCloud<PointXYZRGB> cloud, vector<PointIndices> plane_list, PointIndices::Ptr &old_indices)
{
	old_indices.reset(new PointIndices());
	old_indices->indices.resize(cloud.size());
	PointCloud<PointXYZRGB>::Ptr cloud_new(new PointCloud<PointXYZRGB>());
	size_t new_index = 0;

	for(size_t i = 0; i < n_planes; ++i)
	{
		for(size_t j = 0; j < plane_list[i].indices.size(); ++j)
		{
			cloud.points[plane_list[i].indices[j]].x = 
				cloud.points[plane_list[i].indices[j]].y =
				cloud.points[plane_list[i].indices[j]].z = 0;
		}
	}

	for(size_t i = 0; i < cloud.size(); ++i)
	{
		if(cloud.points[i].x != 0)
		{
			cloud_new->points.push_back(cloud.points[i]);
			old_indices->indices[i] = new_index;
			++new_index;
		}
	}

	return cloud_new;
}

void ClusteringBasedSegmentation::SegmentUsingKMeans(PointCloud<PointXYZRGB>::Ptr cloud, int nloops, vector<PointIndices> &list_subcloud)
{
	//Choosing k points
	size_t size_cloud = cloud->size();
	float distance_squared_temp;
	float distance_squared_temp_next_point;
	size_t cluster_index;
	size_t size_subcloud = (size_t)floor((float)size_cloud / k);
	PointIndices temp_indices;

	//Set the initial subclouds
	for(size_t i = 0; i < size_cloud; ++i)
	{
		temp_indices.indices.push_back(i);

		if(i != 0 && i % size_subcloud == 0 || i == size_cloud - 1)
		{
			list_subcloud.push_back(temp_indices);
			temp_indices.indices.clear();
		}
	}

	//Assign each point into a cluster
	for(size_t u = 0; u < nloops; ++u)
	{
		vector<Vector4f> list_centroids;
		// Get each subcloud's centroid	
		for(size_t i = 0; i < k ; ++i)
		{
			Vector4f centroid;
			compute3DCentroid(*cloud, list_subcloud[i].indices,centroid);
			list_centroids.push_back(centroid);
			list_subcloud[i].indices.clear();
		}

		for(size_t i = 0; i < size_cloud; ++i)
		{
			cluster_index = 0;
			
			distance_squared_temp = (cloud->points[i].x - list_centroids[0].x()) * (cloud->points[i].x - list_centroids[0].x())
								+	(cloud->points[i].y - list_centroids[0].y()) * (cloud->points[i].y - list_centroids[0].y())
								+	(cloud->points[i].z - list_centroids[0].z()) * (cloud->points[i].z - list_centroids[0].z());

			for(size_t j = 1; j < k; ++j)
			{
				distance_squared_temp_next_point = 
									(cloud->points[i].x - list_centroids[j].x()) * (cloud->points[i].x - list_centroids[j].x())
								+	(cloud->points[i].y - list_centroids[j].y()) * (cloud->points[i].y - list_centroids[j].y())
								+	(cloud->points[i].z - list_centroids[j].z()) * (cloud->points[i].z - list_centroids[j].z());

				if(distance_squared_temp > distance_squared_temp_next_point)
				{
					distance_squared_temp = distance_squared_temp_next_point;
					cluster_index = j;
				}
			}

			list_subcloud[cluster_index].indices.push_back(i);
		}	
	}

}

void ClusteringBasedSegmentation::FindNeighbours(PointCloud<PointXYZRGB>::Ptr cloud, size_t index_point, float epsilon, PointIndices &eps_neighborhood)
{
	for(size_t i = 0; i < neighbor_map[index_point].size(); ++i)
	{
		if(is_in_neighbor_list[neighbor_map[index_point][i]] == false)
		{
			is_in_neighbor_list[neighbor_map[index_point][i]] = true;
			eps_neighborhood.indices.push_back(neighbor_map[index_point][i]);
		}
	}
}

void ClusteringBasedSegmentation::CreateCluster(PointCloud<PointXYZRGB>::Ptr cloud,vector<bool> &status_point, size_t index_point, PointIndices &eps_neighborhood, float epsilon, size_t min_points)
{
	eps_neighborhood.indices.push_back(index_point);

	size_t neighborhood_size = eps_neighborhood.indices.size();
	size_t i = 0;
	while(i < neighborhood_size)
	{
		if(status_point[eps_neighborhood.indices[i]] == false)
		{
			status_point[eps_neighborhood.indices[i]] = true;
			PointIndices sub_neighborhood;
			FindNeighbours(cloud, eps_neighborhood.indices[i], epsilon, sub_neighborhood);
			eps_neighborhood.indices.insert(eps_neighborhood.indices.end(), sub_neighborhood.indices.begin(), sub_neighborhood.indices.end());
		}
		++i;
		neighborhood_size = eps_neighborhood.indices.size();
	}
}
/*
void ClusteringBasedSegmentation::SegmentUsingDBSCAN(PointCloud<PointXYZRGB>::Ptr cloud, float epsilon, size_t min_points, vector<PointIndices> &list_subcloud)
{
	PointIndices temp_indices;
	PointIndices eps_neighborhood;
	vector<bool> status_point;
	size_t cluster_index = 0;

	status_point.resize(cloud->size());

	// All points are not visited

	for(size_t i = 0; i < cloud->size(); ++i)
	{
		if(status_point[i] == false)
		{
			status_point[i] = true;
			eps_neighborhood.indices.clear();
			FindNeighbours(cloud, i, epsilon, eps_neighborhood);
			if(eps_neighborhood.indices.size() >= min_points)
			{
				PointIndices::Ptr indices_list(new PointIndices());
				list_subcloud.push_back(*indices_list);
				CreateCluster(cloud, status_point, i, eps_neighborhood, cluster_index, list_subcloud, epsilon, min_points);
				++cluster_index;
			}
		}
	}
}
*/

void ClusteringBasedSegmentation::SegmentUsingModifiedDBSCAN(PointCloud<PointXYZRGB>::Ptr cloud, size_t index_point, float epsilon, size_t min_points, PointIndices &list_subcloud)
{
	tree.setInputCloud(cloud);
	vector<bool> status_point;

	status_point.resize(cloud->size());
	is_in_neighbor_list.resize(cloud->size());

	//Construct neighbor map using KdTree
	for(size_t i = 0; i < cloud->size(); ++i)
	{
		vector<float> sqrt_distance;
		vector<int> neighbors;
		tree.radiusSearch(i, epsilon, neighbors, sqrt_distance);
		neighbor_map.push_back(neighbors);
	}

	// All points are not visited

	if(status_point[index_point] == false)
	{
		status_point[index_point] = true;
		list_subcloud.indices.clear();
		FindNeighbours(cloud, index_point, epsilon, list_subcloud);
		CreateCluster(cloud, status_point, index_point, list_subcloud, epsilon, min_points);
	}
}
