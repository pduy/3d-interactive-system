#include "ObjectRemoval.h"

float ObjectRemoval::SEARCH_RADIUS_ = 0.01;

ObjectRemoval::ObjectRemoval(void)
{
}


ObjectRemoval::~ObjectRemoval(void)
{
}

void ObjectRemoval::DetectObjectPlanes(PointCloud<PointXYZRGB>::Ptr cloud, vector<PointIndices> & plane_index_list, vector<size_t> &plane_map, PointIndices &object, vector<size_t> &object_plane_list)
{
	size_t size_object = object.indices.size();
	for(size_t i = 0; i < size_object; ++i)
	{
		int plane_index = plane_map[object.indices[i]];
		if(MainProgram::SearchForObject(plane_index, object_plane_list) == -1)
		{
			object_plane_list.push_back(plane_index);
		}
	}
}

void ObjectRemoval::DetectObjectGroundPlanes(PointCloud<PointXYZRGB>::Ptr cloud, vector<PointIndices> & plane_index_list, vector<size_t> & plane_map, vector<ModelCoefficients> &coefficient_list, PointIndices &object, vector<size_t> &ground_plane_list)
{
	/*
	PointCloud<PointXYZ>::Ptr temp_cloud (new PointCloud<PointXYZ>());
	copyPointCloud(*cloud, *temp_cloud);
	vector<bool> is_in_ground_plane_list;
	vector<size_t> number_of_object_points;
	
	size_t size_plane_list = plane_index_list.size();
	for(size_t i = 0; i < size_plane_list; ++i)
	{
		is_in_ground_plane_list.push_back(false);
		number_of_object_points.push_back(0);
	}

	float center_x = 0;
	float center_y = 0;
	float center_z = 0;

	size_t size_object = object.indices.size();

	for(size_t i = 0; i < size_object; ++i)
	{		
		center_x += cloud->points[object.indices[i]].x;
		center_y += cloud->points[object.indices[i]].y;
		center_z += cloud->points[object.indices[i]].z;
		temp_cloud->points[object.indices[i]].x = temp_cloud->points[object.indices[i]].y = temp_cloud->points[object.indices[i]].z = 0;

		//Increase number of object points of the plane containing i
		number_of_object_points[plane_map[object.indices[i]]]++;
	}

	//Get the centroid of the object
	center_x /= size_object;
	center_y /= size_object;
	center_z /= size_object;

	//Get the radius of the object
	float radius = 0;
	for(size_t i = 0; i < size_object; ++i)
	{
		float cur_distance = (cloud->points[object.indices[i]].x - center_x) * (cloud->points[object.indices[i]].x - center_x)
			+ (cloud->points[object.indices[i]].y - center_y) * (cloud->points[object.indices[i]].y - center_y)
			+ (cloud->points[object.indices[i]].z - center_z) * (cloud->points[object.indices[i]].z - center_z);

		if (cur_distance > radius)
			radius = cur_distance;
	}

	//find the points inside the radius in the whole point cloud
	size_t size_cloud = temp_cloud->size();
	for(size_t i = 0; i < size_cloud; ++i)
	{
		if(temp_cloud->points[i].x != 0 || temp_cloud->points[i].y != 0 || temp_cloud->points[i].z != 0)
		{
			float distance = (temp_cloud->points[i].x - center_x) * (temp_cloud->points[i].x - center_x)
				+ (temp_cloud->points[i].y - center_y) * (temp_cloud->points[i].y - center_y)
				+ (temp_cloud->points[i].z - center_z) * (temp_cloud->points[i].z - center_z);
			if(distance <= radius && !is_in_ground_plane_list[plane_map[i]] && ((float) number_of_object_points[plane_map[i]] / (float) plane_index_list[plane_map[i]].indices.size()) < 0.01)
			{
				ground_plane_list.push_back(plane_map[i]);
				is_in_ground_plane_list[plane_map[i]] = true;
			}
		}
	}
	*/

	KdTreeFLANN<PointXYZRGB>::Ptr tree (new KdTreeFLANN<PointXYZRGB>());
	tree->setInputCloud(cloud);

	vector<bool> is_in_object;
	vector<bool> is_in_ground_plane_list;
	vector<size_t> number_of_object_points;
	
	//Initialize the maps
	size_t size_plane_list = plane_index_list.size();
	size_t size_cloud = cloud->size();
	for(size_t i = 0; i < size_plane_list; ++i)
	{
		number_of_object_points.push_back(0);
	}
	is_in_object.resize(size_cloud);
	is_in_ground_plane_list.resize(size_plane_list);

	//Calculate number of object points in each plane
	size_t size_object = object.indices.size();
	for(size_t i = 0; i < size_object; ++i)
	{
		number_of_object_points[plane_map[object.indices[i]]]++;
		is_in_object[object.indices[i]] = true;
	}

	//find some nearest neighbors and check if they are not in the object
	for(size_t i = 0; i < size_object; ++i)
	{
		vector<int> neighbors;
		vector<float> neighbor_distance;
		float radius = 0.04;
		tree->radiusSearch(cloud->points[object.indices[i]], radius, neighbors, neighbor_distance);

		size_t size_neighbors = neighbors.size();
		for(size_t j = 0; j < size_neighbors; ++j)
		{
			size_t plane_index = plane_map[neighbors[j]];
			if(plane_index_list[plane_index].indices.size() > (0.25 * size_object) &&
				!is_in_object[neighbors[j]] && 
				!is_in_ground_plane_list[plane_index] &&
				((float) number_of_object_points[plane_index] / (float) plane_index_list[plane_index].indices.size()) < 0.5)
			{
				ground_plane_list.push_back(plane_index);
				is_in_ground_plane_list[plane_index] = true;
			}
		}
	}

	cout << endl << "Ground planes before filtering = " << ground_plane_list.size() << endl;

	//Filter the ground planes list to have the actual object ground planes
	size_t size_ground_plane = ground_plane_list.size();
	vector<size_t> temp_ground_plane_list;
	for(size_t i = 0; i < size_ground_plane; ++i)
	{
		int n_left_points = 0;
		int n_right_points = 0;
		float a = coefficient_list[ground_plane_list[i]].values[0];
		float b = coefficient_list[ground_plane_list[i]].values[1];
		float c = coefficient_list[ground_plane_list[i]].values[2];
		float d = coefficient_list[ground_plane_list[i]].values[3];

		for(size_t j = 0; j < size_object; ++j)
		{			
			if(a * cloud->points[object.indices[j]].x + b * cloud->points[object.indices[j]].y + c * cloud->points[object.indices[j]].z + d > 0)
				n_left_points++;
			else if(a * cloud->points[object.indices[j]].x + b * cloud->points[object.indices[j]].y + c * cloud->points[object.indices[j]].z + d < 0)
				n_right_points++;
		}

		float ratio = (float)n_left_points / (float)(n_left_points + n_right_points);
		cout << ratio << endl;
		if(ratio < 0.2 || ratio > 0.8)
		{
			/*
			ground_plane_list[i] = ground_plane_list.back();
			ground_plane_list.pop_back();
			size_ground_plane = ground_plane_list.size();
			*/
			temp_ground_plane_list.push_back(ground_plane_list[i]);
		}	
	}

	ground_plane_list = temp_ground_plane_list;
}

void ObjectRemoval::RemoveObject(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices object_index)
{
	for(size_t i = 0; i < object_index.indices.size(); ++i)
	{
		cloud->points[object_index.indices[i]].x = cloud->points[object_index.indices[i]].y = cloud->points[object_index.indices[i]].z = 0;
	}
}

void ObjectRemoval::RemoveObject(PointCloud<PointXYZRGB>::Ptr cloud, vector<ModelCoefficients> &coefficient_list, vector<PointIndices> & plane_index_list, PointIndices &object_index, vector<size_t> &object_plane_list, vector<size_t> &ground_plane_list, vector<PointIndices> &restored_indices, vector<PointCloud<PointXYZRGB>::Ptr> &restored_points)
{
	Vector3f normal_ground_plane;
	Vector3f normal_object_plane;

	size_t size_cloud = cloud->size();
	size_t size_object = object_index.indices.size();
	size_t index_new = 0;
//	PointCloud<PointXYZRGB>::Ptr restored_points(new PointCloud<PointXYZRGB>());

	//Project all points of the object to the ground planes
	for(size_t i = 0; i < ground_plane_list.size(); ++i)
	{
		PointIndices indices;
		PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>());

/*		srand(time(NULL));
		for(size_t j = 0; j < 100; ++j)
		{
			int point_index = rand() % plane_index_list[ground_plane_list[i]].indices.size();
			r += (float) (cloud->points[plane_index_list[ground_plane_list[i]].indices[point_index]].r) / 100.0;
			g += (float) (cloud->points[plane_index_list[ground_plane_list[i]].indices[point_index]].g) / 100.0;
			b += (float) (cloud->points[plane_index_list[ground_plane_list[i]].indices[point_index]].b) / 100.0;
		}*/

		normal_ground_plane << coefficient_list[ground_plane_list[i]].values[0], coefficient_list[ground_plane_list[i]].values[1], coefficient_list[ground_plane_list[i]].values[2];

		for(size_t j = 0; j < size_object; ++j)
		{
			Normal point_normal = MainProgram::normal_model->points[object_index.indices[j]];
			Vector3f vnormal(point_normal.normal_x, point_normal.normal_y, point_normal.normal_z);
			if(abs(vnormal.dot(normal_ground_plane) / (vnormal.norm() * normal_ground_plane.norm())) > 0.5)
			{
				// identify the parameter of the line crossing the point and perpendicular to the plane a + tb
				float t = normal_ground_plane(0,0) * cloud->points[object_index.indices[j]].x
						+ normal_ground_plane(1,0) * cloud->points[object_index.indices[j]].y
						+ normal_ground_plane(2,0) * cloud->points[object_index.indices[j]].z
						+ coefficient_list[ground_plane_list[i]].values[3];

				t = -t / (normal_ground_plane(0,0) * normal_ground_plane(0,0)
						+ normal_ground_plane(1,0) * normal_ground_plane(1,0)
						+ normal_ground_plane(2,0) * normal_ground_plane(2,0));

				// transform the point along the plane's normal vector
				PointXYZRGB point1;
				point1.x  = cloud->points[object_index.indices[j]].x + t * normal_ground_plane(0,0);
				point1.y  = cloud->points[object_index.indices[j]].y + t * normal_ground_plane(1,0);
				point1.z  = cloud->points[object_index.indices[j]].z + t * normal_ground_plane(2,0);
				point1.r = 0;
				point1.g = 0;
				point1.b = 0;
				temp_cloud->push_back(point1);
				indices.indices.push_back(size_cloud - size_object + index_new);
				++ index_new;
			}
		}
		restored_indices.push_back(indices);
		restored_points.push_back(temp_cloud);
	}

	//Apply voxel-grid filter to the restored surface to normalize the density
/*	cout << endl << "Model size before filtering = " << restored_points->size() << endl;
	VoxelGrid<PointXYZRGB> grid;
	grid.setInputCloud(restored_points);
	grid.setLeafSize(0.01f, 0.01f, 0.01f);
	grid.filter(*restored_points);

	cout << endl << "Model size after filtering = " << restored_points->size() << endl;*/

	for(size_t i = 0; i < size_object; ++i)
	{
		//Mark xyz = 0 to know that this point is in object
		cloud->points[object_index.indices[i]].x = cloud->points[object_index.indices[i]].y = cloud->points[object_index.indices[i]].z = 0;
	}

	PointCloud<PointXYZRGB>::Ptr temp(new PointCloud<PointXYZRGB>());

	for(size_t j = 0; j < size_cloud; ++j)
	{
		if(cloud->points[j].x != 0 || cloud->points[j].y != 0 || cloud->points[j].z != 0)
		{
			temp->push_back(cloud->points[j]);
		}
	}

	cloud->clear();
	cloud->resize(temp->size());
	copyPointCloud(*temp, *cloud);

	//attach the restored surface to the cloud
	size_t size_list_restored_points = restored_points.size();
	for(size_t i = 0; i < size_list_restored_points; ++i)
	{
		size_t size_restored_points = restored_points[i]->size();
		for(size_t j = 0; j < size_restored_points; ++j)
			cloud->push_back(restored_points[i]->points[j]);
	}
	cout << endl << " Index list size = " << restored_indices.size() << endl;
	cout << endl << " Point list size = " << restored_points.size() << endl;
}

void ObjectRemoval::FuseColorToRestoredSurface(PointCloud<PointXYZRGB>::Ptr cloud, vector<PointIndices> &restored_indices, vector<PointCloud<PointXYZRGB>::Ptr> &restored_points)
{
	//vector<PointIndices> list_visited;
	KdTreeFLANN<PointXYZRGB> tree;
	vector<int> neighbors;
	vector<float> sqrt_distances;

	size_t size_restored_list = restored_points.size();

	cout << endl << "Fusing color " << endl;
	cout << endl << " Index list size = " << restored_indices.size() << endl;
	cout << endl << " Point list size = " << restored_points.size() << endl;

	for(size_t i = 0; i < restored_indices.size(); ++i)
	{
		cout << endl << "Index list " << i << ": " << restored_indices[i].indices.size() << endl;
		cout << endl << "Point list " << i << ": " << restored_points[i]->size() << endl;
	}

	// Find the correct sequence of points to fill color and save to list_visited
	for(size_t index = 0; index < size_restored_list; ++index)
	{	
		//calculate centroid of cloud
		Vector4f centroid;
		compute3DCentroid(*restored_points[index], centroid);
	
		vector<int> list_distance_2_center;
		for(size_t i = 0; i < restored_points[index]->size(); ++i)
		{
			float distance_sqrt = (centroid(0,0) - restored_points[index]->points[i].x) * (centroid(0,0) - restored_points[index]->points[i].x)
								+ (centroid(1,0) - restored_points[index]->points[i].y) * (centroid(1,0) - restored_points[index]->points[i].y)
								+ (centroid(2,0) - restored_points[index]->points[i].z) * (centroid(2,0) - restored_points[index]->points[i].z);

			list_distance_2_center.push_back((int)(distance_sqrt * 10000));
		}

		int size_list_distance = list_distance_2_center.size();
		SupportingFunctions::quicksort(list_distance_2_center, 0, size_list_distance - 1, restored_indices[index].indices);

		/*
		//push the calculated centroid to the restored cloud to use kd-tree
		restored_points[index]->push_back(center_point);
		restored_indices[index].indices.push_back(restored_points[index]->size() - 1);
	
		//Build KdTree
		tree.setInputCloud(restored_points[index]);

		//Push the centroid (the last index) to the visited list
		visited_points.indices.push_back(restored_points[index]->size() - 1);
		is_in_visited_list[restored_points[index]->size() - 1] = true;

		for(size_t i = 0; i < visited_points.indices.size(); ++i)
		{
			PointXYZRGB search_point;
			tree.radiusSearch(visited_points.indices[i], SEARCH_RADIUS_, neighbors, sqrt_distances);
			for(size_t j = 0; j < neighbors.size(); ++j)
			{
				if(!(is_in_visited_list[neighbors[j]]))
				{
					// the jth element in neighbors is the jth index int the restored list
					visited_points.indices.push_back(neighbors[j]);
					is_in_visited_list[neighbors[j]] = true;
				}
			}
		}
		*/

//		list_visited.push_back(visited_points);
//		cout << endl << restored_indices[index].indices.size() << " ------ " << list_visited.indices.size() << endl;
	}

	//convert the visited list to the original cloud indices
	/*
	size_t size_visited_list = list_visited.size();
	for(size_t i = 0; i < size_visited_list; ++i)
	{
		size_t size_visited_points = list_visited[i].indices.size();
		for(size_t j = 1; j < size_visited_points; ++j)
			list_visited[i].indices[j] = restored_indices[i].indices[list_visited[i].indices[j]];
	}

	cout << endl << "Conversion finished. " << endl;
	*/

	/*
	for(size_t i = 0; i < size_restored_list; ++i)
	{
		int r = 30, g = 60, b = 70;
		size_t size_visited_points = restored_indices[i].indices.size();
		for(size_t j = 0; j < size_visited_points; ++j)
		{
			int point_index = restored_indices[i].indices[j];
			cloud->points[point_index].r = (i + 1) * r;
			cloud->points[point_index].g = (i + 1) * g;
			cloud->points[point_index].b = (i + 1) * b;
		}
	}

	int r = 255, g = 250, b = 230;

	for(size_t i = 0; i < size_restored_list; ++i)
	{
		size_t size_visited_points = restored_indices[i].indices.size();
		for(size_t j = 0; j < 30; ++j)
		{
			int point_index = restored_indices[i].indices[j];
			cloud->points[point_index].r = r;
			cloud->points[point_index].g = g;
			cloud->points[point_index].b = b;
		}
	}
	*/
	
	//Build Kd Tree on the whole point cloud
	cout << "sorting complete" << endl;
	
	tree.setInputCloud(cloud);

	// start filling color using Simple Moving Average, traverse from the border to the center of the region	
	for(size_t i = 0; i < size_restored_list; ++i)
	{
		cout << "Plane " << i << endl;
		size_t size_visited_points = restored_indices[i].indices.size();
		for(size_t j = size_visited_points - 1; j > 0; --j)
		{
			//cout << j << " ";
			//Find neighbors
			neighbors.clear();
			sqrt_distances.clear();
			tree.radiusSearch(restored_indices[i].indices[j], 0.15, neighbors, sqrt_distances);
			//cout << endl << neighbors.size();
			float r = 0.0f, g = 0.0f, b = 0.0f;
			int size_color_points = 0;
			int size_neighbors = neighbors.size();
			/*
			//build the distance list to sort the neighbor list
			vector<int> distance_neighbors;
			vector<int> color_indices;

			for(size_t u = 0; u < size_neighbors; ++u)
			{
				PointXYZRGB point = cloud->points[neighbors[u]];
				if(point.r > 0 || point.g > 0 || point.b > 0)
				{
					size_color_points++;
					PointXYZRGB point_center = cloud->points[restored_indices[i].indices[u]];
					int distance_sqrt = (point.x - point_center.x)*(point.x - point_center.x) +
										(point.y - point_center.y)*(point.y - point_center.y) +
										(point.z - point_center.z)*(point.z - point_center.z);

					distance_neighbors.push_back(distance_sqrt);
					color_indices.push_back(neighbors[u]);
				}

			}

			//sort the neighbors
			SupportingFunctions::quicksort(distance_neighbors, 0, distance_neighbors.size() - 1, color_indices);
			
			for(size_t u = 0; u < size_color_points; ++u)
			{
				PointXYZRGB point = cloud->points[color_indices[u]];
				r += (size_color_points - u) * point.r;
				g += (size_color_points - u) * point.g;
				b += (size_color_points - u) * point.b;
			}
			*/			

			for(size_t u = 0; u < size_neighbors; ++u)
			{
				PointXYZRGB point = cloud->points[neighbors[u]];
				PointXYZRGB center_point = cloud->points[restored_indices[i].indices[u]];
				if(point.r > 0 || point.g > 0 || point.b > 0)
				{
					r += point.r;
					g += point.g;
					b += point.b;
					size_color_points++;
				}
			}

			//Assign color to real point
			if(size_color_points > 0)
			{
				cloud->points[restored_indices[i].indices[j]].r = r / size_color_points;
				cloud->points[restored_indices[i].indices[j]].g = g / size_color_points;
				cloud->points[restored_indices[i].indices[j]].b = b / size_color_points;
			}

		}
		cout << endl;
	}
}
