#include "GraphBasedSegmentation.h"

PointIndices GraphBasedSegmentation::foreground_points_;
PointIndices GraphBasedSegmentation::background_points_;
bool GraphBasedSegmentation::foreground_updated_;
bool GraphBasedSegmentation::background_updated_;

GraphBasedSegmentation::GraphBasedSegmentation(void)
{
}

GraphBasedSegmentation::~GraphBasedSegmentation(void)
{
}

void GraphBasedSegmentation::SetForeground(PointIndices foreground)
{
	foreground_points_ = foreground;
	foreground_updated_ = false;
}

void GraphBasedSegmentation::SetBackground(PointIndices background)
{
	background_points_ = background;
	background_updated_ = false;
}

void GraphBasedSegmentation::AddForeground(PointIndices foreground)
{
	for(size_t i = 0; i < foreground.indices.size(); ++i)
	{
		if (MainProgram::SearchForObject(foreground.indices[i], foreground_points_.indices) == -1)
			foreground_points_.indices.push_back(foreground.indices[i]);
	}
	foreground_updated_ = true;
}

void GraphBasedSegmentation::AddBackground(PointIndices background)
{
	for(size_t i = 0; i < background.indices.size(); ++i)
	{
		if (MainProgram::SearchForObject(background.indices[i], background_points_.indices) == -1)
			background_points_.indices.push_back(background.indices[i]);
	}
	background_updated_ = true;
}

bool GraphBasedSegmentation::ForegroundIsEmpty()
{
	if(foreground_points_.indices.empty())
		return true;
	return false;
}

bool GraphBasedSegmentation::BackgroundIsEmpty()
{
	if(background_points_.indices.empty())
		return true;
	return false;
}

void GraphBasedSegmentation::ClearForeground()
{
	foreground_points_.indices.clear();
}

void GraphBasedSegmentation::ClearBackground()
{
	background_points_.indices.clear();
}

void GraphBasedSegmentation::MinCutBaseSegmentation(PointCloud<PointXYZRGB>::Ptr xyzcloud, vector<PointIndices> &clusters)
{
	MinCutSegmentation<PointXYZRGB> seg;
	seg.setInputCloud (xyzcloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr background_points(new pcl::PointCloud<pcl::PointXYZRGB> ());
	PointXYZRGB point, radius_point;
	point.x = xyzcloud->points[MainProgram::picked_points->indices[0]].x;
	point.y = xyzcloud->points[MainProgram::picked_points->indices[0]].y;
	point.z = xyzcloud->points[MainProgram::picked_points->indices[0]].z;

	for(size_t i = 0; i < MainProgram::picked_points->indices.size() / 2; ++i)
	{
		foreground_points->points.push_back(xyzcloud->points[MainProgram::picked_points->indices[i]]);
		background_points->points.push_back(xyzcloud->points[MainProgram::picked_points->indices[i + MainProgram::picked_points->indices.size() / 2]]);
	}
	seg.setForegroundPoints(foreground_points);
	seg.setBackgroundPoints(background_points);

	/*
	radius_point.x = xyzcloud->points[MainProgram::picked_points->indices[MainProgram::picked_points->indices.size() / 2]].x;
	radius_point.y = xyzcloud->points[MainProgram::picked_points->indices[MainProgram::picked_points->indices.size() / 2]].y;
	radius_point.z = xyzcloud->points[MainProgram::picked_points->indices[MainProgram::picked_points->indices.size() / 2]].z;

	float radius_squared =  (point.x - radius_point.x) * (point.x - radius_point.x)
				+	(point.y - radius_point.y) * (point.y - radius_point.y)
				+	(point.z - radius_point.z) * (point.z - radius_point.z);*/

	seg.setRadius(0.5);
	seg.setSigma (0.25);
	seg.setNumberOfNeighbours(14);
	seg.setSourceWeight(0.8);
	
	seg.extract(clusters);

	MainProgram::viewer.updatePointCloud(seg.getColoredCloud(), "cloud");
}

void GraphBasedSegmentation::SegmentUsingMyMinCut(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals, vector<PointIndices> &clusters)
{
	MyMinCut seg;
	seg.setInputCloud(cloud);
	seg.setInputNormal(normals);

	seg.setForegroundPoints( foreground_points_);
	seg.setBackgroundPoints( background_points_);
	seg.setRadius(25);
	seg.setSigma (10, 1);
	seg.setNumberOfNeighbours(10);
	seg.setSourceWeight(0.55);
	
	seg.extract(clusters);

	clusters.clear();

	clusters.insert(clusters.end(), seg.clusters_.begin(), seg.clusters_.end());
}

PointIndices::Ptr GraphBasedSegmentation::GetEdgeBuffer(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices &object, float size, vector<bool> &is_in_object_buffer) 
{
	PointIndices::Ptr edge_buffer(new PointIndices());

	KdTreeFLANN<PointXYZRGB> tree;
	tree.setInputCloud(cloud);
	size_t size_object = object.indices.size();
	vector<bool> is_in_object;
	vector<bool> is_in_buffer;
	is_in_object.resize(cloud->size());
	is_in_buffer.resize(cloud->size());

	//Mark the object points as "is in object"
	for(size_t i = 0; i < size_object; ++i)
	{
		is_in_object[object.indices[i]] = true;
	}

	for(size_t i = 0; i < size_object; ++i)
	{
		//Search for all neighbors inside buffer width
		vector<float> sqrt_distance;
		vector<int> neighbors;
		tree.radiusSearch(i, size, neighbors, sqrt_distance);
		size_t n_in_object = 0;

		//Calculate the ratio between point inside | outside object
		size_t size_neighbors = neighbors.size();
		for(size_t j = 0; j < size_neighbors; ++j)
		{
			if(is_in_object[neighbors[j]])
				n_in_object++;
		}

		//if not most of the neighbors are in object or background, then the point should be in the edge
		float ratio = (float)n_in_object / (float)size_neighbors;
		if(ratio > 0.3 && ratio < 0.7)
		{
			for(size_t j = 0; j < size_neighbors; ++j)
			{
				if(!is_in_buffer[neighbors[j]])
				{
					edge_buffer->indices.push_back(neighbors[j]);
					is_in_buffer[neighbors[j]] = true;
					if(is_in_object[neighbors[j]])
						is_in_object_buffer.push_back(true);
					else
						is_in_object_buffer.push_back(false);
				}
			}
		}
	}

	return edge_buffer;
}

void GraphBasedSegmentation::RefineMinCutSegmentation(PointCloud<PointXYZRGB>::Ptr cloud, vector<PointIndices> &cluster)
{
	//Get a buffer around the edge of the object
	vector<bool> is_in_object_buffer;
	PointIndices::Ptr edge_buffer = GetEdgeBuffer(cloud, cluster[1], 0.08, is_in_object_buffer);

	//Get the real point cloud
	size_t size_buffer = edge_buffer->indices.size();
	PointCloud<PointXYZRGB>::Ptr buffer(new PointCloud<PointXYZRGB>());
	for(size_t i = 0; i < size_buffer; ++i)
	{
		buffer->push_back(cloud->points[edge_buffer->indices[i]]);
	}

	PointIndices::Ptr old;
	vector<size_t> plane_map_buffer;
	vector<PointIndices> plane_list_buffer;
	vector<ModelCoefficients> equation_list;

	//Segment the planes of the buffer
	PlaneSegmentation::GetAllPlanes(buffer, old, 0.03, plane_map_buffer, plane_list_buffer, equation_list);

	//Examining the plane list to specify the object planes and background planes
	size_t size_plane_list = plane_list_buffer.size();
	for(size_t i = 0; i < size_plane_list; ++i) 
	{
		size_t size_plane = plane_list_buffer[i].indices.size();
		size_t n_in_object = 0;
		for(size_t j = 0; j < size_plane; ++j)
		{
			//calculate the ratio between points inside object / points outside object
			if(is_in_object_buffer[plane_list_buffer[i].indices[j]])
				n_in_object ++;
		}

		//if most of the points are in object => the whole plane is in object
		if(n_in_object / size_plane > 0.5)
		{
			for(size_t j = 0; j < size_plane; ++j)
			{
				if(!is_in_object_buffer[plane_list_buffer[i].indices[j]])
				{
					cluster[1].indices.push_back(edge_buffer->indices[plane_list_buffer[i].indices[j]]);
					is_in_object_buffer[plane_list_buffer[i].indices[j]] = true;
				}
			}
		}
	}
}