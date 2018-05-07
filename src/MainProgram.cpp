#include "MainProgram.h"

int MainProgram::mode;
int MainProgram::model_type;
PointIndices::Ptr MainProgram::picked_points;
visualization::PCLVisualizer MainProgram::viewer;
PolygonMesh::Ptr MainProgram::mesh_model;
PointCloud<PointXYZRGB>::Ptr MainProgram::model;
PointCloud<PointXYZRGB>::Ptr MainProgram::edited_model;
PointCloud<Normal>::Ptr MainProgram::normal_model;
vector<PointIndices> MainProgram::clusters;
vector<bool> MainProgram::is_in_object;
vector<pcl::visualization::Camera> MainProgram::camera;
int MainProgram::PointFilter = 0;
int MainProgram::iObject = 0;

const int MainProgram::POINT_FILTER_LIMITATION = 2;

MainProgram::MainProgram(void)
{
}

MainProgram::~MainProgram(void)
{
}

void MainProgram::Initialize()
{
	mode = 0;
//	object_normal << 0.010406317, -0.89685518, -0.44220197;
	picked_points.reset(new PointIndices());
	viewer.setWindowName("Point Cloud View");
	model.reset(new PointCloud<PointXYZRGB>());
	edited_model.reset(new PointCloud<PointXYZRGB>());
	normal_model.reset(new PointCloud<Normal>());
	model_type = 0;
}

Normal MainProgram::GetNormal(int plane_index)
{
	Normal normal_plane;

	normal_plane.normal_x = PlaneSegmentation::coefficient_list[plane_index].values[0];
	normal_plane.normal_y = PlaneSegmentation::coefficient_list[plane_index].values[1];
	normal_plane.normal_z = PlaneSegmentation::coefficient_list[plane_index].values[2];

	return normal_plane;
}

int MainProgram::SearchForObject(size_t index_point, vector<size_t> &map)
{
	for(size_t i = 0; i < map.size(); ++i)
	{
		if(index_point == map[i])
			return i;
	}

	return -1;
}

int MainProgram::SearchForObject(size_t index_point, vector<int> &list)
{
	for(size_t i = 0; i < list.size(); ++i)
	{
		if(index_point == list[i])
			return i;
	}

	return -1;
}

int MainProgram::SearchForObject(size_t index_point, vector<PointIndices> &list)
{
	for(size_t i = 0; i < list.size(); ++i)
	{
		for(size_t j = 0; j < list[i].indices.size(); ++j)
			if(index_point == list[i].indices[j])
				return i;
	}

	return -1;
}

void MainProgram::RemoveOutliers(PointCloud<PointXYZRGB>::Ptr cloud_in, PointCloud<PointXYZRGB>::Ptr &cloud_out, int meanK, float stdDev)
{
	StatisticalOutlierRemoval<PointXYZRGB> sor;
	sor.setInputCloud (cloud_in);
	sor.setMeanK (meanK);
	sor.setStddevMulThresh (stdDev);
	sor.filter (*cloud_out);
}

void MainProgram::FillColor2Cloud(PointCloud<PointXYZRGB>::Ptr cloud, vector<PointIndices> cluster_list)
{
	srand (time(NULL) );
	size_t size = cluster_list.size();
	size_t size_cluster;
	int r, g, b;

	for(size_t j = 0; j < size; ++j)
	{
		size_cluster = cluster_list[j].indices.size();

		r = rand()%256 + 1;
		g = rand()%256 + 1;
		b = rand()%256 + 1;

		for(size_t i = 0; i < size_cluster; ++i)
		{
			cloud->points[cluster_list[j].indices[i]].r = r;
			cloud->points[cluster_list[j].indices[i]].g = g;
			cloud->points[cluster_list[j].indices[i]].b = b;
		}
	}
}

void MainProgram::pp_callback (const pcl::visualization::PointPickingEvent& event, void* cookie)
{
	pcl::search::KdTree<pcl::PointXYZRGB> search_pt;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzcloud;
	int idx = event.getPointIndex ();
	if (idx == -1)
	return;
		
//	xyzcloud = *reinterpret_cast<PointCloud<PointXYZRGB>::Ptr*>(cookie);
	search_pt.setInputCloud(model);

	// Return the correct index in the cloud instead of the index on the screen
	std::vector<int> indices (1);
	std::vector<float> distances (1);

	// Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search_pt for the real point
	pcl::PointXYZRGB picked_pt;
	event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);
	search_pt.nearestKSearch (picked_pt, 1, indices, distances);

	PCL_INFO ("Point index picked: %d (real: %d) - [%f, %f, %f]\n", idx, indices[0], picked_pt.x, picked_pt.y, picked_pt.z);
	picked_points->indices.push_back(indices[0]);

	size_t index_plane = PlaneSegmentation::plane_map[indices[0]];

	if(mode == 0)
	{
		//Get destination normal vector
		//Normal n = GetNormal(index_plane);
		Normal n = MainProgram::normal_model->points[indices[0]];
		Vector3f normal_plane(n.normal_x, n.normal_y, n.normal_z);
		
		cout << endl << "Plane normal = " << n.normal_x << "," << n.normal_y << "," << n.normal_z << endl;

		time_t t1 = clock();
		
		Vector3f des_point;
		des_point << model->points[indices[0]].x , model->points[indices[0]].y , model->points[indices[0]].z;
		ObjectInsertion::TransformPointCloud(ObjectInsertion::object_library[iObject], ObjectInsertion::object_normals[iObject], normal_plane, des_point, ObjectInsertion::object_standings[iObject]);
		ObjectInsertion::object_standings[iObject] = des_point;
		char buffer[30];
		char * cloud_name = itoa(iObject, buffer, 10);
		cloud_name = strcat(cloud_name, "object");

		if(model_type == 0)
		{
			viewer.removePointCloud(cloud_name);
			viewer.addPointCloud(ObjectInsertion::object_library[iObject], cloud_name);
		}
		else
		{
			ObjectInsertion::object_meshes[iObject]->cloud.data.clear();
			toROSMsg(*ObjectInsertion::object_library[iObject], ObjectInsertion::object_meshes[iObject]->cloud);
			viewer.removePolygonMesh(cloud_name);
			viewer.addPolygonMesh(*ObjectInsertion::object_meshes[iObject],cloud_name);
		}
		//cout << endl << "Object update time = " << (float)t_update_2 / CLOCKS_PER_SEC << endl;

		time_t runtime = clock() - t1;

		cout << "Transformation time = " << (float)(runtime) / CLOCKS_PER_SEC << endl;
	}
	else if(mode == 1)
	{
		vector<PointIndices> list_subcloud;

		cout << endl << "Running k-means with k = " << ClusteringBasedSegmentation::k << "..." << endl;
		time_t t1 = clock();
		
		PointIndices::Ptr old;
		PointCloud<PointXYZRGB>::Ptr cloud_new = ClusteringBasedSegmentation::RemoveBackgroundPlanes(*model, PlaneSegmentation::plane_index_list, old);
		ClusteringBasedSegmentation::SegmentUsingKMeans(cloud_new, 100, list_subcloud);

		time_t runtime = clock() - t1;

		cout << endl << "segmentation time = " << (float)runtime / CLOCKS_PER_SEC << endl;

		int new_index = old->indices[indices[0]];
		int index_object = SearchForObject(new_index, list_subcloud);
		clusters.clear();
		PointIndices i;
		clusters.push_back(i);
		clusters.push_back(list_subcloud[index_object]);
		for(size_t i = 0; i < list_subcloud[index_object].indices.size(); ++i)
		{
			clusters[1].indices[i] = SearchForObject(clusters[1].indices[i], old->indices);
		}
		

		PointCloud<PointXYZRGB>::Ptr segmented_object(new PointCloud<PointXYZRGB>());

		for(size_t i = 0; i < list_subcloud[index_object].indices.size(); ++i)
		{
			segmented_object->points.push_back(cloud_new->points[list_subcloud[index_object].indices[i]]);
		}

		if(model_type == 0)
			viewer.updatePointCloud(segmented_object, "cloud");
		else
		{
			PolygonMesh::Ptr mesh_segmented_object = Triangulation::FullTriangulate(segmented_object);
			viewer.removePolygonMesh("model");
			viewer.addPolygonMesh(*mesh_segmented_object, "model");
		}

		cout << endl << "Press b to increase k ..." << endl;
	}
	else if(mode == 4)
	{
		PointIndices::Ptr list_subcloud (new PointIndices());

		cout << endl << "Running DBSCAN after removing n = " << ClusteringBasedSegmentation::n_planes << " planes ..." << endl;
		time_t t1 = clock();
		
		PointIndices::Ptr old;
		PointCloud<PointXYZRGB>::Ptr cloud_new = ClusteringBasedSegmentation::RemoveBackgroundPlanes(*model, PlaneSegmentation::plane_index_list, old);

		int new_index = old->indices[indices[0]];
		ClusteringBasedSegmentation::SegmentUsingModifiedDBSCAN(cloud_new, new_index, 0.04, 100, *list_subcloud);

		time_t runtime = clock() - t1;

		cout << endl << "segmentation time = " << (float)runtime / CLOCKS_PER_SEC << endl;

		clusters.clear();
		PointIndices i;
		clusters.push_back(i);
		clusters.push_back(*list_subcloud);
		for(size_t i = 0; i < list_subcloud->indices.size(); ++i)
		{
			clusters[1].indices[i] = SearchForObject(clusters[1].indices[i], old->indices);
		}
		

		PointCloud<PointXYZRGB>::Ptr segmented_object(new PointCloud<PointXYZRGB>());

		for(size_t i = 0; i < list_subcloud->indices.size(); ++i)
		{
			segmented_object->points.push_back(cloud_new->points[list_subcloud->indices[i]]);
		}

		if(model_type == 0)
			viewer.updatePointCloud(segmented_object, "cloud");
		else
		{
			PolygonMesh::Ptr mesh_segmented_object = Triangulation::FullTriangulate(segmented_object);
			viewer.removePolygonMesh("model");
			viewer.addPolygonMesh(*mesh_segmented_object, "model");
		}

		cout << endl << "Press b to increase n ..." << endl;
	}
} 

void MainProgram::Helper()
{
	cout << "------------------------------ Help ------------------------------" << endl;
	cout << "This is the key command list: " << endl;
	cout << " 'c' : Clear the picked-point list " << endl;
	cout << " 'x' : Reload the model" << endl;
	cout << " 's' : Fill color to model planes	" << endl;
	cout << " 'n' : Change mode(See mode list) " << endl;
	cout << " 'b' : Forward to the next stage of object segmentation process " << endl;
	cout << " 't' : Take segmented object out" << endl;
	cout << " 'v' : Save segmented object to file" << endl;
	cout << " 'd' : Remove segmented object from model" << endl;
	cout << " 'a' : Self rotation " << endl;
	cout << " 'z' : Rotate the object by 180 degree" << endl;
	cout << "==================================================================" << endl;
}

void MainProgram::kb_callback (const pcl::visualization::KeyboardEvent& event, void* cookie)
{
	//Get the input cloud
	/*
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzcloud;
	xyzcloud = *reinterpret_cast<PointCloud<PointXYZRGB>::Ptr*>(cookie);
	*/

	char character = event.getKeyCode();
	if (event.keyUp())
		return;

	time_t t_mincut_1, runtime;

	switch (character)
	{
		case 's':
		{
			FillColor2Cloud(edited_model,PlaneSegmentation::plane_index_list);
			if(model_type == 0)
			{
				viewer.updatePointCloud(edited_model,"cloud");
			}
			else
			{
				mesh_model->cloud.data.clear();
				toROSMsg(*edited_model, mesh_model->cloud);
				viewer.removePolygonMesh("model");
				viewer.addPolygonMesh(*mesh_model, "model");
			}
			break;
		}
#pragma region c
		case 'c':
		{
			picked_points->indices.clear();
			break;
		}
#pragma endregion c

#pragma region x
		case 'x':
		{	
			cout << endl << "Reloading model ... " << endl;

			time_t t1 = clock();

			if(model_type == 0)
			{
				viewer.updatePointCloud(model, "cloud");
			}
			else
			{
				mesh_model->cloud.data.clear();
				toROSMsg(*model, mesh_model->cloud);
				viewer.removePolygonMesh("model");
				viewer.addPolygonMesh(*mesh_model, "model");
			}

			copyPointCloud(*model, *edited_model);

			time_t t2 = clock();

			cout << endl << "Total loading time = " << (float) (t2 - t1) / CLOCKS_PER_SEC;
			break;
		}
#pragma endregion x

#pragma region n
		case 'n':
		{
			mode = (mode + 1) % 6;
			cout << endl << "mode = " << mode;
			switch (mode)
			{
				case 0:
					cout << " ======> Object insertion mode" << endl;
					break;
				case 1:
				{
					//Initialize k and number of planes to remove
					cout << " ======> K-means segmentation mode" << endl;

					ClusteringBasedSegmentation::k = 2;
					ClusteringBasedSegmentation::n_planes = 1;
					break;
				}
				case 2:
				{
					cout << " ======> Minimum cut segmentation mode" << endl;
					char a;
					if(!GraphBasedSegmentation::BackgroundIsEmpty() && !GraphBasedSegmentation::ForegroundIsEmpty())
					{
						cout << endl << "Do you want to clear data? Y/N: ";
						cin >> a;
						cout << endl;
						if(a == 'Y' || a == 'y')
						{
							GraphBasedSegmentation::ClearForeground();
							GraphBasedSegmentation::ClearBackground();
						}
					}
					break;
				}
				case 3:
				{
					cout << " ======> Update Min-cut segmentation mode" << endl;
					GraphBasedSegmentation::foreground_updated_ = GraphBasedSegmentation::background_updated_ = false;
					break;
				}
				case 4:
				{
					//Initialize number of planes to remove
					cout << " ======> DBSCAN mode" << endl;
					ClusteringBasedSegmentation::n_planes = 1;
					break;
				}
				case 5:
				{
					cout << " ======> Refine Graph-cuts mode" << endl;
					break;
				}
				default:
					break;
			}
			break;
		}
#pragma endregion n
		//Graph cut segmentation commands
#pragma region b
		case 'b':
		{
			switch(mode)
			{
				case 0:
				{
					iObject = (iObject + 1) % ObjectInsertion::object_library.size();
					cout << endl << "Object " << iObject << " picked" << endl;
					break;
				}
				// Increase k and run k-means
				case 1:
				{
					if(model_type == 0)
						viewer.updatePointCloud(model, "cloud");
					else
					{
						viewer.removePolygonMesh("model");
						viewer.addPolygonMesh(*mesh_model, "model");
					}
					cout << "Enter k and number of planes to remove : ";
					cin >> ClusteringBasedSegmentation::k >> ClusteringBasedSegmentation::n_planes;
					cout << endl;

					break;
				}
				// Run graph-cut segmentation
				case 2:
				{
					if(GraphBasedSegmentation::ForegroundIsEmpty())
					{
						GraphBasedSegmentation::SetForeground(*picked_points);
						picked_points->indices.clear();
						cout << endl << "Foreground points are ready. Please pick background points !" << endl;
					}
					else
					{
						GraphBasedSegmentation::SetBackground(*picked_points);
						picked_points->indices.clear();
						cout << endl << "Background points are ready. Executing min-cut segmentation ... " << endl;

						t_mincut_1 = clock();

						GraphBasedSegmentation::SegmentUsingMyMinCut(model, normal_model, clusters);

						runtime = clock() - t_mincut_1;
						cout << endl << "Min-cut segmentation completed!" << endl;
						cout << endl << "Total runtime = " << (float)runtime / CLOCKS_PER_SEC << " seconds = " << (float)runtime / CLOCKS_PER_SEC / 60 << " minutes" << endl;					

						FillColor2Cloud(edited_model, clusters);

						if(model_type == 0)
						{
							viewer.updatePointCloud(edited_model, "cloud");
						}
						else
						{
							mesh_model->cloud.data.clear();
							toROSMsg(*edited_model, mesh_model->cloud);

							viewer.removePolygonMesh("model");
							viewer.addPolygonMesh(*mesh_model, "model");
						}
					}
					break;
				}
				// Modify background, foreground and re-run graph-cut segmentation
				case 3:
				{
					if(GraphBasedSegmentation::foreground_updated_ == false)
					{
						GraphBasedSegmentation::AddForeground(*picked_points);
						picked_points->indices.clear();
						//GraphBasedSegmentation::foreground_updated_ = true;
						cout << endl << "Foreground seeds updated. Please update background seeds!" << endl;
					}
					else if(GraphBasedSegmentation::background_updated_ == false)
					{
						GraphBasedSegmentation::AddBackground(*picked_points);
						picked_points->indices.clear();
						//GraphBasedSegmentation::background_updated_ = true;
						cout << endl << "Re-executing min-cut segmentation ..." << endl;

						t_mincut_1 = clock();

						GraphBasedSegmentation::SegmentUsingMyMinCut(model, normal_model, clusters);

						runtime = clock() - t_mincut_1;
						cout << endl << "Min-cut segmentation completed!" << endl;
						cout << endl << "Total runtime = " << (float)runtime / CLOCKS_PER_SEC << " seconds = " << (float)runtime / CLOCKS_PER_SEC / 60 << " minutes" << endl;					

						FillColor2Cloud(edited_model, clusters);

						if(model_type == 0)
						{
							viewer.updatePointCloud(edited_model, "cloud");
						}
						else
						{
							mesh_model->cloud.data.clear();
							toROSMsg(*edited_model, mesh_model->cloud);

							viewer.removePolygonMesh("model");
							viewer.addPolygonMesh(*mesh_model, "model");
						}
					}
					break;
				}
				case 4:
				{
					if(model_type == 0)
						viewer.updatePointCloud(model, "cloud");
					else
					{
						viewer.removePolygonMesh("model");
						viewer.addPolygonMesh(*mesh_model, "model");
					}
					cout << "Enter number of planes to remove : ";
					cin >> ClusteringBasedSegmentation::n_planes;
					cout << endl;

					break;
				}
				case 5:
				{
					//GraphBasedSegmentation::RefineMinCutSegmentation(model, clusters);
					vector<bool> is_in_object_buffer;
					PointIndices::Ptr edge_buffer = GraphBasedSegmentation::GetEdgeBuffer(model, clusters[1], 0.08, is_in_object_buffer);
					vector<PointIndices> clusters_;
					PointIndices i;
					clusters_.push_back(i);
					clusters_.push_back(*edge_buffer);
					FillColor2Cloud(edited_model, clusters_);

					if(model_type == 0)
					{
						viewer.updatePointCloud(edited_model, "cloud");
					}
					else
					{
						mesh_model->cloud.data.clear();
						toROSMsg(*edited_model, mesh_model->cloud);

						viewer.removePolygonMesh("model");
						viewer.addPolygonMesh(*mesh_model, "model");
					}
					break;
				}
				default:
					break;
			}
			break;
		}
#pragma endregion b

#pragma region t
		case 't':
		{
			if(clusters.size() > 0)
			{
				PointCloud<PointXYZRGB>::Ptr object(new PointCloud<PointXYZRGB>());
				for(size_t i = 0; i < clusters[1].indices.size(); ++i)
				{
					object->push_back(model->points[clusters[1].indices[i]]);
				}

				if(model_type == 0)
					viewer.updatePointCloud(object, "cloud");
				else
				{
					PolygonMesh::Ptr segmented_object = Triangulation::FullTriangulate(object);

					viewer.removePolygonMesh("model");
					viewer.addPolygonMesh(*segmented_object, "model");
				}
			}
			break;
		}
#pragma endregion t

#pragma region z
		case 'z':
		{
			char buffer[30];
			char * cloud_name = itoa(iObject, buffer, 10);
			cloud_name = strcat(cloud_name, "object");

			ObjectInsertion::TransformPointCloud(ObjectInsertion::object_library[iObject], ObjectInsertion::object_normals[iObject], -ObjectInsertion::object_normals[iObject], ObjectInsertion::object_standings[iObject], ObjectInsertion::object_standings[iObject]);

			if(model_type == 0)
			{
				viewer.removePointCloud(cloud_name);
				viewer.addPointCloud(ObjectInsertion::object_library[iObject], cloud_name);
			}
			else
			{
				ObjectInsertion::object_meshes[iObject]->cloud.data.clear();
				toROSMsg(*ObjectInsertion::object_library[iObject], ObjectInsertion::object_meshes[iObject]->cloud);
				viewer.removePolygonMesh(cloud_name);
				viewer.addPolygonMesh(*ObjectInsertion::object_meshes[iObject], cloud_name);
			}
			break;
		}
#pragma endregion z

#pragma region a
		case 'a':
		{
			char buffer[30];
			char * cloud_name = itoa(iObject, buffer, 10);
			cloud_name = strcat(cloud_name, "object");

			ObjectInsertion::RotatePointCloud(ObjectInsertion::object_library[iObject], ObjectInsertion::object_normals[iObject], 0.18, ObjectInsertion::object_standings[iObject]);

			if(model_type == 0)
			{
				viewer.removePointCloud(cloud_name);
				viewer.addPointCloud(ObjectInsertion::object_library[iObject], cloud_name);
			}
			else
			{ 
				ObjectInsertion::object_meshes[iObject]->cloud.data.clear();
				toROSMsg(*ObjectInsertion::object_library[iObject], ObjectInsertion::object_meshes[iObject]->cloud);
				viewer.removePolygonMesh(cloud_name);
				viewer.addPolygonMesh(*ObjectInsertion::object_meshes[iObject], cloud_name);
			}
			break;
		}
#pragma endregion a
		
#pragma region v
		case 'v':
		{			
			PointCloud<PointXYZRGB>::Ptr cutout_object(new PointCloud<PointXYZRGB>());
			for(size_t i = 0; i < clusters[1].indices.size(); ++i)
			{
				cutout_object->push_back(model->points[clusters[1].indices[i]]);
			}
			char buffer[30];
			srand(time(NULL));
			size_t i = rand();
			itoa(i, buffer, 10);
			char * filename = strcat(buffer, ".pcd");
			io::savePCDFile(filename, *cutout_object );

			break;
		}
#pragma endregion v

#pragma region d
		case 'd':
		{
/*			ObjectRemoval::RemoveObject(edited_model, clusters[1]);
			if(model_type == 0)
			{
				viewer.updatePointCloud(edited_model, "cloud");
			}
			else
			{
				mesh_model->cloud.data.clear();
				toROSMsg(*edited_model, mesh_model->cloud);
				viewer.removePolygonMesh("model");
				viewer.addPolygonMesh(*mesh_model, "model");
			}
*/
			vector<size_t> object_planes;
			vector<size_t> object_ground_planes;

//			ObjectRemoval::DetectObjectPlanes(MainProgram::edited_model, PlaneSegmentation::plane_index_list,PlaneSegmentation::plane_map, clusters[1], object_planes);
			time_t t1_groundplane = clock();
			ObjectRemoval::DetectObjectGroundPlanes(MainProgram::edited_model, PlaneSegmentation::plane_index_list, PlaneSegmentation::plane_map, PlaneSegmentation::coefficient_list, clusters[1], object_ground_planes);
			time_t runtime_groundplane = clock() - t1_groundplane;

			cout << endl << "Number of object planes = " << object_planes.size();
			cout << endl << "Number of object ground planes = " << object_ground_planes.size();
			cout << endl << "DetectObjectGroundPlanes time = " << (float)(runtime_groundplane) / CLOCKS_PER_SEC;

/*			copyPointCloud(*model, *edited_model);
			for(size_t i = 0; i < object_ground_planes.size(); ++i)
			{
				for(size_t j = 0; j < PlaneSegmentation::plane_index_list[object_ground_planes[i]].indices.size(); ++j)
				{
					edited_model->points[PlaneSegmentation::plane_index_list[object_ground_planes[i]].indices[j]].r = 255 - edited_model->points[PlaneSegmentation::plane_index_list[object_ground_planes[i]].indices[j]].r;
						edited_model->points[PlaneSegmentation::plane_index_list[object_ground_planes[i]].indices[j]].g = 255 - edited_model->points[PlaneSegmentation::plane_index_list[object_ground_planes[i]].indices[j]].g;
						edited_model->points[PlaneSegmentation::plane_index_list[object_ground_planes[i]].indices[j]].b = 255 - edited_model->points[PlaneSegmentation::plane_index_list[object_ground_planes[i]].indices[j]].b;
				}
			}*/

			vector<PointIndices> restored_indices;
			vector<PointCloud<PointXYZRGB>::Ptr> restored_points;

			time_t t1_removeObject = clock();
			ObjectRemoval::RemoveObject(MainProgram::edited_model, PlaneSegmentation::coefficient_list, PlaneSegmentation::plane_index_list, clusters[1], object_planes, object_ground_planes, restored_indices, restored_points);
			time_t runtime_removeObject = clock() - t1_removeObject;

			time_t t1_fusecolor = clock();
			ObjectRemoval::FuseColorToRestoredSurface(edited_model, restored_indices, restored_points);
			time_t runtime_fusecolor = clock() - t1_fusecolor;

			cout << endl << "RemoveObject time = " << (float)(runtime_removeObject) / CLOCKS_PER_SEC;
			cout << endl << "FuseColorToRestoredSurface time = " << (float)(runtime_fusecolor) / CLOCKS_PER_SEC;
			/*
			PointCloud<PointXYZRGB>::Ptr temp(new PointCloud<PointXYZRGB>());
			copyPointCloud(*MainProgram::edited_model, *temp);

			
			cout << endl << "Model size before filtering = " << temp->size() << endl;
			VoxelGrid<PointXYZRGB> grid;
			grid.setInputCloud(temp);
			grid.setLeafSize(0.01f, 0.01f, 0.01f);
			grid.filter(*MainProgram::edited_model);

			cout << endl << "Model size after filtering = " << MainProgram::edited_model->size() << endl;
			*/

			if(model_type == 0)
			{
				viewer.updatePointCloud(edited_model, "cloud");
			}
			else
			{
				/*
				mesh_model->cloud.data.clear();
				toROSMsg(*edited_model, mesh_model->cloud);
				viewer.removePolygonMesh("model");
				viewer.addPolygonMesh(*mesh_model, "model");
				*/
				PolygonMesh::Ptr new_mesh_model = Triangulation::FullTriangulate(edited_model);
				viewer.removePolygonMesh("model");
				viewer.addPolygonMesh(*new_mesh_model, "model");
			}

			break;
		}
#pragma endregion d
		case 'h': 
		{
			Helper();
			break;
		}
		default:
			break;
	}

	return;
}

void MainProgram::mouse_callback (const pcl::visualization::MouseEvent& event, void* cookie)
{

	if (event.getKeyboardModifiers() == 1)
	{
		PointFilter = (PointFilter + 1) % POINT_FILTER_LIMITATION;
		vtkRenderWindowInteractor *iwren = viewer.getInteractorStyle()->GetInteractor();
		float x = 0, y = 0, z = 0;
		int idx = MainProgram::performSinglePick (iwren, x, y, z);
		edited_model->points[idx].r = 255 - edited_model->points[idx].r;
		edited_model->points[idx].g = 255 - edited_model->points[idx].g;
		edited_model->points[idx].b = 255 - edited_model->points[idx].b;

		if (PointFilter == 1)
		{
			picked_points->indices.push_back(idx);
		}
	}
	else if (picked_points->indices.size() > 0 && event.getKeyboardModifiers() == 0 && PointFilter != -1)
	{
		PointFilter = -1;
		cout << endl << "List size = " << picked_points->indices.size() << endl;

		if(model_type == 0)
			viewer.updatePointCloud(edited_model, "cloud");
		else
		{
			mesh_model->cloud.data.clear();
			toROSMsg(*edited_model, mesh_model->cloud);
			viewer.removePolygonMesh("model");
			viewer.addPolygonMesh(*mesh_model, "model");
			cout << endl << "visualizer updated" << endl;
		}
	}
}

int MainProgram::performSinglePick (vtkRenderWindowInteractor *iren, float &x, float &y, float &z)
{
  int mouse_x, mouse_y;
  vtkPointPicker *picker = vtkPointPicker::SafeDownCast (iren->GetPicker ());
  
  if (!picker)
  {
    pcl::console::print_error ("Point picker not available, not selecting any points!\n");  
    return -1;
  }
  mouse_x = iren->GetEventPosition ()[0];
  mouse_y = iren->GetEventPosition ()[1];
  iren->StartPickCallback ();
  
  vtkRenderer *ren = iren->FindPokedRenderer (iren->GetEventPosition ()[0], iren->GetEventPosition ()[1]);
  picker->Pick (mouse_x, mouse_y, 0.0, ren);

  int idx = static_cast<int> (picker->GetPointId ());
  if (picker->GetDataSet () != NULL)
  {
    double p[3];
    picker->GetDataSet ()->GetPoint (idx, p);
    x = float (p[0]); y = float (p[1]); z = float (p[2]);
  }



  return (idx);
}