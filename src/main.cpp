#include "MainProgram.h"
#include "ObjectInsertion.h"
#include "ClusteringBasedSegmentation.h"
#include "GraphBasedSegmentation.h"
#include "PlaneSegmentation.h"
#include "Triangulation.h"
	
using namespace std;
using namespace pcl;
using namespace Eigen;

int main(int argc, char* argv[])
{
	//Reading filename from command line arguments
	string filename;
	int is_colored_cloud = 1;

	// Initializing the materials for the main program
	MainProgram::Initialize();
	
	if(argc > 0)
	{
		filename = argv[1];
		if(argc > 2)
			MainProgram::model_type = atoi(argv[2]);
	}
	else
	{
		cout << "No file to open. Program is terminated!" << endl;
		return -1;
	}

	// Temporaty point cloud to store the raw data from file
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());
	PointCloud<PointXYZRGB>::Ptr cloud_raw(new PointCloud<PointXYZRGB>());
	PointCloud<PointXYZRGB>::Ptr object_raw(new PointCloud<PointXYZRGB>());

	cout << "			Loading data ... " << endl;

	//load pcd file
	if(io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud_raw) == -1)
	{
		cout << "Could not load the file !" << endl;
		cin.get();
		return -1;
	}

	size_t i1 = cloud_raw->points[0].rgba;
	size_t i2 = cloud_raw->points[1].rgba;
	size_t i3 = cloud_raw->points[2].rgba;
	size_t cons = 4278190080;

	if(i1 == cons && i2 == cons && i3 == cons)
		is_colored_cloud = 0;

	cout << "			Applying voxel grid filter ... " << endl;

	//Filter model	
	cout << endl << "Model size before filtering = " << cloud_raw->size() << endl;
	VoxelGrid<PointXYZRGB> grid;
	grid.setInputCloud(cloud_raw);
	grid.setLeafSize(0.01f, 0.01f, 0.01f);
	grid.filter(*MainProgram::model);

	cout << endl << "Model size after filtering = " << MainProgram::model->size() << endl;

	//Copy point cloud
	copyPointCloud(*MainProgram::model, *MainProgram::edited_model);

	//load object
	vector<string> object_filename;
	object_filename.push_back("object_bag.pcd");
	object_filename.push_back("object_chair.pcd");
	object_filename.push_back("object_guitar_normalized.pcd");
	object_filename.push_back("object_organ_normalized.pcd");
	ObjectInsertion::LoadObjectList(object_filename);
	
	cloud->resize(MainProgram::model->size());

	// make a copy of source data to process
	size_t size_cloud = MainProgram::model->size();
	copyPointCloud(*MainProgram::model, *cloud);

	PointIndices::Ptr indices;

	//Estimate surface normals
	cout << endl << "			Estimating Surface Normals ... " << endl;
	time_t t_normal_1 = clock();
	MainProgram::normal_model = Triangulation::EstimateSurfaceNormal(MainProgram::model);
	time_t t_normal_2 = clock() - t_normal_1;
	cout << endl << "Normal calculation time = " << (float) t_normal_2 / CLOCKS_PER_SEC << endl;

	cout << "Start detecting planes ..." << endl;
	time_t t1 = clock();

	// Segment planes
	PlaneSegmentation::GetAllPlanes(cloud, indices, 0.03, PlaneSegmentation::plane_map, PlaneSegmentation::plane_index_list, PlaneSegmentation::coefficient_list);
	//PlaneSegmentation::GetAllPlanesByNormal(cloud, MainProgram::normal_model, indices, 0.03, PlaneSegmentation::plane_map, PlaneSegmentation::plane_index_list, PlaneSegmentation::coefficient_list);
	PlaneSegmentation::DetermineGroundPlanes(MainProgram::model);
	
	time_t runtime = clock() - t1;
	cout << endl << "There are " << PlaneSegmentation::plane_index_list.size() << " planes";
	cout << " and " << PlaneSegmentation::coefficient_list.size() << " equations" << endl;
	cout << endl << "Plane segmentation time = " << (float)(runtime / CLOCKS_PER_SEC) << " seconds" << endl;
	cout << endl << "Number Of Ground Planes = " << PlaneSegmentation::ground_plane_list.size() << endl;

/*	for(size_t i = 0; i < PlaneSegmentation::ground_plane_list.size(); ++i)
	{
		for(size_t j = 0; j < PlaneSegmentation::plane_index_list[PlaneSegmentation::ground_plane_list[i]].indices.size(); ++j)
		{
			MainProgram::model->points[PlaneSegmentation::plane_index_list[PlaneSegmentation::ground_plane_list[i]].indices[j]].r = 
				MainProgram::model->points[PlaneSegmentation::plane_index_list[PlaneSegmentation::ground_plane_list[i]].indices[j]].g = 
				MainProgram::model->points[PlaneSegmentation::plane_index_list[PlaneSegmentation::ground_plane_list[i]].indices[j]].b = 50;
		}
	}
*/
	// Filling color to XYZ cloud (optional)
	if(is_colored_cloud == 0)
		MainProgram::FillColor2Cloud(MainProgram::model, PlaneSegmentation::plane_index_list);	

	//Constructing triangle mesh
	if(MainProgram::model_type != 0)
	{
	
		cout << endl << "			Constructing triangle meshes ... " << endl;

		time_t t_mesh1 = clock();
		MainProgram::mesh_model = Triangulation::Triangulate(MainProgram::model, MainProgram::normal_model);
		time_t t_mesh2 = clock() - t_mesh1;
		cout << endl << "Model triangulation time = " << (float) t_mesh2 / CLOCKS_PER_SEC << endl;

		t_mesh1 = clock();
		ObjectInsertion::TriangulateObjects();
		t_mesh2 = clock() - t_mesh1;
		cout << endl << "Object triangulation time = " << (float) t_mesh2 / CLOCKS_PER_SEC << endl;	
	
		cout << endl << "			Start visualizing ... " << endl;

		MainProgram::viewer.addPolygonMesh(*MainProgram::mesh_model, "model");

	}
	else
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(MainProgram::model);
		MainProgram::viewer.addPointCloud<PointXYZRGB>(MainProgram::model, rgb, "cloud");
	}
	
	
	//Visualize mesh
	MainProgram::viewer.registerPointPickingCallback(&MainProgram::pp_callback, static_cast<void*> (&MainProgram::model));
	MainProgram::viewer.registerKeyboardCallback(&MainProgram::kb_callback, static_cast<void*> (&MainProgram::model));	

	MainProgram::viewer.getCameras(MainProgram::camera);
	MainProgram::viewer.registerMouseCallback(&MainProgram::mouse_callback, static_cast<void*> (&MainProgram::model));
	MainProgram::viewer.initCameraParameters ();
	while(! MainProgram::viewer.wasStopped()) {
		MainProgram::viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100));	
	}

	return 0;
}