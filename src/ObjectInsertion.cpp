#include "ObjectInsertion.h"

vector<PointCloud<PointXYZRGB>::Ptr> ObjectInsertion::object_library;
vector<PolygonMesh::Ptr> ObjectInsertion::object_meshes;
vector<PointCloud<Normal>::Ptr> ObjectInsertion::object_normal_list;
vector<Vector3f> ObjectInsertion::object_normals;
vector<Vector3f> ObjectInsertion::object_standings;
vector<int> ObjectInsertion::n_transformations;

ObjectInsertion::ObjectInsertion(void)
{
}


ObjectInsertion::~ObjectInsertion(void)
{
}

void ObjectInsertion::LoadObjectList(vector<string> file_list)
{
	PointCloud<PointXYZRGB>::Ptr cloud;
	PointCloud<PointXYZRGB>::Ptr filtered_cloud;
	PointCloud<Normal>::Ptr normals;
	int file_list_size = file_list.size();
	for(size_t i = 0; i < file_list_size; ++i)
	{
		cloud.reset(new PointCloud<PointXYZRGB>());
		filtered_cloud.reset(new PointCloud<PointXYZRGB>());
		normals.reset(new PointCloud<Normal>());
		io::loadPCDFile<PointXYZRGB>(file_list[i], *cloud);

		cout << endl << "Object size before fitering = " << cloud->size() << endl;
		VoxelGrid<PointXYZRGB> grid2;
		grid2.setInputCloud(cloud);
		grid2.setLeafSize(0.01f, 0.01f, 0.01f);
		grid2.filter(*filtered_cloud);
		cout << endl << "Object size after filtering = " << filtered_cloud->size() << endl;

		object_library.push_back(filtered_cloud);

		normals = Triangulation::EstimateSurfaceNormal(filtered_cloud);
		
		//Assign point normals
		object_normal_list.push_back(normals);

		//Assign original normals
		Vector3f normal;
		normal << 0, 0, 1;
		object_normals.push_back(normal);

		//Assign original number of transformation taken
		n_transformations.push_back(0);

		//Assign the original standings
		Vector3f standing;
		standing << 0, 0, 0;
		object_standings.push_back(standing);
	}
}

void ObjectInsertion::TriangulateObjects()
{
	for(size_t i = 0; i < object_library.size(); ++i)
	{
		PolygonMesh::Ptr mesh_temp = Triangulation::Triangulate(object_library[i], object_normal_list[i]);
		object_meshes.push_back(mesh_temp);
	}
}

void ObjectInsertion::TransformPointCloud(PointCloud<PointXYZRGB>::Ptr cloud, Vector3f &normal_object, Vector3f normal_dest, Vector3f point, Vector3f source_point)
{
	// Get rotation angle
	float cos = normal_dest.dot(normal_object) / (normal_dest.norm() * normal_object.norm());
	if(cos > 1) cos = 1;
	if(cos < -1) cos = -1;
	float sin = sqrt(1 - cos * cos);
	Vector3f axis;
	Matrix<float, 3, 3> rotation_matrix;

	if(cos < 1 && cos > -1)
	{
		// rotation axis = plane normal X object normal
		axis = normal_object.cross(normal_dest);
		axis.normalize();
		float angle = acos(cos);

		Matrix<float, 3, 3> tensor_matrix;
		tensor_matrix << axis.x() * axis.x(), axis.x() * axis.y(), axis.x() * axis.z(),
						axis.x() * axis.y(), axis.y() * axis.y(), axis.y() * axis.z(),
						axis.x() * axis.z(), axis.y() * axis.z(), axis.z() * axis.z();

		Matrix<float, 3, 3> cross_product_matrix;
		cross_product_matrix << 0 ,-axis.z(), axis.y(),
								axis.z(), 0, -axis.x(),
								-axis.y(), axis.x(), 0;

		// Construct rotation matrix based on axis and angle
		rotation_matrix = Matrix3f::Identity(3, 3) * cos + sin * cross_product_matrix + (1 - cos) * tensor_matrix;
	}
	else if(cos == -1)
	{
		rotation_matrix << -1, 0, 0,
							0, -1, 0,
							0, 0, -1;
	}
	else
	{
		rotation_matrix << 1, 0, 0,
							0, 1, 0,
							0, 0, 1;
	}

	// Rotate the source point
	source_point = rotation_matrix * source_point;

	// Rotate the point cloud
	size_t size_cloud = cloud->size();
	for(size_t i = 0 ; i < size_cloud; ++i)
	{
		Vector3f point_coordinate;
		point_coordinate << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
		point_coordinate = rotation_matrix * point_coordinate;

		cloud->points[i].x = point_coordinate(0,0) + point(0,0) - source_point(0,0);
		cloud->points[i].y = point_coordinate(1,0) + point(1,0) - source_point(1,0);
		cloud->points[i].z = point_coordinate(2,0) + point(2,0) - source_point(2,0);
	}

	normal_object = normal_dest;
}

void ObjectInsertion::TransformPointCloud(PolygonMesh::Ptr mesh, Vector3f &normal_object, Vector3f normal_dest, PointXYZRGB point, PointXYZRGB source_point)
{
	time_t t1 = clock();

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());
	fromROSMsg(mesh->cloud, *cloud);
	
	time_t t2 = clock() - t1;

	cout << endl << "Time to convert PointCloud2 to PointCloud = " << (float) t2 / CLOCKS_PER_SEC << endl;

	// Get rotation angle
	float cos = normal_dest.dot(normal_object) / (normal_dest.norm() * normal_object.norm());
	if(cos >= 1) cos = 1;
	if(cos <= -1) cos = -1;
	float sin = sqrt(1 - cos * cos);
	Vector3f axis;

	if(cos < 1 && cos > -1)
		// rotation axis = plane normal X object normal
		axis = normal_object.cross(normal_dest);
	else
		axis.setIdentity();

	axis.normalize();

	float angle = acos(cos);

	Matrix<float, 3, 3> tensor_matrix;
	tensor_matrix << axis.x() * axis.x(), axis.x() * axis.y(), axis.x() * axis.z(),
					axis.x() * axis.y(), axis.y() * axis.y(), axis.y() * axis.z(),
					axis.x() * axis.z(), axis.y() * axis.z(), axis.z() * axis.z();

	Matrix<float, 3, 3> cross_product_matrix;
	cross_product_matrix << 0 ,-axis.z(), axis.y(),
							axis.z(), 0, -axis.x(),
							-axis.y(), axis.x(), 0;

	// Construct rotation matrix based on axis and angle
	Matrix<float, 3, 3> rotation_matrix = Matrix3f::Identity(3, 3) * cos + sin * cross_product_matrix + (1 - cos) * tensor_matrix;

	// Rotate the source point
	Vector3f source_coordinate;
	source_coordinate << source_point.x, source_point.y, source_point.z;
	source_coordinate = rotation_matrix * source_coordinate;
	source_point.x = source_coordinate(0,0);
	source_point.y = source_coordinate(1,0);
	source_point.z = source_coordinate(2,0);

	// Rotate the point cloud
	size_t size_cloud = cloud->size();
	for(size_t i = 0 ; i < size_cloud; ++i)
	{
		Vector3f point_coordinate;
		point_coordinate << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
		point_coordinate = rotation_matrix * point_coordinate;

		cloud->points[i].x = point_coordinate(0,0) + point.x - source_point.x;
		cloud->points[i].y = point_coordinate(1,0) + point.y - source_point.y;
		cloud->points[i].z = point_coordinate(2,0) + point.z - source_point.z;
	}

	time_t t3 = clock();
	toROSMsg(*cloud, mesh->cloud);
	time_t t4 = clock() - t3;
	cout << endl << "Time to convert PointCloud to PointCloud2 = " << (float) t4 / CLOCKS_PER_SEC << endl;

	normal_object = normal_dest;
}

void ObjectInsertion::RotatePointCloud(PointCloud<PointXYZRGB>::Ptr cloud, Vector3f &normal_object, float angle, Vector3f point)
{
	float cosine = cos(angle);
	float sine = sin(angle);
	Matrix<float, 3, 3> tensor_matrix;
	tensor_matrix << normal_object.x() * normal_object.x(), normal_object.x() * normal_object.y(), normal_object.x() * normal_object.z(),
					normal_object.x() * normal_object.y(), normal_object.y() * normal_object.y(), normal_object.y() * normal_object.z(),
					normal_object.x() * normal_object.z(), normal_object.y() * normal_object.z(), normal_object.z() * normal_object.z();

	Matrix<float, 3, 3> cross_product_matrix;
	cross_product_matrix << 0 ,-normal_object.z(), normal_object.y(),
							normal_object.z(), 0, -normal_object.x(),
							-normal_object.y(), normal_object .x(), 0;

	// Construct rotation matrix based on axis and angle
	Matrix<float, 3, 3> rotation_matrix = Matrix3f::Identity(3, 3) * cosine + sine * cross_product_matrix + (1 - cosine) * tensor_matrix;

	// Rotate the source point
	Vector3f source_coordinate;
	source_coordinate = point;
	source_coordinate = rotation_matrix * source_coordinate;
	float dis_x = source_coordinate(0,0) - point(0,0);
	float dis_y = source_coordinate(1,0) - point(1,0);
	float dis_z = source_coordinate(2,0) - point(2,0);

	// Rotate the point cloud
	size_t size_cloud = cloud->size();
	for(size_t i = 0 ; i < size_cloud; ++i)
	{
		Vector3f point_coordinate;
		point_coordinate << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
		point_coordinate = rotation_matrix * point_coordinate;

		cloud->points[i].x = point_coordinate(0,0) - dis_x;
		cloud->points[i].y = point_coordinate(1,0) - dis_y;
		cloud->points[i].z = point_coordinate(2,0) - dis_z;
	}
}