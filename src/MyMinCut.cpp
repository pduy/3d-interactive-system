#include "MyMinCut.h"

MyMinCut::MyMinCut(void) :
	input_normal_(new pcl::PointCloud<Normal>),
	inverse_sigma_color_ (16.0),
	inverse_sigma_euclidean_ (16.0),
	binary_potentials_are_valid_ (false),
	epsilon_ (0.0001),
	radius_ (16.0),
	unary_potentials_are_valid_ (false),
	source_weight_ (0.8),
	search_ (),
	number_of_neighbours_ (14),
	graph_is_valid_ (false),
	clusters_ (0),
	graph_ (),
	capacity_ (),
	reverse_edges_ (),
	vertices_ (0),
	edge_marker_ (0),
	source_ (),/////////////////////////////////
	sink_ (),///////////////////////////////////
	max_flow_ (0.0),
	color_distance_ratio (0.0),
	PRIORITY_RATE (0.05)
{
}


MyMinCut::~MyMinCut(void)
{
	if (search_ != 0)
	search_.reset ();
	if (graph_ != 0)
	graph_.reset ();
	if (capacity_ != 0)
	capacity_.reset ();
	if (reverse_edges_ != 0)
	reverse_edges_.reset ();

	foreground_points_.indices.clear ();
	background_points_.indices.clear ();
	clusters_.clear ();
	vertices_.clear ();
	edge_marker_.clear ();
	input_normal_->clear();
}

void MyMinCut::setInputCloud (const PointCloud::Ptr &cloud)
{
  input_ = cloud;
  graph_is_valid_ = false;
  unary_potentials_are_valid_ = false;
  binary_potentials_are_valid_ = false;
  calculateColorDistanceRatio();
}

void MyMinCut::setInputNormal(pcl::PointCloud<Normal>::Ptr normals)
{
	input_normal_ = normals;
}

void
MyMinCut::setSigma (double sigma_color, double sigma_euclidean)
{
  if (sigma_color > epsilon_)
  {
    inverse_sigma_color_ = 1.0 / (sigma_color * sigma_color);
	inverse_sigma_euclidean_ = 1.0 / (sigma_euclidean * sigma_euclidean);
    binary_potentials_are_valid_ = false;
  }
}

 void
MyMinCut::setRadius (double radius)
{
  if (radius > epsilon_)
  {
    radius_ = radius * radius;
    unary_potentials_are_valid_ = false;
  }
}

 void
MyMinCut::setSourceWeight (double weight)
{
  if (weight > epsilon_)
  {
    source_weight_ = weight;
    unary_potentials_are_valid_ = false;
  }
}

 void
MyMinCut::setSearchMethod (const search::KdTree<PointXYZRGB>::Ptr& tree)
{
  if (search_ != 0)
    search_.reset ();

  search_ = tree;
}

 void
MyMinCut::setNumberOfNeighbours (unsigned int neighbour_number)
{
  if (number_of_neighbours_ != neighbour_number && neighbour_number != 0)
  {
    number_of_neighbours_ = neighbour_number;
    graph_is_valid_ = false;
    unary_potentials_are_valid_ = false;
    binary_potentials_are_valid_ = false;
  }
}

 void
MyMinCut::setForegroundPoints ( PointIndices foreground_points)
{
	foreground_points_.indices.clear ();
  foreground_points_ = foreground_points;

  unary_potentials_are_valid_ = false;
}

 void
MyMinCut::setBackgroundPoints (PointIndices background_points)
{
  background_points_.indices.clear ();
  background_points_ = background_points;

  unary_potentials_are_valid_ = false;
}

 void
MyMinCut::extract (std::vector <pcl::PointIndices>& clusters)
{
  clusters.clear ();

  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  if ( graph_is_valid_ && unary_potentials_are_valid_ && binary_potentials_are_valid_ )
  {
    clusters.reserve (clusters_.size ());
    std::copy (clusters_.begin (), clusters_.end (), std::back_inserter (clusters));
    deinitCompute ();
    return;
  }

  clusters_.clear ();
  bool success = true;

  if ( !graph_is_valid_ )
  {
    success = buildGraph ();
    if (success == false)
    {
      deinitCompute ();
      return;
    }
    graph_is_valid_ = true;
    unary_potentials_are_valid_ = true;
    binary_potentials_are_valid_ = true;
  }

  if ( !unary_potentials_are_valid_ )
  {
    success = recalculateUnaryPotentials ();
    if (success == false)
    {
      deinitCompute ();
      return;
    }
    unary_potentials_are_valid_ = true;
  }

  if ( !binary_potentials_are_valid_ )
  {
    success = recalculateBinaryPotentials ();
    if (success == false)
    {
      deinitCompute ();
      return;
    }
    binary_potentials_are_valid_ = true;
  }

  //IndexMap index_map = boost::get (boost::vertex_index, *graph_);
  ResidualCapacityMap residual_capacity = boost::get (boost::edge_residual_capacity, *graph_);

  max_flow_ = boost::boykov_kolmogorov_max_flow (*graph_, source_, sink_);

  assembleLabels (residual_capacity);

  deinitCompute ();
}

 bool
MyMinCut::buildGraph ()
{
  int number_of_points = static_cast<int> (input_->points.size ());
  int number_of_indices = static_cast<int> (indices_->size ());

  if (input_->points.size () == 0 || number_of_points == 0 || foreground_points_.indices.empty() == true )
    return (false);

  if (search_ == 0)
    search_ = boost::shared_ptr<pcl::search::KdTree<PointXYZRGB> > (new pcl::search::KdTree<PointXYZRGB>);

  graph_.reset ();
  graph_ = boost::shared_ptr< mGraph > (new mGraph ());

  capacity_.reset ();
  capacity_ = boost::shared_ptr<CapacityMap> (new CapacityMap ());
  *capacity_ = boost::get (boost::edge_capacity, *graph_);

  reverse_edges_.reset ();
  reverse_edges_ = boost::shared_ptr<ReverseEdgeMap> (new ReverseEdgeMap ());
  *reverse_edges_ = boost::get (boost::edge_reverse, *graph_);

  VertexDescriptor vertex_descriptor(0);
  vertices_.clear ();
  vertices_.resize (number_of_points + 2, vertex_descriptor);

  std::set<int> out_edges_marker;
  edge_marker_.clear ();
  edge_marker_.resize (number_of_points + 2, out_edges_marker);

  for (int i_point = 0; i_point < number_of_points + 2; i_point++)
    vertices_[i_point] = boost::add_vertex (*graph_);

  source_ = vertices_[number_of_points];
  sink_ = vertices_[number_of_points + 1];

  for (int i_point = 0; i_point < number_of_indices; i_point++)
  {
    int point_index = (*indices_)[i_point];
    double source_weight = 0.0;
    double sink_weight = 0.0;
    calculateUnaryPotential (point_index, source_weight, sink_weight);
    addEdge (static_cast<int> (source_), point_index, source_weight);
    addEdge (point_index, static_cast<int> (sink_), sink_weight);
  }

  std::vector<int> neighbours;
  std::vector<float> distances;
  search_->setInputCloud (input_, indices_);
  for (int i_point = 0; i_point < number_of_indices; i_point++)
  {
    int point_index = (*indices_)[i_point];
    search_->nearestKSearch (i_point, number_of_neighbours_, neighbours, distances);
    for (size_t i_nghbr = 1; i_nghbr < neighbours.size (); i_nghbr++)
    {
      double weight = calculateBinaryPotential (point_index, neighbours[i_nghbr]);
      addEdge (point_index, neighbours[i_nghbr], weight);
      addEdge (neighbours[i_nghbr], point_index, weight);
    }
    neighbours.clear ();
    distances.clear ();
  }

  return (true);
}

 void
MyMinCut::calculateUnaryPotential (int point, double& source_weight, double& sink_weight) const
{
	double min_dist_to_foreground = std::numeric_limits<double>::max ();
	double min_dist_to_background = std::numeric_limits<double>::max ();
	//  double closest_foreground_point[3];
	//  closest_foreground_point[0] = closest_foreground_point[1] = closest_foreground_point[2] = 0;
	//  double closest_background_point[3];
	//  closest_background_point[0] = closest_background_point[1] = closest_background_point[2] = 0;
	double initial_point[] = {0.0, 0.0, 0.0};
	
	initial_point[0] = input_->points[point].x;
	initial_point[1] = input_->points[point].y;
	initial_point[2] = input_->points[point].z;

	size_t size_foreground = foreground_points_.indices.size();
	for (size_t i_point = 0; i_point < size_foreground; i_point++)
	{
		PointXYZRGB cur_point = input_->points[foreground_points_.indices[i_point]];
		double dist = inverse_sigma_euclidean_ * 
				((cur_point.x - initial_point[0]) * (cur_point.x- initial_point[0])
				+ (cur_point.y - initial_point[1]) * (cur_point.y - initial_point[1])
				+ (cur_point.z - initial_point[2]) * (cur_point.z - initial_point[2]));
		if (min_dist_to_foreground > dist)
		{
			min_dist_to_foreground = dist;
		//    closest_foreground_point[0] = foreground_points_[i_point].x;
		//    closest_foreground_point[1] = foreground_points_[i_point].y;
		//	  closest_foreground_point[2] = foreground_points_[i_point].z
		}
	}

	//  sink_weight = pow (min_dist_to_foreground / radius_, 0.5);

	//  source_weight = source_weight_;
	//  return;

	if (background_points_.indices.size () == 0)
	{
		sink_weight = sqrt(min_dist_to_foreground / radius_);
		source_weight = source_weight_;
		return;
	}
	  
	size_t size_background = background_points_.indices.size ();
	for (int i_point = 0; i_point < size_background; i_point++)
	{
		PointXYZRGB cur_point = input_->points[background_points_.indices[i_point]];
		double dist = inverse_sigma_euclidean_ * 
				((cur_point.x - initial_point[0]) * (cur_point.x - initial_point[0])
				+ (cur_point.y - initial_point[1]) * (cur_point.y - initial_point[1])
				+ (cur_point.z - initial_point[2]) * (cur_point.z - initial_point[2]));
		if (min_dist_to_background > dist)
		{
			min_dist_to_background = dist;
			//      closest_background_point[0] = background_points_[i_point].x;
			//     closest_background_point[1] = background_points_[i_point].y;
			//	  closest_background_point[2] = background_points_[i_point].z;
		}
	}

	if (min_dist_to_background <= epsilon_)
	{
		source_weight = 0.0;
		sink_weight = 1.0;
		return;
	}
	if(min_dist_to_foreground <= epsilon_)
	{
		source_weight = 1.0;
		sink_weight = 0.0;
		return;
	}

	sink_weight = 1.0 / (1.0 + sqrt(min_dist_to_background / min_dist_to_foreground));
	source_weight = 1.0 - sink_weight;
	//  source_weight = 1.0 / (1.0 + pow (min_dist_to_background / min_dist_to_foreground, 0.5));
	//  sink_weight = 1 - source_weight;
}

 bool
MyMinCut::addEdge (int source, int target, double weight)
{
  std::set<int>::iterator iter_out = edge_marker_[source].find (target);
  if ( iter_out != edge_marker_[source].end () )
    return (false);

  EdgeDescriptor edge;
  EdgeDescriptor reverse_edge;
  bool edge_was_added, reverse_edge_was_added;

  boost::tie (edge, edge_was_added) = boost::add_edge ( vertices_[source], vertices_[target], *graph_ );
  boost::tie (reverse_edge, reverse_edge_was_added) = boost::add_edge ( vertices_[target], vertices_[source], *graph_ );
  if ( !edge_was_added || !reverse_edge_was_added )
    return (false);

  (*capacity_)[edge] = weight;
  (*capacity_)[reverse_edge] = 0.0;
  (*reverse_edges_)[edge] = reverse_edge;
  (*reverse_edges_)[reverse_edge] = edge;
  edge_marker_[source].insert (target);

  return (true);
}

 double
MyMinCut::calculateBinaryPotential (int source, int target) const
{
//  distance *= inverse_sigma_color_;
//  weight = exp (-distance);
	double dist = inverse_sigma_color_ *
					((input_->points[source].r - input_->points[target].r) * (input_->points[source].r - input_->points[target].r)
				+ (input_->points[source].g - input_->points[target].g) * (input_->points[source].g - input_->points[target].g)
				+ (input_->points[source].b - input_->points[target].b) * (input_->points[source].b - input_->points[target].b));
				
/*	double dist = inverse_sigma_euclidean_ *
					((input_->points[source].x - input_->points[target].x) * (input_->points[source].x - input_->points[target].x)
				+ (input_->points[source].y - input_->points[target].y) * (input_->points[source].y - input_->points[target].y)
				+ (input_->points[source].z - input_->points[target].z) * (input_->points[source].z - input_->points[target].z));*/

//	weight = 1 / (1 + dist);

	return (exp (-dist));
}

 bool
MyMinCut::recalculateUnaryPotentials ()
{
  OutEdgeIterator src_edge_iter;
  OutEdgeIterator src_edge_end;
  std::pair<EdgeDescriptor, bool> sink_edge;

  for (boost::tie (src_edge_iter, src_edge_end) = boost::out_edges (source_, *graph_); src_edge_iter != src_edge_end; src_edge_iter++)
  {
    double source_weight = 0.0;
    double sink_weight = 0.0;
    sink_edge.second = false;
    calculateUnaryPotential (static_cast<int> (boost::target (*src_edge_iter, *graph_)), source_weight, sink_weight);
    sink_edge = boost::lookup_edge (boost::target (*src_edge_iter, *graph_), sink_, *graph_);
    if (!sink_edge.second)
      return (false);

    (*capacity_)[*src_edge_iter] = source_weight;
    (*capacity_)[sink_edge.first] = sink_weight;
  }

  return (true);
}

 bool
MyMinCut::recalculateBinaryPotentials ()
{
  int number_of_points = static_cast<int> (indices_->size ());

  VertexIterator vertex_iter;
  VertexIterator vertex_end;
  OutEdgeIterator edge_iter;
  OutEdgeIterator edge_end;

  std::vector< std::set<VertexDescriptor> > edge_marker;
  std::set<VertexDescriptor> out_edges_marker;
  edge_marker.clear ();
  edge_marker.resize (number_of_points + 2, out_edges_marker);

  for (boost::tie (vertex_iter, vertex_end) = boost::vertices (*graph_); vertex_iter != vertex_end; vertex_iter++)
  {
    VertexDescriptor source_vertex = *vertex_iter;
    if (source_vertex == source_ || source_vertex == sink_)
      continue;
    for (boost::tie (edge_iter, edge_end) = boost::out_edges (source_vertex, *graph_); edge_iter != edge_end; edge_iter++)
    {
      //If this is not the edge of the graph, but the reverse fictitious edge that is needed for the algorithm then continue
      EdgeDescriptor reverse_edge = (*reverse_edges_)[*edge_iter];
      if ((*capacity_)[reverse_edge] != 0.0)
        continue;

      //If we already changed weight for this edge then continue
      VertexDescriptor target_vertex = boost::target (*edge_iter, *graph_);
      std::set<VertexDescriptor>::iterator iter_out = edge_marker[static_cast<int> (source_vertex)].find (target_vertex);
      if ( iter_out != edge_marker[static_cast<int> (source_vertex)].end () )
        continue;

      if (target_vertex != source_ && target_vertex != sink_)
      {
        //Change weight and remember that this edges were updated
        double weight = calculateBinaryPotential (static_cast<int> (target_vertex), static_cast<int> (source_vertex));
        (*capacity_)[*edge_iter] = weight;
        edge_marker[static_cast<int> (source_vertex)].insert (target_vertex);
      }
    }
  }

  return (true);
}

 void
MyMinCut::assembleLabels (ResidualCapacityMap& residual_capacity)
{
  std::vector<int> labels;
  labels.resize (input_->points.size (), 0);
  int number_of_indices = static_cast<int> (indices_->size ());
  for (int i_point = 0; i_point < number_of_indices; i_point++)
    labels[(*indices_)[i_point]] = 1;

  clusters_.clear ();

  pcl::PointIndices segment;
  clusters_.resize (2, segment);

  OutEdgeIterator edge_iter, edge_end;
  for ( boost::tie (edge_iter, edge_end) = boost::out_edges (source_, *graph_); edge_iter != edge_end; edge_iter++ )
  {
    if (labels[edge_iter->m_target] == 1)
    {
      if (residual_capacity[*edge_iter] > epsilon_)
        clusters_[1].indices.push_back (static_cast<int> (edge_iter->m_target));
      else
        clusters_[0].indices.push_back (static_cast<int> (edge_iter->m_target));
    }
  }
}

 pcl::PointCloud<pcl::PointXYZRGB>::Ptr
MyMinCut::getColoredCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters_.empty ())
  {
    int num_of_pts_in_first_cluster = static_cast<int> (clusters_[0].indices.size ());
    int num_of_pts_in_second_cluster = static_cast<int> (clusters_[1].indices.size ());
    int number_of_points = num_of_pts_in_first_cluster + num_of_pts_in_second_cluster;
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
    unsigned char foreground_color[3] = {255, 255, 255};
    unsigned char background_color[3] = {255, 0, 0};
    colored_cloud->width = number_of_points;
    colored_cloud->height = 1;
    colored_cloud->is_dense = input_->is_dense;

    pcl::PointXYZRGB point;
    int point_index = 0;
    for (int i_point = 0; i_point < num_of_pts_in_first_cluster; i_point++)
    {
      point_index = clusters_[0].indices[i_point];
      point.x = *(input_->points[point_index].data);
      point.y = *(input_->points[point_index].data + 1);
      point.z = *(input_->points[point_index].data + 2);
      point.r = background_color[0];
      point.g = background_color[1];
      point.b = background_color[2];
      colored_cloud->points.push_back (point);
    }

    for (int i_point = 0; i_point < num_of_pts_in_second_cluster; i_point++)
    {
      point_index = clusters_[1].indices[i_point];
      point.x = *(input_->points[point_index].data);
      point.y = *(input_->points[point_index].data + 1);
      point.z = *(input_->points[point_index].data + 2);
      point.r = foreground_color[0];
      point.g = foreground_color[1];
      point.b = foreground_color[2];
      colored_cloud->points.push_back (point);
    }
  }

  return (colored_cloud);
}

double MyMinCut::getColorDistanceRatio()
{
	return color_distance_ratio;
}

void MyMinCut::setColorDistanceRatio(double ratio)
{
	color_distance_ratio = ratio;
}

void MyMinCut::calculateColorDistanceRatio()
{
	double point1[] = {0.0, 0.0, 0.0};
	double point2[] = {0.0, 0.0, 0.0};

	point1[0] = input_->points[0].x;
	point1[1] = input_->points[0].y;
	point1[2] = input_->points[0].z;

	point2[0] = input_->points[1].x;
	point2[1] = input_->points[1].y;
	point2[2] = input_->points[1].z;

	double average_distance = (point1[0] - point2[0]) * (point1[0] - point2[0])
							+ (point1[1] - point2[1]) * (point1[1] - point2[1])
							+ (point1[2] - point2[2]) * (point1[2] - point2[2]);

	setColorDistanceRatio((average_distance/1)/ MyMinCut::PRIORITY_RATE);
}