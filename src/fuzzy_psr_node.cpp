#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>


Eigen::MatrixXd loadClusterFile(const std::string &fileName) {

  Eigen::MatrixXd ret;

  std::ifstream ifs;
  ifs.open(fileName.c_str());
  if (!ifs.is_open()) {
    std::cerr << __FILE__ << ":" << __LINE__ << " cannot open file : " << fileName << std::endl;
  }

  unsigned int nb_centers;

  std::vector<double> tmp; // Container to 
  
  while (!ifs.eof())
    {
      std::string line;
      getline(ifs, line);

      Eigen::Vector3d center;
      if (sscanf(line.c_str(), "%lf %lf %lf",
		 &center(0), &center(1), &center(2)) == 3)
	{
	  tmp.push_back(center(0));
	  tmp.push_back(center(1));
	  tmp.push_back(center(2));
	  nb_centers++;
	}
    }
  return Eigen::MatrixXd::Map(tmp.data(), 3, nb_centers);
}

class FuzzyPSRNode {

private:

  ros::NodeHandle nh_;

  ros::Subscriber sub_point_cloud_; // Input point cloud - merged with the table top removed.
  ros::Publisher pub_poses_;      // Publish poses of the object - should probably have another message type incase you want also to know what object you are seeing (maybe also add some measures of uncertainty as well).

  Eigen::MatrixXd fuzzy_centers_; // Contains the object data base - fuzzy centers of known objects.
  
public:
  FuzzyPSRNode(ros::NodeHandle &paramHandle) {


    sub_point_cloud_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/merged_point_cloud",10,&FuzzyPSRNode::process_point_cloud, this);
    pub_poses_ = nh_.advertise<geometry_msgs::PoseArray>("object_poses", 1000);

    std::string filename;
    paramHandle.param("filename", filename, std::string("objects.txt"));

    ROS_INFO_STREAM("filename : " << filename);

    // Load the object files, note; the types and number of object will not change during execution.
    if (!filename.empty()) {
      // try to load the file
      try {
	      fuzzy_centers_ = loadClusterFile(filename);
      }
      catch (...) {
	
      }
    }
  }
  
  void process_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg) {

    // Convert to matrix format
    Eigen::MatrixXd eigen_pts;
    //eigen_pts = PointCloud::getMatrixXfMap(); // Convert directly to Eigen format from pcl.
    
    // Do registration.

    //    object_poses = registration(fuzzy_centers_, eigen_pts);
  
    //    pub_poses_.publish(object_poses);
  }


};


int main(int argc, char** argv) {

  ros::init(argc,argv,"fuzzy-psr_node");
  ros::NodeHandle params ("~");

  FuzzyPSRNode n(params);


  ros::spin();

}


