#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <time.h>
#include "fuzzy_clustering_3d.h"

using namespace Eigen;
using namespace std;
using namespace fuzzy_psr;

#define DEFAULT_MODEL_FNAME "model.txt"

int loadPointCloud(string FName, int & N_P, Matrix3Xd ** PointCloud);

int main(int argc, char** argv)
{
	ros::init(argc,argv,"fuzzy_psr_clustering_test");
  	ros::NodeHandle nh();

	string filepath = "";
   	ros::param::get("/fuzzy_psr_clustering_test/filepath", filepath);

	string filename = "";
   	ros::param::get("/fuzzy_psr_clustering_test/filename", filename);

	int N_C = 30, N_P;
	double AFPCD;
	bool Noisy = 0;
	Matrix3Xd *P;
	clock_t  clockBegin, clockEnd;

	cout << "Loading pointcloud from: " << filepath + filename << "\n";
	loadPointCloud(filepath + filename, N_P, &P);
	cout << "Number of points loaded = " << endl << N_P << "\n";

	FuzzyClustering3D fc3D(P, N_C);

	cout << "Starting clustering..\n";
	clockBegin = clock();
	AFPCD = fc3D.runClustering();
	clockEnd = clock();
	cout << "Clustering finished.\n";

	cout << "C =" << endl << fc3D.getClusters() << endl;
	cout << "AFPCD =" << endl << AFPCD << endl;
	cout << "Time cost is " << (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC << " s" << endl;

	return 0;
}


int loadPointCloud(string FName, int & N_P, Matrix3Xd ** PointCloud)
{
	ifstream ifile;

	ifile.open(FName.c_str(), ifstream::in);
	if(!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}
	ifile >> N_P; // First line has number of points to follow
	*PointCloud = new Matrix3Xd;
	(*PointCloud)->resize(3, N_P);

	for(int i = 0; i < N_P; i++)
	{
		ifile >> (**PointCloud)(0,i) >> (**PointCloud)(1,i) >> (**PointCloud)(2,i);
	}

	ifile.close();

	return 0;
}
