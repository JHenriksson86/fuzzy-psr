#ifndef FUZZYCLUSTERING3D_H
#define FUZZYCLUSTERING3D_H

#include <Eigen/Dense>

namespace fuzzy_psr
{
	class FuzzyClustering3D
	{
		private:
		Eigen::Matrix3Xd* 	points_;
		Eigen::Matrix3Xd* 	pruned_points_;
		Eigen::Matrix3Xd 	clusters_;
		Eigen::MatrixXd 	partition_matrix_;
		Eigen::MatrixXd 	partition_squared_;
		Eigen::Matrix<double, -1, -1, Eigen::RowMajor> 	distance_matrix_;
		int 				number_of_points_;
		int 				number_of_points_after_prune_;
		int 				number_of_clusters_;
		int 				max_iter_;
		bool 				noisy_;
		double 				prune_ratio_;
		double 				objective_cost_;
		double 				old_objective_cost_;
		double 				min_difference_;
		
		public:
		FuzzyClustering3D();
		FuzzyClustering3D(Eigen::Matrix3Xd* points, int number_of_clusters, 
			int max_iterations = 100, bool noisy = false, double prune_ratio = 0.15);
		~FuzzyClustering3D();
		double runClustering(); // return AFPCD	
		const Eigen::Matrix3Xd& getClusters() const;

		private:
		void initialization();
		void updateClusterCenters(); // update fuzzy cluster centers
		void updateDistanceMatrix(); // update distance matrix
		void updatePartitionMatrix(); // update fuzzy membership grades
		double calculateObjectiveFunction() const;
	};
};

#endif

