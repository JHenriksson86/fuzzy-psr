#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#include "fuzzy_clustering_3d.h"

using namespace Eigen;
using std::cout;

namespace fuzzy_psr
{
	FuzzyClustering3D::FuzzyClustering3D()
	{
		this->points_ = nullptr;
		this->pruned_points_ = nullptr;
		this->number_of_points_ = 0;
		this->number_of_points_after_prune_ = 0;
		this->number_of_clusters_ = 0;
		this->noisy_ = false;
		this->prune_ratio_ = 0.15;
		this->objective_cost_ = 0.0;
		this->old_objective_cost_ = 0.0;
		this->min_difference_ = 0.1;
	}

	FuzzyClustering3D::FuzzyClustering3D(
		Eigen::Matrix3Xd* points, int number_of_clusters, int max_iterations,
		bool noisy, double prune_ratio)
	{
		this->points_ = points;
		this->number_of_points_ = points->cols();

		this->pruned_points_ = nullptr;
		this->number_of_points_after_prune_ = 0;
		
		this->clusters_ = Matrix3Xd::Zero(3, number_of_clusters);
		this->number_of_clusters_ = number_of_clusters;

		this->noisy_ = noisy;
		this->max_iter_ = max_iterations;
		this->prune_ratio_ = prune_ratio;
		this->objective_cost_ = 0.0;
		this->old_objective_cost_ = 0.0;
		this->min_difference_ = 0.1;
	}


	FuzzyClustering3D::~FuzzyClustering3D(){}

	double FuzzyClustering3D::runClustering()
	{
		initialization();
		for(int i = 0;i < max_iter_; i++)
		{
			updateClusterCenters();
			updateDistanceMatrix();
			old_objective_cost_ = objective_cost_;
			objective_cost_ = calculateObjectiveFunction();
			updatePartitionMatrix();

			double difference = std::abs(objective_cost_-old_objective_cost_);
			cout << "Clustering iter=" << i << ", difference=" << difference << "\n";
			if (difference <= min_difference_)
			{
				break;
			}
		}
		return objective_cost_/number_of_points_; // AFPCD
	}

	const Eigen::Matrix3Xd& FuzzyClustering3D::getClusters() const { return clusters_; }

	// ### Private functions ###

	void FuzzyClustering3D::initialization()
	{
		cout << "Initializing variables..\n";
		if ((points_ == nullptr) || (number_of_points_ == 0))
		{
			throw std::logic_error("ERROR: the point cloud is not defined");
		}
		if (noisy_)
		{
			//TODO
			this->number_of_points_after_prune_ = 
				int(double(number_of_points_)*(1.0-prune_ratio_));
			this->pruned_points_ = new Matrix3Xd();
			pruned_points_->setZero(3, number_of_points_after_prune_);
		}
		if (number_of_clusters_ <= 0)
		{
			throw std::logic_error("ERROR: wrong setting for the number of clusters");
		}

		srand((unsigned int)(time(NULL)));

		partition_matrix_.setRandom(number_of_clusters_, number_of_points_).array().abs();
		RowVectorXd fm_colsum = partition_matrix_.colwise().sum();
		for(int i = 0; i < number_of_clusters_; i++)
		{
			partition_matrix_.row(i) = 
				partition_matrix_.row(i).cwiseQuotient(fm_colsum).array().square();
		}

		distance_matrix_.setZero(number_of_clusters_, number_of_points_);
		cout << "Initialization successful!\n";
	}

	void FuzzyClustering3D::updateClusterCenters()
	{
		cout << "Updating cluster centers\n";
		partition_squared_ = partition_matrix_.array().pow(2);

		clusters_ = (*points_ * partition_squared_.transpose()).cwiseQuotient(
			Eigen::Vector3d::Ones() * partition_squared_.rowwise().sum().transpose()
		);
	}

	void FuzzyClustering3D::updateDistanceMatrix()
	{
		cout << "Updating distance matrix\n";
		Eigen::MatrixXd ones;
		ones.setOnes(1, number_of_points_);
		for(int i = 0;i < number_of_clusters_; i++)
		{
			cout << "loop " << i << "\n";
			distance_matrix_.row(i) = (clusters_.col(i)*ones-*points_).colwise().squaredNorm().eval();
		}
	}

	double FuzzyClustering3D::calculateObjectiveFunction() const
	{
		cout << "Calculating objective function\n";
		RowVectorXd obj_c = distance_matrix_.cwiseProduct(partition_squared_).colwise().sum();
		return obj_c.sum();
	}

	void FuzzyClustering3D::updatePartitionMatrix()
	{
		cout << "Updating partition matrix\n";
		Eigen::MatrixXd distance_inverse = distance_matrix_.cwiseInverse();
		partition_matrix_ = distance_inverse.cwiseQuotient(
			Eigen::MatrixXd::Ones(number_of_clusters_, 1)*distance_inverse.colwise().sum()
		);
	}

};












