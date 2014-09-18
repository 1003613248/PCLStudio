#include "StdAfx.h"

#include <QtGui>

#include "poisson_reconstruction_dialog.h"
#include "pclstudio.h"

Poisson_Reconstruction_Dialog::Poisson_Reconstruction_Dialog(QWidget *parent)
	: QDialog(parent)
{
	//setAttribute(Qt::WA_DeleteOnClose, true);

	setupUi(this);

	spinBox_threads->setValue(4);
	doubleSpinBox_NormalSearchRadius->setValue(0.01);
	spinBox_PoissonDepth->setValue(9);

	connect(pushButton_OK, SIGNAL(clicked()), this, SLOT(do_poisson_reconstruction()));
	connect(pushButton_Cancel, SIGNAL(clicked()), this, SLOT(cancel_poisson_reconstruction()));
	
	do_reconstruction=false;
}

void Poisson_Reconstruction_Dialog::do_poisson_reconstruction()
{
	do_reconstruction=true;
	close();
}

void Poisson_Reconstruction_Dialog::cancel_poisson_reconstruction()
{
	do_reconstruction=false;
	close();
}

pcl::PolygonMesh::Ptr Poisson_Reconstruction_Dialog::poisson_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	pcl::PolygonMesh::Ptr empty_mesh(new pcl::PolygonMesh);
	
	if(do_reconstruction)
	{
		current_threads = spinBox_threads->value();
		current_NormalSearchRadius = doubleSpinBox_NormalSearchRadius->value();
		current_PoissonDepth = spinBox_PoissonDepth->value();

		// MLS Smoothing
		pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointXYZ> mls_omp;
		mls_omp.setNumberOfThreads(current_threads);
		mls_omp.setInputCloud (input_cloud);
		mls_omp.setSearchRadius (0.01);
		mls_omp.setPolynomialFit (true);
		mls_omp.setPolynomialOrder (2);
		mls_omp.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
		mls_omp.setUpsamplingRadius (0.005);
		mls_omp.setUpsamplingStepSize (0.003);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
		mls_omp.process (*cloud_smoothed);
		
		// Normal estimation
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_omp;
		ne_omp.setNumberOfThreads (current_threads);
		ne_omp.setInputCloud (cloud_smoothed);
		ne_omp.setRadiusSearch (current_NormalSearchRadius);
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid (*cloud_smoothed, centroid);
		ne_omp.setViewPoint (centroid[0], centroid[1], centroid[2]);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
		ne_omp.compute (*cloud_normals);

		for(size_t i = 0; i < cloud_normals->size (); ++i)
		{
			cloud_normals->points[i].normal_x *= -1;
			cloud_normals->points[i].normal_y *= -1;
			cloud_normals->points[i].normal_z *= -1;
		}

		// Concatenate the XYZ and normal fields
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
		pcl::concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

		// Poisson reconstruction
		pcl::Poisson<pcl::PointNormal> poisson;
		poisson.setDepth (current_PoissonDepth);
		poisson.setInputCloud (cloud_smoothed_normals);
		pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
		poisson.reconstruct (*mesh);

		return mesh;
	}
	else
	{
		return empty_mesh;
	}
}