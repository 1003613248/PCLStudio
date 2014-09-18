#include "StdAfx.h"

#include <QtGui>

#include "mls_resampling_dialog.h"
#include "pclstudio.h"

MLS_Resampling_Dialog::MLS_Resampling_Dialog(QWidget *parent)
    : QDialog(parent)
{
	//setAttribute(Qt::WA_DeleteOnClose, true);

    setupUi(this);

	spinBox_PolynomialOrder->setValue(2);
	doubleSpinBox_SearchRadius->setValue(30.0);

    connect(pushButton_OK, SIGNAL(clicked()), this, SLOT(do_mls_resamplinging()));
    connect(pushButton_Cancel, SIGNAL(clicked()), this, SLOT(cancel_mls_resamplinging()));
	
	do_resampling=false;
}


void MLS_Resampling_Dialog::do_mls_resamplinging()
{
	do_resampling=true;
	close();
}

void MLS_Resampling_Dialog::cancel_mls_resamplinging()
{
	do_resampling=false;
	close();
}

pcl::PointCloud<pcl::PointNormal>::Ptr MLS_Resampling_Dialog::mls_resampling(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr empty_mls_cloud(new pcl::PointCloud<pcl::PointNormal>);

	if(do_resampling)
	{
		current_PolynomialOrder = spinBox_PolynomialOrder->value();
		current_SearchRadius = doubleSpinBox_SearchRadius->value();

		// Create a KD-Tree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

		// Output has the PointNormal type in order to store the normals calculated by MLS
		pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

		// Init object (second point type is for the normals, even if unused)
		pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal> mls_omp;
 
		mls_omp.setNumberOfThreads(4);

		mls_omp.setComputeNormals (true);

		// Set parameters
		mls_omp.setInputCloud (input_cloud);
		mls_omp.setPolynomialFit (true);
		mls_omp.setPolynomialOrder(current_PolynomialOrder);
		mls_omp.setSearchMethod (tree);
		//orig search radius 0.03, the larger the better? default to 30 now
		mls_omp.setSearchRadius (current_SearchRadius);

		// Reconstruct
		mls_omp.process (*mls_points);
		
		return mls_points;
	}
	else
	{
		return empty_mls_cloud;
	}
}