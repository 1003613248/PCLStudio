#include "StdAfx.h"

#include <QtGui>

#include "greedy_triangulation_dialog.h"
#include "pclstudio.h"

Greedy_Triangulation_Dialog::Greedy_Triangulation_Dialog(QWidget *parent)
    : QDialog(parent)
{
	//setAttribute(Qt::WA_DeleteOnClose, true);

    setupUi(this);

	spinBox_KSearch->setValue(20);
	doubleSpinBox_SearchRadius->setValue(0.025);
	doubleSpinBox_Mu->setValue(2.5);
	spinBox_MaximumNearestNeighbors->setMaximum(10000);
	spinBox_MaximumNearestNeighbors->setValue(100);
	doubleSpinBox_MaximumSurfaceAngle->setSingleStep(45);
	doubleSpinBox_MaximumSurfaceAngle->setMaximum(720);
	doubleSpinBox_MaximumSurfaceAngle->setValue(45);
	doubleSpinBox_MinimumAngle->setSingleStep(10);
	doubleSpinBox_MinimumAngle->setMaximum(720);
	doubleSpinBox_MinimumAngle->setValue(10);
	doubleSpinBox_MaximumAngle->setSingleStep(10);
	doubleSpinBox_MaximumAngle->setMaximum(720);
	doubleSpinBox_MaximumAngle->setValue(120);
	checkBox_NormalConsistency->setChecked(true);
	
    connect(pushButton_OK, SIGNAL(clicked()), this, SLOT(do_greedy_triangulation()));
    connect(pushButton_Cancel, SIGNAL(clicked()), this, SLOT(cancel_greedy_triangulation()));
	
	do_triangulation=false;
}

void Greedy_Triangulation_Dialog::do_greedy_triangulation()
{
	do_triangulation=true;
	close();
}

void Greedy_Triangulation_Dialog::cancel_greedy_triangulation()
{
	do_triangulation=false;
	close();
}

pcl::PolygonMesh::Ptr Greedy_Triangulation_Dialog::greedy_triangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	pcl::PolygonMesh::Ptr empty_triangles(new pcl::PolygonMesh);

	if(do_triangulation)
	{
		current_KSearch = spinBox_KSearch->value();
		current_SearchRadius = doubleSpinBox_SearchRadius->value();
		current_Mu = doubleSpinBox_Mu->value();
		current_MaximumNearestNeighbors = spinBox_MaximumNearestNeighbors->value();
		current_MaximumSurfaceAngle = doubleSpinBox_MaximumSurfaceAngle->value();
		current_MinimumAngle = doubleSpinBox_MinimumAngle->value();
		current_MaximumAngle = doubleSpinBox_MaximumAngle->value();
		current_NormalConsistency = checkBox_NormalConsistency->isChecked();

		// Normal estimation*
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n_omp;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		n_omp.setNumberOfThreads(4);
		tree->setInputCloud (input_cloud);
		n_omp.setInputCloud (input_cloud);
		n_omp.setSearchMethod (tree);
		n_omp.setKSearch (current_KSearch);
		n_omp.compute (*normals);
		//* normals should not contain the point normals + surface curvatures

		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*input_cloud, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals

		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud (cloud_with_normals);

		// Initialize objects
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

		// Set the maximum distance between connected points (maximum edge length)
		gp3.setSearchRadius (current_SearchRadius);

		// Set typical values for the parameters
		gp3.setMu (current_Mu);
		gp3.setMaximumNearestNeighbors (current_MaximumNearestNeighbors);
		gp3.setMaximumSurfaceAngle(current_MaximumSurfaceAngle*M_PI/180); // 45 degrees
		gp3.setMinimumAngle(current_MinimumAngle*M_PI/180); // 10 degrees
		gp3.setMaximumAngle(current_MaximumAngle*M_PI/180); // 120 degrees
		gp3.setNormalConsistency(current_NormalConsistency);

		// Get result
		gp3.setInputCloud (cloud_with_normals);
		gp3.setSearchMethod (tree2);
		gp3.reconstruct (*triangles);

		// Additional vertex information
		std::vector<int> parts = gp3.getPartIDs();
		std::vector<int> states = gp3.getPointStates();

		return triangles;
	}
	else
	{
		return empty_triangles;
	}
}