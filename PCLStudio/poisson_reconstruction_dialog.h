#ifndef POISSON_RECONSTRUCTION_DIALOG_H
#define POISSON_RECONSTRUCTION_DIALOG_H

#include <QDialog>

#include "ui_poisson_reconstruction_dialog.h"
#include "pclstudio.h"

class Poisson_Reconstruction_Dialog : public QDialog, public Ui::Poisson_Reconstruction_Dialog
{
    Q_OBJECT

public:
    Poisson_Reconstruction_Dialog(QWidget *parent = 0);
	pcl::PolygonMesh::Ptr poisson_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
signals:


private slots:
	void do_poisson_reconstruction();
	void cancel_poisson_reconstruction();

private:
	// Poisson Reconstruction Options
	int current_threads;
	double current_NormalSearchRadius;
	int current_PoissonDepth;

	bool do_reconstruction;
};

#endif
