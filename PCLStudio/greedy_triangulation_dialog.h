#ifndef GREEDY_TRIANGULATION_DIALOG_H
#define GREEDY_TRIANGULATION_DIALOG_H

#include <QDialog>

#include "ui_greedy_triangulation_dialog.h"
#include "pclstudio.h"

class Greedy_Triangulation_Dialog : public QDialog, public Ui::Greedy_Triangulation_Dialog
{
    Q_OBJECT

public:
    Greedy_Triangulation_Dialog(QWidget *parent = 0);
	pcl::PolygonMesh::Ptr greedy_triangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
signals:


private slots:
	void do_greedy_triangulation();
	void cancel_greedy_triangulation();

private:
	// Greedy Triangulation Options
	int current_KSearch;
	double current_SearchRadius;
	double current_Mu;
	int current_MaximumNearestNeighbors;
	double current_MaximumSurfaceAngle;
	double current_MinimumAngle;
	double current_MaximumAngle;
	bool current_NormalConsistency;

	bool do_triangulation;
};

#endif
