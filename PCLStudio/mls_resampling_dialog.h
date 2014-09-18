#ifndef MLS_RESAMPLING_DIALOG_H
#define MLS_RESAMPLING_DIALOG_H

#include <QDialog>

#include "ui_mls_resampling_dialog.h"
#include "pclstudio.h"

class MLS_Resampling_Dialog : public QDialog, public Ui::MLS_Resampling_Dialog
{
    Q_OBJECT

public:
    MLS_Resampling_Dialog(QWidget *parent = 0);
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_resampling(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
signals:


private slots:
	void do_mls_resamplinging();
	void cancel_mls_resamplinging();

private:
	// MLS Resampling Options
	int current_PolynomialOrder;
	double current_SearchRadius;
	bool do_resampling;
};

#endif
