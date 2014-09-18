#ifndef STATISTICAL_FILTER_DIALOG_H
#define STATISTICAL_FILTER_DIALOG_H

#include <QDialog>

#include "ui_statistical_filter_dialog.h"
#include "pclstudio.h"

class Statistical_Filter_Dialog : public QDialog, public Ui::Statistical_Filter_Dialog
{
    Q_OBJECT

public:
    Statistical_Filter_Dialog(QWidget *parent = 0);
	void statistical_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);

signals:


private slots:
	void do_statistical_filtering();
	void cancel_statistical_filtering();

private:
	// Statistical Filter Options
	int current_MeanK;
	double current_StddevMul;
	bool do_filtering;
};

#endif
