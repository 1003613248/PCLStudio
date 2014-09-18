#include "StdAfx.h"

#include <QtGui>

#include "statistical_filter_dialog.h"
#include "pclstudio.h"

Statistical_Filter_Dialog::Statistical_Filter_Dialog(QWidget *parent)
    : QDialog(parent)
{
	//setAttribute(Qt::WA_DeleteOnClose, true);

    setupUi(this);

	spinBox_MeanK->setValue(50);
	doubleSpinBox_StddevMul->setValue(1.0);

    connect(pushButton_OK, SIGNAL(clicked()), this, SLOT(do_statistical_filtering()));
    connect(pushButton_Cancel, SIGNAL(clicked()), this, SLOT(cancel_statistical_filtering()));
	
	do_filtering=false;
}


void Statistical_Filter_Dialog::do_statistical_filtering()
{
	do_filtering=true;
	close();
}

void Statistical_Filter_Dialog::cancel_statistical_filtering()
{
	do_filtering=false;
	close();
}

void Statistical_Filter_Dialog::statistical_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
	if(do_filtering)
	{
		current_MeanK = spinBox_MeanK->value();
		current_StddevMul = doubleSpinBox_StddevMul->value();
		
		// Create the filtering object - StatisticalOutlierRemoval filter
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (input_cloud);
		sor.setMeanK (current_MeanK);
		sor.setStddevMulThresh (current_StddevMul);
		sor.filter (*output_cloud);
	}
}