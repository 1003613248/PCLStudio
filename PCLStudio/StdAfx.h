// StdAfx.h

#pragma message("Compiling precompiled headers.\n")

// std headers
#include <iostream>
#include <string>

// Qt headers
//#include <QtGui>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QDir>
#include <QString>
#include <QTextStream>
#include <QMessageBox>
#include <QMainWindow>
#include <QWidget>
#include <QMenuBar>
#include <QStatusBar>
#include <QLabel>
#include <QTextEdit>
#include <QTextBrowser>
#include <QUrl>
#include <QKeySequence>
#include <QCoreApplication>

QT_BEGIN_NAMESPACE
class QVBoxLayout;
class QHBoxLayout;
class QDebug;
class QFile;
class QFileInfo;
class QFileDialog;
class QDir;
class QString;
class QTextStream;
class QMessageBox;
class QMainWindow;
class QWidget;
class QMenuBar;
class QStatusBar;
class QLabel;
class QTextEdit;
class QTextBrowser;
class QUrl;
class QKeySequence;
class QCoreApplication;
QT_END_NAMESPACE

// Boost headers
#include <boost/thread/thread.hpp>

// PCL headers
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>