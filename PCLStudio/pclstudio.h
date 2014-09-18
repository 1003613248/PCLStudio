#ifndef PCLSTUDIO_H
#define PCLSTUDIO_H

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

class PCLStudio : public QMainWindow
{
    Q_OBJECT

public:
    PCLStudio();
	~PCLStudio();

private slots:
    void open();
	void openMesh();
    void save();
	void saveAs();
	void close();
	void closeMesh();
	void filter1();
	void filter2();
	void resample1();
	void reconstruct1();
	void reconstruct2();
    void visualize1();
	void visualize2();
	void visualize3();
	void show_help();
	void show_changelog();
	void about();

	void updateStatusBar();

private:
    QTextEdit *textEdit;

    QAction *openAct;
	QAction *openMeshAct;
    QAction *saveAct;
	QAction *saveAsAct;
	QAction *closeAct;
	QAction *closeMeshAct;
	QAction *filterAct1;
	QAction *filterAct2;
	QAction *resampleAct1;
	QAction *reconstructAct1;
	QAction *reconstructAct2;
    QAction *visualizeAct1;
	QAction *visualizeAct2;
	QAction *visualizeAct3;
    QAction *exitAct;
	QAction *helpAct;
	QAction *changelogAct;
	QAction *aboutAct;
	QAction *aboutQtAct;

    QMenu *fileMenu;
	QMenu *filterMenu;
	QMenu *resampleMenu;
	QMenu *reconstructMenu;
    QMenu *visualizeMenu;
	QMenu *helpMenu;

	QStatusBar *status;
	//QLabel *statusLabel;

	QLabel *statusLabel_PointCloud;
	QLabel *statusLabel_PointCloud_fileName;
	QLabel *statusLabel_blank;
	QLabel *statusLabel_Mesh;
	QLabel *statusLabel_Mesh_fileName;

	void createActions();
	void updateActions();
	void createMenus();
	void createStatusBar();
	void createConnections();

	QString current_fileName;
	std::string current_fileName_mesh;
	pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
	pcl::PolygonMesh::Ptr current_mesh;

	void open_new(QString new_fileName);
	void openMesh_new(std::string new_fileName_mesh);
	std::string get_fileExt(std::string fileName);
	//std::string convert_fileName(std::string orig_input_filename);
	std::string convert_Data(std::string fileName);
	void read_Data();
	void save_Data(std::string new_output_filename, pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud);

	void removeNaN(std::string orig_input_filename, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
};

#endif
