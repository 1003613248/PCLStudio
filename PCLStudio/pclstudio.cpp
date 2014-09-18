#include "StdAfx.h"

#include "pclstudio.h"
#include "statistical_filter_dialog.h"
#include "mls_resampling_dialog.h"
#include "greedy_triangulation_dialog.h"
#include "poisson_reconstruction_dialog.h"
#include "helpviewer.h"

PCLStudio::PCLStudio()
{
	setAttribute(Qt::WA_DeleteOnClose, true);

    textEdit = new QTextEdit;
    setCentralWidget(textEdit);

    setWindowTitle(tr("PCLStudio"));
	showMaximized();

	pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PolygonMesh::Ptr current_mesh(new pcl::PolygonMesh);

	createActions();
	createMenus();
	createConnections();

	updateActions();

	createStatusBar();
}

PCLStudio::~PCLStudio()
{
	current_cloud.reset();
	current_mesh.reset();
}

void PCLStudio::createActions()
{
	openAct = new QAction(tr("&Load"), this);
	openMeshAct = new QAction(tr("Load Mesh"), this);
	saveAct = new QAction(tr("&Save"), this);
	saveAsAct = new QAction(tr("Save As"), this);
	closeAct = new QAction(tr("Close"), this);
	closeMeshAct = new QAction(tr("Close Mesh"), this);
	filterAct1 = new QAction(tr("Statistical Filter"), this);
	filterAct2 = new QAction(tr("Remove NaN"), this);
	resampleAct1 = new QAction(tr("MLS Resampling"), this);
	reconstructAct1 = new QAction(tr("Greedy Triangulation"), this);
	reconstructAct2 = new QAction(tr("Possion Reconstruction"), this);
	visualizeAct1 = new QAction(tr("View PointCloud"), this);
	visualizeAct2 = new QAction(tr("View Mesh"), this);
	visualizeAct3 = new QAction(tr("View Mesh with PointCloud"), this);
	exitAct = new QAction(tr("E&xit"), this);

	helpAct = new QAction(tr("Help"), this);
	helpAct->setShortcut(QKeySequence("F1"));
	changelogAct = new QAction(tr("Changelog"), this);
	aboutAct = new QAction(tr("&About"), this);
	aboutQtAct = new QAction(tr("About &Qt"), this);
}

void PCLStudio::updateActions()
{
	saveAct->setEnabled(false);
	saveAsAct->setEnabled(false);
	closeAct->setEnabled(false);
	closeMeshAct->setEnabled(false);
	filterAct1->setEnabled(false);
	filterAct2->setEnabled(false);
	resampleAct1->setEnabled(false);
	reconstructAct1->setEnabled(false);
	reconstructAct2->setEnabled(false);
	visualizeAct1->setEnabled(false);
	visualizeAct2->setEnabled(false);
	visualizeAct3->setEnabled(false);

	if(current_cloud)
	{
		saveAct->setEnabled(true);
		saveAsAct->setEnabled(true);
		closeAct->setEnabled(true);
		filterAct1->setEnabled(true);
		filterAct2->setEnabled(true);
		resampleAct1->setEnabled(true);
		reconstructAct1->setEnabled(true);
		reconstructAct2->setEnabled(true);
		visualizeAct1->setEnabled(true);
	}

	if(current_mesh)
	{
		closeMeshAct->setEnabled(true);
		visualizeAct2->setEnabled(true);
	}

	if(current_cloud && current_mesh)
	{
		visualizeAct3->setEnabled(true);
	}
}

void PCLStudio::createMenus()
{
	fileMenu = menuBar()->addMenu(tr("&File"));
	filterMenu = menuBar()->addMenu(tr("Filter"));
	resampleMenu = menuBar()->addMenu(tr("Resample"));
	reconstructMenu = menuBar()->addMenu(tr("Reconstruct"));
	visualizeMenu = menuBar()->addMenu(tr("&Visualize"));
	fileMenu->addAction(openAct);
	fileMenu->addAction(openMeshAct);
	fileMenu->addAction(saveAct);
	fileMenu->addAction(saveAsAct);
	fileMenu->addAction(closeAct);
	fileMenu->addAction(closeMeshAct);
	fileMenu->addSeparator();
	fileMenu->addAction(exitAct);
	filterMenu->addAction(filterAct1);
	filterMenu->addAction(filterAct2);
	resampleMenu->addAction(resampleAct1);
	reconstructMenu->addAction(reconstructAct1);
	reconstructMenu->addAction(reconstructAct2);
	visualizeMenu->addAction(visualizeAct1);
	visualizeMenu->addAction(visualizeAct2);
	visualizeMenu->addAction(visualizeAct3);

	helpMenu = menuBar()->addMenu(tr("&Help"));
	helpMenu->addAction(helpAct);
	helpMenu->addAction(changelogAct);
	helpMenu->addAction(aboutAct);
	helpMenu->addAction(aboutQtAct);
}

void PCLStudio::createStatusBar()
{
	/*
	statusLabel = new QLabel;
    QString status_string;
    QTextStream(&status_string)<<tr("Current PointCloud: ")<<"        "
                               <<"        "
                               <<tr("Current Mesh: ")<<"        ";
    statusLabel->setText(status_string);
	*/

	statusLabel_PointCloud = new QLabel(tr("Current PointCloud: "));
	statusLabel_PointCloud_fileName = new QLabel(tr("No PointCloud Loaded."));
	statusLabel_blank = new QLabel("                  ");
	statusLabel_Mesh = new QLabel(tr("Current Mesh: "));
	statusLabel_Mesh_fileName = new QLabel(tr("No Mesh Loaded."));

    status = statusBar();
    status->addWidget(statusLabel_PointCloud);
	status->addWidget(statusLabel_PointCloud_fileName);
	status->addWidget(statusLabel_blank);
	status->addWidget(statusLabel_Mesh);
	status->addWidget(statusLabel_Mesh_fileName);
}

void PCLStudio::updateStatusBar()
{
	if((current_cloud) && (!current_fileName.toStdString().empty()))
	//if((!current_fileName.toStdString().empty()))
	{
		// Get filename without path
		QFile file_1(current_fileName);
		QFileInfo fileInfo_1(file_1.fileName());
		QString filename_1(fileInfo_1.fileName());

		statusLabel_PointCloud_fileName->setText(filename_1);
	}
	else
	{
		statusLabel_PointCloud_fileName->setText(tr("No PointCloud Loaded."));
	}

	if((current_mesh) && (!current_fileName_mesh.empty()))
	//if(((!current_fileName_mesh.empty())))
	{
		// Get filename without path
		QString tmp = QString::fromStdString(current_fileName_mesh);
		QFile file_2(tmp);
		QFileInfo fileInfo_2(file_2.fileName());
		QString filename_2(fileInfo_2.fileName());

		statusLabel_Mesh_fileName->setText(filename_2);
	}
	else
	{
		statusLabel_Mesh_fileName->setText(tr("No Mesh Loaded."));
	}
}

void PCLStudio::createConnections()
{
	connect(openAct, SIGNAL(triggered()), this, SLOT(open()));
	connect(openMeshAct, SIGNAL(triggered()), this, SLOT(openMesh()));
	connect(saveAct, SIGNAL(triggered()), this, SLOT(save()));
	connect(saveAsAct, SIGNAL(triggered()), this, SLOT(saveAs()));
	connect(closeAct, SIGNAL(triggered()), this, SLOT(close()));
	connect(closeMeshAct, SIGNAL(triggered()), this, SLOT(closeMesh()));
	connect(filterAct1, SIGNAL(triggered()), this, SLOT(filter1()));
	connect(filterAct2, SIGNAL(triggered()), this, SLOT(filter2()));
	connect(resampleAct1, SIGNAL(triggered()), this, SLOT(resample1()));
	connect(reconstructAct1, SIGNAL(triggered()), this, SLOT(reconstruct1()));
	connect(reconstructAct2, SIGNAL(triggered()), this, SLOT(reconstruct2()));
	connect(visualizeAct1, SIGNAL(triggered()), this, SLOT(visualize1()));
	connect(visualizeAct2, SIGNAL(triggered()), this, SLOT(visualize2()));
	connect(visualizeAct3, SIGNAL(triggered()), this, SLOT(visualize3()));
	connect(exitAct, SIGNAL(triggered()), qApp, SLOT(quit()));

	connect(helpAct, SIGNAL(triggered()), this, SLOT(show_help()));
	connect(changelogAct, SIGNAL(triggered()), this, SLOT(show_changelog()));
	connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));
	connect(aboutQtAct, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
}

void PCLStudio::open()
{
	// Close current file and free memory first 
	close();

    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "",
        tr("PCD Files (*.pcd)\n"
		   "XYZ Files (*.xyz)\n"
		   "PLY Files (*.ply)\n"));

	// Get file extension
	//std::string ext = orig_input_filename.substr(lastindex+1);
	std::string ext = get_fileExt(fileName.toStdString());

	QString new_fileName;
	if(ext=="pcd")
	{
		new_fileName = fileName;
	}
	else if(ext=="xyz"||ext=="ply")
	{
		new_fileName = QString::fromStdString(convert_Data(fileName.toStdString()));
	}

    if (!new_fileName.toStdString().empty())
	{
        QFile file(new_fileName);
		
		current_fileName = new_fileName;

        if (!file.open(QIODevice::ReadOnly)) 
		{
            QMessageBox::critical(this, tr("Error"), tr("Could not open file"));
            return;
        }
		else
		{
			QTextStream in(&file);
			textEdit->setText(in.readAll());
			file.close();
		}
    }

	read_Data();

	updateActions();

	// Test Code: test current_fileName
	//QMessageBox msgBox;
	//msgBox.setText(current_fileName);
	//msgBox.exec();

	updateStatusBar();
}

void PCLStudio::openMesh()
{
	// Close current mesh and free memory first 
	closeMesh();

	QString fileName = QFileDialog::getOpenFileName(this, tr("Open Mesh File"), "",
		tr("VTK Files (*.vtk)\n"
		   "PLY Files (*.ply)\n"
		   "OBJ FIles (*.obj)\n"
		   "STL Files (*stl)\n"));

	// Get file extension
	//std::string ext = orig_input_filename.substr(lastindex+1);
	std::string ext = get_fileExt(fileName.toStdString());

	if (!fileName.toStdString().empty()) 
	{
		QFile file(fileName);
		if (!file.open(QIODevice::ReadOnly)) 
		{
			QMessageBox::critical(this, tr("Error"), tr("Could not open file"));
			return;
		}
		else
		{
			file.close();
		}
	}

	pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);

	if(ext == "vtk")
	{
		pcl::io::loadPolygonFileVTK(fileName.toStdString(), *mesh);
	}
	else if (ext == "ply")
	{
		pcl::io::loadPolygonFilePLY(fileName.toStdString(), *mesh);
	}
	else if (ext == "obj")
	{
		pcl::io::loadPolygonFileOBJ(fileName.toStdString(), *mesh);
	}
	else if (ext == "stl")
	{
		pcl::io::loadPolygonFileSTL(fileName.toStdString(), *mesh);
	}

	current_fileName_mesh = fileName.toStdString();
	current_mesh = mesh;

	updateActions();

	// Test Code: test current_fileName_mesh
	//QMessageBox msgBox;
	//msgBox.setText(QString::fromStdString(current_fileName_mesh));
	//msgBox.exec();

	updateStatusBar();
}

void PCLStudio::open_new(QString new_fileName)
{
	// Close current file and free memory first 
	close();

    if (!new_fileName.toStdString().empty())
	{
        QFile file(new_fileName);

		current_fileName = new_fileName;

        if (!file.open(QIODevice::ReadOnly)) 
		{
            QMessageBox::critical(this, tr("Error"), tr("Could not open file"));
            return;
        }
        else
		{
			QTextStream in(&file);
			textEdit->setText(in.readAll());
			file.close();
		}
    }

	read_Data();

	updateActions();

	updateStatusBar();
}


void PCLStudio::openMesh_new(std::string new_fileName_mesh)
{
	// Close current mesh and free memory first 
	closeMesh();

	// Get file extension
	//std::string ext = orig_input_filename.substr(lastindex+1);
	std::string ext = get_fileExt(new_fileName_mesh);

	if (!new_fileName_mesh.empty()) 
	{
		QFile file(QString::fromStdString(new_fileName_mesh));
		if (!file.open(QIODevice::ReadOnly)) 
		{
			QMessageBox::critical(this, tr("Error"), tr("Could not open file"));
			return;
		}
		else
		{
			file.close();
		}
	}


	pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);

	if(ext == "vtk")
	{
		pcl::io::loadPolygonFileVTK(new_fileName_mesh, *mesh);
	}
	else if (ext == "ply")
	{
		pcl::io::loadPolygonFilePLY(new_fileName_mesh, *mesh);
	}
	else if (ext == "obj")
	{
		pcl::io::loadPolygonFileOBJ(new_fileName_mesh, *mesh);
	}
	else if (ext == "stl")
	{
		pcl::io::loadPolygonFileSTL(new_fileName_mesh, *mesh);
	}

	current_fileName_mesh = new_fileName_mesh;
	current_mesh = mesh;

	updateActions();

	updateStatusBar();
}

void PCLStudio::save()
{
    QString fileName = current_fileName;

    if (fileName != "") {
        QFile file(fileName);

		current_fileName = fileName;

        if (!file.open(QIODevice::WriteOnly)) 
		{
           QMessageBox::critical(this, tr("Error"), tr("Could not save file"));
		   return;
        } 
		else 
		{
            QTextStream stream(&file);
            stream << textEdit->toPlainText();
            stream.flush();
            file.close();
        }
    }
}

void PCLStudio::saveAs()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "",
		tr("PCD Files (*.pcd)"));

	if (fileName != "") {
		QFile file(fileName);

		current_fileName = fileName;

		if (!file.open(QIODevice::WriteOnly)) 
		{
			QMessageBox::critical(this, tr("Error"), tr("Could not save file"));
			return;
		} 
		else 
		{
			QTextStream stream(&file);
			stream << textEdit->toPlainText();
			stream.flush();
			file.close();
		}
	}
}

void PCLStudio::close()
{
	// Clear current file area and reset filenames
	textEdit->clear();
	current_fileName = "";

	// Free memory
	current_cloud.reset();

	updateActions();

	updateStatusBar();
}

void PCLStudio::closeMesh()
{
	// Clear current mesh and reset filenames
	current_fileName_mesh = "";

	// Free memory
	current_mesh.reset();

	updateActions();

	updateStatusBar();
}

std::string PCLStudio::get_fileExt(std::string fileName)
{
	int lastindex = fileName.find_last_of(".");
	std::string fileExt = fileName.substr(lastindex+1);
	return fileExt;
}

std::string PCLStudio::convert_Data(std::string fileName)
{
	// Work with input_filename and output_filename
	int lastindex = fileName.find_last_of("."); 
	std::string input_filename = fileName.substr(0, lastindex);
	std::string output_filename = input_filename + ".pcd";

	// Get file extension
	//std::string ext = orig_input_filename.substr(lastindex+1);
	std::string ext = get_fileExt(fileName);

	if(ext=="xyz")
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointXYZ p;

		// Read text file (ASCII xyz format)
		std::ifstream infile(fileName.c_str());
		if(!infile)
		{
			std::cerr<<"Error: file could not be opened!"<<std::endl;
		}
		else
		{
			while(!infile.eof())
			{
				infile >> p.x >> p.y >> p.z;
				cloud.points.push_back(p);
			}
			cloud.width = cloud.points.size();
			cloud.height = 1; 

			// Remove NaN
			pcl::PointCloud<pcl::PointXYZ> output_cloud;
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(cloud, output_cloud, indices);
			cloud = output_cloud;

			// Ensure cloud is not empty
			if(!cloud.empty())
			{
				pcl::io::savePCDFileASCII (output_filename, cloud);
			}
			//return output_filename;
		}
	} 
	else if(ext=="ply")
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;

		// Load ply file
		if(pcl::io::loadPLYFile(fileName, cloud) == -1)
		{
			std::cout << "fail" << std::endl;
		} 
		else
		{
			// Fix VIEWPOINT
			Eigen::Vector4f sensor_origin;
			sensor_origin<<0,0,0,0;
			cloud.sensor_origin_ = sensor_origin;
			Eigen::Quaternionf sensor_orientation(1,0,0,0);
			cloud.sensor_orientation_ = sensor_orientation;

			// Remove NaN
			pcl::PointCloud<pcl::PointXYZ> output_cloud;
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(cloud, output_cloud, indices);
			cloud = output_cloud;

			// Ensure cloud is not empty
			if(!cloud.empty())
			{
				pcl::io::savePCDFileASCII (output_filename, cloud);
			}
			//return output_filename;
		}
	}
	return output_filename;
}

void PCLStudio::read_Data()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;

	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZ> (current_fileName.toStdString(), *cloud);

	// Ensure cloud is not empty
	//if(cloud->size())
	if(!cloud->empty())
	{
		current_cloud = cloud;
	}
}

void PCLStudio::save_Data(std::string new_output_filename, pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud)
{
	pcl::PCDWriter writer;
	// Ensure cloud is not empty
	//if(new_cloud->size())
	if(!new_cloud->empty())
	{
		writer.write<pcl::PointXYZ> (new_output_filename, *new_cloud, false);
		current_cloud=new_cloud;
	}
}

void PCLStudio::removeNaN(std::string orig_input_filename, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	//read_Data();

	// Handle input_filename and output_filename
	int lastindex = orig_input_filename.find_last_of("."); 
	std::string input_filename = orig_input_filename.substr(0, lastindex);
	std::string output_filename = input_filename + "_nan_removed.pcd";

	// Prepare output cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// Remove NaN
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*input_cloud, *output_cloud, indices);
	cloud_filtered = output_cloud;

	// Save output
	// Ensure cloud is not empty
	//if(cloud_filtered->size())
	if(!cloud_filtered->empty())
	{
		save_Data(output_filename, cloud_filtered);
		open_new(QString::fromStdString(output_filename));
	}
}
void PCLStudio::filter1()
{
	//read_Data();
	
	// Handle input_filename and output_filename
	std::string orig_input_filename(current_fileName.toStdString());
	int lastindex = orig_input_filename.find_last_of("."); 
	std::string input_filename = orig_input_filename.substr(0, lastindex);
	std::string output_filename = input_filename + "_filtered1.pcd";

	// Prepare output cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


	Statistical_Filter_Dialog *statistical_filter_dialog = new Statistical_Filter_Dialog(this);
	statistical_filter_dialog->exec();
	statistical_filter_dialog->statistical_filter(current_cloud, cloud_filtered);

	// Save output
	// Ensure cloud is not empty
	//if(cloud_filtered->size())
	if(!cloud_filtered->empty())
	{
		save_Data(output_filename, cloud_filtered);
		open_new(QString::fromStdString(output_filename));
	}
}

void PCLStudio::filter2()
{
	//read_Data();

	// Handle input_filename and output_filename
	std::string orig_input_filename(current_fileName.toStdString());
	int lastindex = orig_input_filename.find_last_of("."); 
	std::string input_filename = orig_input_filename.substr(0, lastindex);
	std::string output_filename = input_filename + "_filtered2.pcd";

	// Prepare output cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// Remove NaN
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*current_cloud, *output_cloud, indices);
	cloud_filtered = output_cloud;

	// Save output
	// Ensure cloud is not empty
	//if(cloud_filtered->size())
	if(!cloud_filtered->empty())
	{
		save_Data(output_filename, cloud_filtered);
		open_new(QString::fromStdString(output_filename));
	}
}

void PCLStudio::resample1()
{
	//read_Data();
	
	// Handle input_filename and output_filename
	std::string orig_input_filename(current_fileName.toStdString());
	int lastindex = orig_input_filename.find_last_of("."); 
	std::string input_filename = orig_input_filename.substr(0, lastindex);
	std::string output_filename = input_filename + "_resampled.pcd";

	// Prepare output cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointNormal>);

	MLS_Resampling_Dialog *mls_resampling_dialog = new MLS_Resampling_Dialog(this);
	mls_resampling_dialog->exec();
	mls_cloud = mls_resampling_dialog->mls_resampling(current_cloud);

	// Save output
	// Ensure cloud is not empty
	//save_Data(output_filename, cloud_resampled);
	//if(mls_cloud->size())
	if(!mls_cloud->empty())
	{
		pcl::io::savePCDFile (output_filename, *mls_cloud,false);
		open_new(QString::fromStdString(output_filename));
	}
}

void PCLStudio::reconstruct1()
{
	//read_Data();
	
	// Handle input_filename and output_filename
	std::string orig_input_filename(current_fileName.toStdString());
	int lastindex = orig_input_filename.find_last_of("."); 
	std::string input_filename = orig_input_filename.substr(0, lastindex);
	std::string output_filename_vtk = input_filename + "_reconstructed_1.vtk";
	//current_fileName_mesh = output_filename_vtk;

	// Prepare output mesh
	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

	Greedy_Triangulation_Dialog *greedy_triangulation_dialog = new Greedy_Triangulation_Dialog(this);
	greedy_triangulation_dialog->exec();

	triangles = greedy_triangulation_dialog->greedy_triangulation(current_cloud);

	// Ensure the mesh is not empty
	if(!triangles->polygons.empty())
	{
		// Save output
		pcl::io::saveVTKFile(output_filename_vtk, *triangles);
		//viewVTK(output_filename_vtk);
		//open_new(QString::fromStdString(output_filename_vtk));

		current_mesh = triangles;
		current_fileName_mesh = output_filename_vtk;

		updateActions();

		openMesh_new(current_fileName_mesh);

		// Test code: view PointCloud and Mesh at the same time
		//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("View Mesh with PointCloud"));
		//viewer->addPolygonMesh(triangles, "view");
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(current_cloud, 255, 0, 0);
		//viewer->addPointCloud<pcl::PointXYZ> (current_cloud, single_color, "view1");
		//viewer->addPointCloud<pcl::PointXYZ> (current_cloud, "view1");
		//viewer->resetCameraViewpoint("view");
		//while(!viewer->wasStopped())
		//{
		//	viewer->spinOnce(100);
		//	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		//}
	}
}

void PCLStudio::reconstruct2()
{
	//read_Data();
	
	// Handle input_filename and output_filename
	std::string orig_input_filename(current_fileName.toStdString());
	int lastindex = orig_input_filename.find_last_of("."); 
	std::string input_filename = orig_input_filename.substr(0, lastindex);
	std::string output_filename_vtk = input_filename + "_reconstructed_2.vtk";
	//current_fileName_mesh = output_filename_vtk;

	// Prepare output mesh
	pcl::PolygonMesh::Ptr mesh_poisson(new pcl::PolygonMesh);

	Poisson_Reconstruction_Dialog *poisson_reconstruction_dialog = new Poisson_Reconstruction_Dialog(this);
	poisson_reconstruction_dialog->exec();

	mesh_poisson = poisson_reconstruction_dialog->poisson_reconstruction(current_cloud);

	// Ensure the mesh is not empty
	if(!mesh_poisson->polygons.empty())
	{
		// Save output
		pcl::io::saveVTKFile(output_filename_vtk, *mesh_poisson);
		//viewVTK(output_filename_vtk);
		//open_new(QString::fromStdString(output_filename_vtk));

		current_mesh = mesh_poisson;
		current_fileName_mesh = output_filename_vtk;

		updateActions();

		openMesh_new(current_fileName_mesh);

		// Test code: view PointCloud and Mesh at the same time
		//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("View Mesh with PointCloud"));
		//viewer->addPolygonMesh(mesh_poisson, "view");
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(current_cloud, 255, 0, 0);
		//viewer->addPointCloud<pcl::PointXYZ> (current_cloud, single_color, "view1");
		//viewer->addPointCloud<pcl::PointXYZ> (current_cloud, "view1");
		//viewer->resetCameraViewpoint("view");
		//while(!viewer->wasStopped())
		//{
		//	viewer->spinOnce(100);
		//	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		//}
	}
}

void PCLStudio::visualize1()
{
    //read_Data();
	
    //pcl::visualization::CloudViewer *viewer = new pcl::visualization::CloudViewer("Simple Cloud Viewer");
    //viewer->showCloud (current_cloud);
	
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("View PointCloud"));
	//pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("View PointCloud");
	//viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (current_cloud, "view");
	//viewer->resetCameraViewpoint("view");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "view");
	//viewer->addCoordinateSystem (1.0);
	//viewer->initCameraParameters ();
	
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	viewer->removeAllPointClouds();
	viewer.reset();
}

void PCLStudio::visualize2()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("View Mesh"));
	viewer->addPolygonMesh(*current_mesh, "view");
	//viewer->resetCameraViewpoint("view");
	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	viewer->removeAllShapes();
	viewer.reset();
}

void PCLStudio::visualize3()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("View Mesh with PointCloud"));
	viewer->addPolygonMesh(*current_mesh, "view");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(current_cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (current_cloud, single_color, "view1");
	//viewer->resetCameraViewpoint("view");
	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
	viewer.reset();
}

void PCLStudio::show_help()
{
	/*
	QWidget *helpViewer = new QWidget;
	QTextBrowser *helpTextBrowser = new QTextBrowser;

	QFile help_file("help.txt");
	if(!help_file.open(QFile::ReadOnly | QFile::Text))
	{
		QMessageBox::critical(this, tr("Error"), tr("Could not open help file"));
		return;
	}
	QTextStream in(&help_file);
	helpTextBrowser->setPlainText(in.readAll());

	QVBoxLayout *mainLayout = new QVBoxLayout;
	mainLayout->addWidget(helpTextBrowser);
	helpViewer->setLayout(mainLayout);

	helpViewer->resize(500, 400);
	helpViewer->show();
	*/
	HelpViewer *helpViewer = new HelpViewer(tr("Help"));
	helpViewer->showTextHelp("help.txt");
}

void PCLStudio::show_changelog()
{
	HelpViewer *helpViewer = new HelpViewer(tr("Changelog"));
	helpViewer->showTextHelp("changelog.txt");
}

void PCLStudio::about()
{
    QString copyright_symbol(QChar(169));
    QString str;
    QTextStream(&str)<<tr("Simple PCL GUI.\n")
                    <<tr("Version 1.5.4. ")<<tr("Copyright ")<<copyright_symbol<<tr(" Leo <a14331990@163.com>.\n")
					<<tr("Mar 17, 2014.");
    QMessageBox::about(this, tr("About PCLStudio"), str);
}

