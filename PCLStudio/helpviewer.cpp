#include "StdAfx.h"

#include <QtGui>

#include "helpviewer.h"
#include "pclstudio.h"

HelpViewer::HelpViewer(QWidget *parent)
    : QWidget(parent)
{
	setAttribute(Qt::WA_DeleteOnClose, true);

	textBrowser = new QTextBrowser;
	help_title = "";
}

HelpViewer::HelpViewer(const QString &title)
{
	setAttribute(Qt::WA_DeleteOnClose, true);

	textBrowser = new QTextBrowser;
	help_title = title;
}

HelpViewer::~HelpViewer()
{
	delete(textBrowser);
}

void HelpViewer::setTitle(const QString &title)
{
	help_title = title;
}

QString HelpViewer::getTitle()
{
	return help_title;
}

void HelpViewer::showTextHelp(const QString &fileName_text)
{
	QFile help_file(fileName_text);
	if(!help_file.open(QFile::ReadOnly | QFile::Text))
	{
		QMessageBox::critical(this,tr("Error"), tr("Could not open help file"));
		return;
	}
	QTextStream in(&help_file);
	textBrowser->setPlainText(in.readAll());

	QVBoxLayout *mainLayout = new QVBoxLayout;
	mainLayout->addWidget(textBrowser);
	this->setLayout(mainLayout);

	this->setWindowTitle(help_title);
	this->resize(500, 400);
	this->show();
}
