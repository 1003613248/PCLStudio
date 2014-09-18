#ifndef HELPVIEWER_H
#define HELPVIEWER_H

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QWidget>
#include <QTextEdit>
#include <QTextBrowser>
#include <QUrl>
#include <QKeySequence>

#include "pclstudio.h"

class HelpViewer : public QWidget
{
    Q_OBJECT

public:
    HelpViewer(QWidget *parent = 0);
	HelpViewer(const QString &title);
	~HelpViewer();

	void setTitle(const QString &title);
	QString getTitle();
	void showTextHelp(const QString &fileName_text);

signals:


private slots:


private:
	QTextBrowser *textBrowser;
	QString help_title;
};

#endif
