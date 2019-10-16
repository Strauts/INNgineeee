#include "filebrowser.h"
#include <QDir>
#include <QStringListModel>
#include <QListView>
#include "constants.h"

FileBrowser::FileBrowser(QListView* listview)
{
    QDir dir(QDir::currentPath());
    dir.cd(QString::fromStdString(gsl::assetFilePath));

    currentDir = dir.path();

    setDir(currentDir, listview);
}

void FileBrowser::setDir(const QString &path, QListView* listview)
{
    QDir dir(currentDir);
    dir.cd(path);
    currentDir = dir.path();

    delete listview->model();
    QStringListModel* model = new QStringListModel(listview);

    if(!dir.isReadable())
        return;

    int k = 0;
    QStringList entries = dir.entryList();

    for(int i = 0; i < entries.size(); i++)
    {
        if(entries[i] == ".")
        {
            entries.removeAt(i);
            i = 0;
        }
    }

    model->setStringList(entries);
    listview->setModel(model);
}
