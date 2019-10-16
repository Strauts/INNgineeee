#ifndef FILEBROWSER_H
#define FILEBROWSER_H

#include <QString>

class QListView;
class FileBrowser
{
public:
    FileBrowser(QListView *listview);
    void setDir(const QString &path, QListView* listview);

    QString nameFilter;
    QString currentDir;
};

#endif // FILEBROWSER_H
