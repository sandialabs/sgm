#include "FileMenu.hpp"

#include <QSettings>

FileMenu::FileMenu(QWidget *parent) :
  QMenu(parent)
{
  setTitle(tr("&File"));
  QAction* open_action = addAction("&Open...", this, SIGNAL(open()));
  open_action->setShortcut(tr("Ctrl+O"));

  mRecentFiles = addMenu("Recent &Files");
  restore_recent_files();

  addSeparator();
  addAction(tr("Exit"), this, SIGNAL(exit()));
}

FileMenu::~FileMenu()
{}

void FileMenu::add_recent_file(const QString &filename)
{
  QSettings settings;

  // Get the current list of recent files
  QStringList file_list;
  int num_files = settings.beginReadArray("FileMenu/recent_files");
  for(int i = 0; i < num_files; i++)
  {
    settings.setArrayIndex(i);
    file_list.push_back( settings.value("name").toString() );
  }
  settings.endArray();

  // Check if the given file is already in the list
  int file_index = file_list.indexOf(filename);
  if(file_index != -1)
  {
    // Move to front of the list
    file_list.move(file_index, 0);
  }
  else
  {
    // Add to the new file to the front of the list
    file_list.prepend(filename);
    if(file_list.size() > 10)
      file_list.pop_back();
  }

  // Write out the new list
  settings.beginWriteArray("FileMenu/recent_files");
  for(int i = 0; i < file_list.size(); i++)
  {
    settings.setArrayIndex(i);
    settings.setValue("name", file_list[i]);
  }
  settings.endArray();

  // Clear and restore the recent file submenu
  mRecentFiles->clear();
  restore_recent_files();
}

void FileMenu::recent_file_triggered()
{
  QAction *action = qobject_cast<QAction *>(sender());
  if (action)
      emit recent_file(action->data().toString());
}

void FileMenu::restore_recent_files()
{
  QSettings settings;

  int num_files = settings.beginReadArray("FileMenu/recent_files");
  for(int i = 0; i < num_files; i++)
  {
    settings.setArrayIndex(i);
    QString filename = settings.value("name").toString();
    QAction* file_action = mRecentFiles->addAction(filename);
    file_action->setData(filename);
    connect(file_action, SIGNAL(triggered()),
            this, SLOT(recent_file_triggered()));
  }
  settings.endArray();

  mRecentFiles->setEnabled(num_files != 0);
}
