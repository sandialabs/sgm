#ifndef FILEMENU_HPP
#define FILEMENU_HPP

#include <QMenu>

class FileMenu : public QMenu
{
Q_OBJECT
public:
    explicit FileMenu(QWidget *parent = Q_NULLPTR);

    ~FileMenu() override = default;

    void add_recent_file(const QString &filename);

signals:

    void open();

    void recent_file(const QString &filename);

    void step();

    void stl();

    void sgm();

    void exit();

private slots:

    void recent_file_triggered();

private:
    QMenu *mRecentFiles;

    void restore_recent_files();
};

#endif // FILEMENU_HPP
