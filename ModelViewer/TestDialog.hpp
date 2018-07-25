//
// Created by Copps, Kevin D on 7/20/18.
//

#ifndef SGM_TESTDIALOG_H
#define SGM_TESTDIALOG_H

#include <vector>

#include <QDialog>
#include "ModelData.hpp"

class QAction;
class QCheckBox;
class QComboBox;
class QDialogButtonBox;
class QGroupBox;
class QPlainTextEdit;

class TestPrinter;

class TestDialog : public QDialog
{
Q_OBJECT

public:
    TestDialog() = delete;
    explicit TestDialog(ModelData* pModel);

private slots:
    void runTest();

private:

    void createHorizontalGroupBox();
    void createFontStyleSheet();
    void createPlainTextEdit();
    void updateModelView();

    ModelData* mModel;
    TestPrinter* mTestPrinter;
    QString * mFontStyleSheet;

    QGroupBox *horizontalGroupBox;
    QCheckBox *bindModelCheckbox;
    QComboBox *testComboBox;
    QPushButton * runButton;
    QPlainTextEdit *plainTextEdit;
    QDialogButtonBox *buttonBox;
    QStringList *testNamesList;

    QAction *exitAction;

    void populateTests();
};


#endif //SGM_TESTDIALOG_H
