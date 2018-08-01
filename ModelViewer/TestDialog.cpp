#include <vector>
#include <QtWidgets>
#include "TestDialog.hpp"
#include "../Tests/test_utility.h"

#include <gtest/gtest.h>

using testing::EmptyTestEventListener;
using testing::TestCase;
using testing::TestInfo;
using testing::TestPartResult;
using testing::UnitTest;

// Provides alternative output mode to the dialog box by listening for test run events.
class TestPrinter : public EmptyTestEventListener
{
public:
    TestPrinter() = delete;

    explicit TestPrinter(QPlainTextEdit* plainTextEdit)
            : mPlainTextEdit(plainTextEdit)
    {}

private:

    QPlainTextEdit * mPlainTextEdit;
    
    // Append a line to the log composed of two strings, s1 and s2, of the form:
    // "[===x...===]", " y...\n"
    // Color affects s1 only, s2 should end with '\n'
    void appendStatus(int color, const QString &s1, const QString &s2)
    {
        QTextCharFormat tf;
        tf = mPlainTextEdit->currentCharFormat();
        tf.setForeground(QBrush((Qt::GlobalColor)color));
        mPlainTextEdit->moveCursor(QTextCursor::End);
        mPlainTextEdit->setCurrentCharFormat(tf);
        mPlainTextEdit->insertPlainText(s1);
        tf.setForeground(QBrush((Qt::GlobalColor)Qt::black));
        mPlainTextEdit->moveCursor(QTextCursor::End);
        mPlainTextEdit->setCurrentCharFormat(tf);
        mPlainTextEdit->insertPlainText(s2);
    }

    // Append one or more lines to the log in the given color.
    void appendInfo(int color, const QString &s)
    {
        QTextCharFormat tf;
        tf = mPlainTextEdit->currentCharFormat();
        tf.setForeground(QBrush((Qt::GlobalColor)color));
        mPlainTextEdit->moveCursor(QTextCursor::End);
        mPlainTextEdit->setCurrentCharFormat(tf);
        mPlainTextEdit->insertPlainText(s);
    }

    // Called before any test activity starts.
    void OnTestProgramStart(const UnitTest& unit_test) override
    {
        appendStatus(Qt::darkGreen,
                     QString("[==========]"),
                     QString(" Running %1 tests from %2 test cases.\n")
                             .arg(unit_test.test_to_run_count())
                             .arg(unit_test.test_case_to_run_count()));
    }

    // Called when a test case has been run.
    void OnTestCaseEnd(const TestCase &test_case) override
    {
        appendStatus(Qt::darkGreen,
                     QString("[----------]"),
                     QString(" %1 %2 tests (%3 ms total)\n\n")
                             .arg(test_case.name())
                             .arg(test_case.test_to_run_count())
                             .arg(test_case.elapsed_time())
        );
    }

    void OnTestCaseStart(const TestCase &test_case) override
    {
        appendStatus(Qt::darkGreen,
                     QString("[----------]"),
                     QString(" %1 %2 tests\n")
                             .arg(test_case.name())
                             .arg(test_case.test_to_run_count())
        );
    }

    // Called after all test activities have ended.
    void OnTestProgramEnd(const UnitTest& unit_test) override
    {
        appendStatus(Qt::darkGreen,
                     QString("[==========]"),
                     QString(" %1 tests from %2 test case ran. (%3 ms total)\n")
                             .arg(unit_test.test_to_run_count())
                             .arg(unit_test.test_case_to_run_count())
                             .arg(unit_test.elapsed_time()));
        if (unit_test.successful_test_count() > 0)
            appendStatus(Qt::darkGreen,
                         QString("[  PASSED  ]"),
                         QString(" %1 tests.\n").arg(unit_test.successful_test_count()));
        if (unit_test.failed_test_count())
            {
            appendStatus(Qt::red,
                         QString("[  FAILED  ]"),
                         QString(" %1 tests, listed below:\n").arg(unit_test.failed_test_count()));
            for (int i = 0; i < unit_test.total_test_case_count(); ++i)
                {
                    const TestCase * test_case = unit_test.GetTestCase(i);
                    for (int j = 0; j < test_case->total_test_count(); ++j)
                        {
                            const TestInfo* test_info = test_case->GetTestInfo(j);
                            if (test_info->result()->Failed())
                                appendStatus(Qt::red,
                                             QString("[  FAILED  ]"),
                                             QString(" %1.%2\n").arg(test_case->name()).arg(test_info->name()));
                        }
                }
            }

        mPlainTextEdit->repaint();
    }

    // Called before a test starts.
    void OnTestStart(const TestInfo& test_info) override
    {
        appendStatus(Qt::darkGreen,
                     QString("[ RUN      ]"),
                     QString(" %1.%2\n")
                             .arg(test_info.test_case_name())
                             .arg(test_info.name())
        );
        mPlainTextEdit->repaint();
    }

    // Called after a failed assertion or a SUCCEED() invocation.
    void OnTestPartResult(const TestPartResult& test_part_result) override
    {
        QString result = QString("%1 in %2:%3\n%4\n")
                .arg(test_part_result.failed() ? "Failure" : "Success")
                .arg(test_part_result.file_name())
                .arg(test_part_result.line_number())
                .arg(test_part_result.summary());
        appendInfo(Qt::darkGray, result);
        mPlainTextEdit->repaint();
    }

    // Called after a test ends.
    void OnTestEnd(const TestInfo& test_info) override
    {
        int color;
        QString result;
        if (test_info.result()->Passed())
            {
            result = QString("[       OK ]");
            color = Qt::darkGreen;
            }
        else
            {
            result = QString("[  FAILED  ]");
            color = Qt::red;
            }
        QString ending = QString(" %1.%2 (%3 ms)\n")
                .arg(test_info.test_case_name())
                .arg(test_info.name())
                .arg(test_info.result()->elapsed_time());
        appendStatus(color, result, ending);
    }
};

///////////////////////////////////////////////////////////////////////////////
// TestDialog implementation
///////////////////////////////////////////////////////////////////////////////

TestDialog::TestDialog(ModelData* pModel) : mModel(pModel), mTestPrinter(nullptr), mFontStyleSheet(nullptr)
{
    //setMinimumWidth(800);

    testNamesList = new QStringList;
    testComboBox = new QComboBox();

    populateTests();
    createHorizontalGroupBox();

    plainTextEdit = new QPlainTextEdit;
    createPlainTextEdit();
    mTestPrinter = new TestPrinter(plainTextEdit);

    buttonBox = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

    QVBoxLayout *mainLayout = new QVBoxLayout;

    mainLayout->addWidget(horizontalGroupBox);
    mainLayout->addWidget(plainTextEdit);
    mainLayout->addWidget(buttonBox);
    setLayout(mainLayout);

    setWindowTitle(tr("Run Tests"));
}

void TestDialog::createFontStyleSheet()
{
    if (mFontStyleSheet == nullptr)
        {
        QFontDatabase database;
        if (database.hasFamily("Menlo"))
            // a better font for MacOSX
            mFontStyleSheet = new QString("font: 12pt \"Menlo\";");
        else if (database.hasFamily("Consolas"))
            // a better font for Windows10
            mFontStyleSheet = new QString("font: 12pt \"Consolas\";");
        else
            // this is probably available on every platform
            mFontStyleSheet = new QString("font: 12pt \"Courier\";");
        }
}

void TestDialog::createPlainTextEdit()
{
    plainTextEdit->setTextInteractionFlags(Qt::TextSelectableByMouse);
    plainTextEdit->setReadOnly(true);

    createFontStyleSheet();
    plainTextEdit->setStyleSheet(*mFontStyleSheet);

    QPalette palette;
    palette.setColor(QPalette::Base, QColor(226, 226, 226));
    plainTextEdit->setPalette(palette);
}

void TestDialog::createHorizontalGroupBox()
{
    horizontalGroupBox = new QGroupBox(tr("Gtest"));
    QHBoxLayout *layout = new QHBoxLayout;

    runButton = new QPushButton(tr("&Run"));
    connect(runButton, SIGNAL(clicked()), this, SLOT(runTest()));

    bindModelCheckbox = new QCheckBox("B&ind viewer model", this);
    bindModelCheckbox->setChecked(true);

    layout->addWidget(new QLabel(tr("Test Name:")));
    layout->addWidget(testComboBox);
    layout->addWidget(runButton);
    layout->addWidget(bindModelCheckbox);
    layout->addStretch();

    horizontalGroupBox->setLayout(layout);
}

void TestDialog::populateTests()
{
    testing::UnitTest& unit_test = *testing::UnitTest::GetInstance();
    int num_total_test_case = unit_test.total_test_case_count();
    std::vector<int> indices_to_insert_separators;
    int max_contents_length = 0;

    // number of items that will be added to the combo box, including separators
    int items_count = 0;

    // make an item to select all the tests
    testNamesList->push_back(QString("*.*"));
    indices_to_insert_separators.push_back(++items_count);

    // for each test case
    for (int i = 0; i < num_total_test_case; ++i)
        {
        auto test_case = unit_test.GetTestCase(i);
        auto test_case_name = test_case->name();

        // make an item to select all the tests in this test case
        testNamesList->push_back(QString(test_case_name)+ QString(".*"));
        items_count++;

        // make an item for each single test in the test case
        int num_total_test = test_case->total_test_count();
        for (int j = 0; j < num_total_test; ++j)
            {
            auto test_info = test_case->GetTestInfo(j);
            auto test_name = test_info->name();
            QString item = QString(test_case_name) + QString(".") + QString(test_name);
            testNamesList->push_back(item);
            max_contents_length = std::max(max_contents_length,item.length());
            }
        items_count += num_total_test;

        // we want a separator after this test case
        if (i != (num_total_test_case - 1))
            indices_to_insert_separators.push_back(++items_count);
        }
    // add the items and the separators to the combo box
    testComboBox->addItems(*testNamesList);
    for (auto i: indices_to_insert_separators)
        testComboBox->insertSeparator(i);
    testComboBox->setMinimumContentsLength(max_contents_length - 12);
    testComboBox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);
}

void TestDialog::runTest()
{
    SGMInternal::thing * pThing = nullptr;
    if (bindModelCheckbox->isChecked())
        pThing = mModel->GetThing();

    QString qstring_name = testComboBox->currentText();
    std::string string_name(qstring_name.toLatin1().data());
    std::string string_arg("--gtest_filter=");
    string_arg += string_name;

    // run the gtests specified by the given command-line-like arg
    SGMTesting::PerformViewerTest(pThing, string_arg.c_str(), mTestPrinter);

    if (pThing != nullptr)
        updateModelView();

    // disable running (we're not sure how to make the Gtest run more than one session of tests)
    runButton->setEnabled(false);
    testComboBox->setEnabled(false);
    bindModelCheckbox->setEnabled(false);
}

void TestDialog::updateModelView()
{
    // update the model view
    mModel->rebuild_tree();
    mModel->rebuild_graphics();
}
